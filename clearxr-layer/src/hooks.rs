//! The OpenXR functions the layer intercepts. Each calls the next layer first and then, for hands
//! with a connected bridge controller, replaces the result with the bridge's. The state lock is
//! never held across `xrWaitFrame`, which blocks for frame pacing.

use crate::bindings::{self, Hand, Profile, Source};
use crate::channel::Channel;
use crate::pose::{self, Vec3};
use crate::protocol::{self, tracking, HapticCommand};
use crate::state::{ActionInfo, ActionSpace, ActionValue, BridgePoint, Session, State, Value};
use crate::{layer, log_info, Layer};
use openxr_sys as xr;
use openxr_sys::Handle;
use std::collections::{HashMap, HashSet};
use std::ffi::{c_char, c_void, CString};

/// The device anchor's position in `VIEW` space (PROTOCOL.md, "Head frame"). Zero assumes the
/// two coincide.
const HEAD_OFFSET: Vec3 = Vec3([0.0; 3]);

pub fn lookup(name: &[u8], layer: Option<&Layer>) -> Option<xr::pfn::VoidFunction> {
    macro_rules! hook {
        ($function:expr, $type:ty) => {
            Some(unsafe { std::mem::transmute::<$type, xr::pfn::VoidFunction>($function as $type) })
        };
    }
    let has_locate_spaces = layer.is_some_and(|l| l.next.locate_spaces.is_some());
    let has_locate_spaces_khr = layer.is_some_and(|l| l.next.locate_spaces_khr.is_some());
    match name {
        b"xrGetInstanceProcAddr" => hook!(crate::get_instance_proc_addr, xr::pfn::GetInstanceProcAddr),
        b"xrDestroyInstance" => hook!(crate::destroy_instance, xr::pfn::DestroyInstance),
        b"xrCreateSession" => hook!(create_session, xr::pfn::CreateSession),
        b"xrDestroySession" => hook!(destroy_session, xr::pfn::DestroySession),
        b"xrCreateAction" => hook!(create_action, xr::pfn::CreateAction),
        b"xrSuggestInteractionProfileBindings" => hook!(suggest_bindings, xr::pfn::SuggestInteractionProfileBindings),
        b"xrAttachSessionActionSets" => hook!(attach_action_sets, xr::pfn::AttachSessionActionSets),
        b"xrCreateActionSpace" => hook!(create_action_space, xr::pfn::CreateActionSpace),
        b"xrDestroySpace" => hook!(destroy_space, xr::pfn::DestroySpace),
        b"xrSyncActions" => hook!(sync_actions, xr::pfn::SyncActions),
        b"xrWaitFrame" => hook!(wait_frame, xr::pfn::WaitFrame),
        b"xrGetActionStateBoolean" => hook!(get_action_state_boolean, xr::pfn::GetActionStateBoolean),
        b"xrGetActionStateFloat" => hook!(get_action_state_float, xr::pfn::GetActionStateFloat),
        b"xrGetActionStateVector2f" => hook!(get_action_state_vector2f, xr::pfn::GetActionStateVector2f),
        b"xrGetActionStatePose" => hook!(get_action_state_pose, xr::pfn::GetActionStatePose),
        b"xrGetCurrentInteractionProfile" => hook!(get_current_interaction_profile, xr::pfn::GetCurrentInteractionProfile),
        b"xrPollEvent" => hook!(poll_event, xr::pfn::PollEvent),
        b"xrLocateSpace" => hook!(locate_space, xr::pfn::LocateSpace),
        b"xrLocateSpaces" if has_locate_spaces => hook!(locate_spaces, xr::pfn::LocateSpaces),
        b"xrLocateSpacesKHR" if has_locate_spaces_khr => hook!(locate_spaces_khr, xr::pfn::LocateSpacesKHR),
        b"xrApplyHapticFeedback" => hook!(apply_haptic_feedback, xr::pfn::ApplyHapticFeedback),
        b"xrStopHapticFeedback" => hook!(stop_haptic_feedback, xr::pfn::StopHapticFeedback),
        _ => None,
    }
}

// MARK: Paths

unsafe fn path_string(layer: &Layer, state: &mut State, path: xr::Path) -> Option<String> {
    if path == xr::Path::NULL {
        return None;
    }
    if let Some(string) = state.paths.get(&path.into_raw()) {
        return Some(string.clone());
    }
    let mut buffer = [0u8; xr::MAX_PATH_LENGTH];
    let mut length = 0u32;
    let result = (layer.next.path_to_string)(layer.instance, path, buffer.len() as u32, &mut length, buffer.as_mut_ptr() as *mut c_char);
    if result != xr::Result::SUCCESS || length == 0 {
        return None;
    }
    let string = std::str::from_utf8(&buffer[..length as usize - 1]).ok()?.to_owned();
    state.paths.insert(path.into_raw(), string.clone());
    Some(string)
}

unsafe fn hand_of(layer: &Layer, state: &mut State, path: xr::Path) -> Option<Hand> {
    Hand::from_path(&path_string(layer, state, path)?)
}

unsafe fn string_to_path(layer: &Layer, string: &str) -> xr::Path {
    let string = CString::new(string).unwrap();
    let mut path = xr::Path::NULL;
    (layer.next.string_to_path)(layer.instance, string.as_ptr(), &mut path);
    path
}

// MARK: Sessions, actions, and spaces

unsafe fn reference_space(layer: &Layer, session: xr::Session, ty: xr::ReferenceSpaceType) -> Option<xr::Space> {
    let info = xr::ReferenceSpaceCreateInfo {
        ty: xr::ReferenceSpaceCreateInfo::TYPE,
        next: std::ptr::null(),
        reference_space_type: ty,
        pose_in_reference_space: xr::Posef::IDENTITY,
    };
    let mut space = xr::Space::NULL;
    ((layer.next.create_reference_space)(session, &info, &mut space) == xr::Result::SUCCESS).then_some(space)
}

unsafe extern "system" fn create_session(
    instance: xr::Instance,
    info: *const xr::SessionCreateInfo,
    session: *mut xr::Session,
) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    let result = (layer.next.create_session)(instance, info, session);
    let Some(functions) = layer.channel_functions else { return result };
    if result != xr::Result::SUCCESS {
        return result;
    }
    let session = *session;
    let (Some(view), Some(local)) = (
        reference_space(&layer, session, xr::ReferenceSpaceType::VIEW),
        reference_space(&layer, session, xr::ReferenceSpaceType::LOCAL),
    ) else {
        crate::log_warn("Could not create the reference spaces the bridge needs; it stays inactive for this session");
        return result;
    };
    log_info("Session created; opening the bridge channel");
    let channel = Channel::open(functions, instance, (*info).system_id);
    layer.state.lock().unwrap().sessions.insert(session.into_raw(), Session::new(view, local, Some(channel)));
    result
}

unsafe extern "system" fn destroy_session(session: xr::Session) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    let removed = {
        let mut state = layer.state.lock().unwrap();
        let raw = session.into_raw();
        state.action_spaces.retain(|_, space| space.session != raw);
        state.events.retain(|&s| s != session);
        state.sessions.remove(&raw)
    };
    if let Some(removed) = removed {
        (layer.next.destroy_space)(removed.view_space);
        (layer.next.destroy_space)(removed.local_space);
        // Dropping the session closes its channel.
    }
    (layer.next.destroy_session)(session)
}

unsafe extern "system" fn create_action(
    action_set: xr::ActionSet,
    info: *const xr::ActionCreateInfo,
    action: *mut xr::Action,
) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    let result = (layer.next.create_action)(action_set, info, action);
    if result == xr::Result::SUCCESS {
        layer.state.lock().unwrap().actions.insert(
            (*action).into_raw(),
            ActionInfo { action_set: action_set.into_raw(), action_type: (*info).action_type },
        );
    }
    result
}

unsafe extern "system" fn suggest_bindings(
    instance: xr::Instance,
    suggested: *const xr::InteractionProfileSuggestedBinding,
) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    let mut result = (layer.next.suggest_interaction_profile_bindings)(instance, suggested);
    if suggested.is_null() || layer.channel_functions.is_none() {
        return result;
    }
    let suggested = &*suggested;
    let mut state = layer.state.lock().unwrap();
    let Some(profile_path) = path_string(&layer, &mut state, suggested.interaction_profile) else { return result };
    let Some(profile) = Profile::ALL.into_iter().find(|p| p.path() == profile_path) else { return result };
    if result == xr::Result::ERROR_PATH_UNSUPPORTED {
        // The layer serves this profile itself, so it needs no support from the runtime.
        log_info(&format!("The runtime rejected {profile_path}; the bridge accepts it"));
        result = xr::Result::SUCCESS;
    }
    if result != xr::Result::SUCCESS {
        return result;
    }
    let mut recorded = Vec::new();
    for binding in std::slice::from_raw_parts(suggested.suggested_bindings, suggested.count_suggested_bindings as usize) {
        let Some(path) = path_string(&layer, &mut state, binding.binding) else { continue };
        let Some((hand, component)) = bindings::split_binding(&path) else { continue };
        if let Some(source) = bindings::source(profile, hand, component) {
            recorded.push((binding.action.into_raw(), hand, source));
        }
    }
    log_info(&format!("Recorded {} of {} bindings for {profile_path}", recorded.len(), suggested.count_suggested_bindings));
    state.suggested.insert(profile.path(), recorded);
    result
}

unsafe extern "system" fn attach_action_sets(
    session: xr::Session,
    info: *const xr::SessionActionSetsAttachInfo,
) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    let result = (layer.next.attach_session_action_sets)(session, info);
    if result != xr::Result::SUCCESS || info.is_null() {
        return result;
    }
    let sets: HashSet<u64> = std::slice::from_raw_parts((*info).action_sets, (*info).count_action_sets as usize)
        .iter()
        .map(|set| set.into_raw())
        .collect();
    let mut state = layer.state.lock().unwrap();
    let State { suggested, actions, sessions, .. } = &mut *state;
    let Some(session) = sessions.get_mut(&session.into_raw()) else { return result };
    for profile in Profile::ALL {
        let mut bound: HashMap<(u64, Hand), Vec<Source>> = HashMap::new();
        for &(action, hand, source) in suggested.get(profile.path()).into_iter().flatten() {
            if actions.get(&action).is_some_and(|a| sets.contains(&a.action_set)) {
                bound.entry((action, hand)).or_default().push(source);
            }
        }
        if !bound.is_empty() {
            log_info(&format!("Bridge controllers report {}", profile.path()));
            session.profile = Some(profile);
            session.profile_path = string_to_path(&layer, profile.path());
            session.bindings = bound;
            break;
        }
    }
    if session.profile.is_none() {
        log_info("The application suggested no profile the bridge reports; its controllers stay hidden");
    }
    result
}

unsafe extern "system" fn create_action_space(
    session: xr::Session,
    info: *const xr::ActionSpaceCreateInfo,
    space: *mut xr::Space,
) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    let result = (layer.next.create_action_space)(session, info, space);
    if result != xr::Result::SUCCESS {
        return result;
    }
    let info = &*info;
    let mut state = layer.state.lock().unwrap();
    let subaction = hand_of(&layer, &mut state, info.subaction_path);
    state.action_spaces.insert((*space).into_raw(), ActionSpace {
        session: session.into_raw(),
        action: info.action.into_raw(),
        subaction,
        offset: from_posef(info.pose_in_action_space),
    });
    result
}

unsafe extern "system" fn destroy_space(space: xr::Space) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    layer.state.lock().unwrap().action_spaces.remove(&space.into_raw());
    (layer.next.destroy_space)(space)
}

// MARK: Frame and action sync

unsafe extern "system" fn wait_frame(
    session: xr::Session,
    info: *const xr::FrameWaitInfo,
    frame_state: *mut xr::FrameState,
) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    let result = (layer.next.wait_frame)(session, info, frame_state);
    if result != xr::Result::SUCCESS || frame_state.is_null() {
        return result;
    }
    let now = layer.now();
    if let Some(session) = layer.state.lock().unwrap().sessions.get_mut(&session.into_raw()) {
        session.display_time = (*frame_state).predicted_display_time;
        session.drain(now);
    }
    result
}

unsafe extern "system" fn sync_actions(session: xr::Session, info: *const xr::ActionsSyncInfo) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    let result = (layer.next.sync_actions)(session, info);
    let now = layer.now();
    let mut state = layer.state.lock().unwrap();
    if !state.sessions.contains_key(&session.into_raw()) {
        return result;
    }
    // Action sets synced this time, each limited to one hand or (None) to both.
    let mut active: HashMap<u64, Option<Hand>> = HashMap::new();
    if !info.is_null() {
        for set in std::slice::from_raw_parts((*info).active_action_sets, (*info).count_active_action_sets as usize) {
            let only = if set.subaction_path == xr::Path::NULL {
                None
            } else {
                match hand_of(&layer, &mut state, set.subaction_path) {
                    Some(hand) => Some(hand),
                    None => continue,
                }
            };
            active.entry(set.action_set.into_raw()).and_modify(|e| if *e != only { *e = None }).or_insert(only);
        }
    }
    let State { sessions, actions, events, stats, .. } = &mut *state;
    let bridge = sessions.get_mut(&session.into_raw()).unwrap();
    bridge.drain(now);
    if bridge.sync(actions, &active, result == xr::Result::SUCCESS) {
        log_info(&format!("Bridge controllers: left {}, right {}", bridge.connected[0], bridge.connected[1]));
        events.push_back(session);
    }
    if bridge.connected.contains(&true) {
        stats.log_if_due();
    }
    result
}

// MARK: Action state

/// The bridge's synced values for this query, and whether it names one hand. None leaves the
/// runtime's answer alone.
unsafe fn bridge_values(layer: &Layer, session: xr::Session, info: *const xr::ActionStateGetInfo) -> Option<(Vec<ActionValue>, bool)> {
    if info.is_null() {
        return None;
    }
    let info = &*info;
    let mut state = layer.state.lock().unwrap();
    let one_hand = info.subaction_path != xr::Path::NULL;
    let hands = if one_hand { vec![hand_of(layer, &mut state, info.subaction_path)?] } else { Hand::BOTH.to_vec() };
    let session = state.sessions.get(&session.into_raw())?;
    let values: Vec<ActionValue> = hands.iter()
        .filter_map(|&hand| session.values.get(&(info.action.into_raw(), hand)).copied())
        .collect();
    if !values.is_empty() {
        state.stats.action_queries += 1;
    }
    (!values.is_empty()).then_some((values, one_hand))
}

/// Folds the bridge's values, and the runtime's when the query spans both hands, into one.
fn combine<T: Copy>(
    values: &[ActionValue],
    runtime: Option<(T, bool, xr::Time)>,
    read: impl Fn(Value) -> Option<T>,
    pick: impl Fn(T, T) -> T,
) -> Option<(T, bool, xr::Time)> {
    values.iter()
        .filter_map(|v| read(v.value).map(|value| (value, v.changed, v.last_change_time)))
        .chain(runtime)
        .reduce(|a, b| (pick(a.0, b.0), a.1 || b.1, if b.2.as_nanos() > a.2.as_nanos() { b.2 } else { a.2 }))
}

unsafe extern "system" fn get_action_state_boolean(
    session: xr::Session,
    info: *const xr::ActionStateGetInfo,
    out: *mut xr::ActionStateBoolean,
) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    let result = (layer.next.get_action_state_boolean)(session, info, out);
    if result != xr::Result::SUCCESS || out.is_null() {
        return result;
    }
    let Some((values, one_hand)) = bridge_values(&layer, session, info) else { return result };
    let out = &mut *out;
    let runtime = (!one_hand && out.is_active == xr::TRUE)
        .then_some((out.current_state == xr::TRUE, out.changed_since_last_sync == xr::TRUE, out.last_change_time));
    let read = |v| if let Value::Bool(b) = v { Some(b) } else { None };
    if let Some((current, changed, time)) = combine(&values, runtime, read, |a, b| a || b) {
        out.current_state = current.into();
        out.changed_since_last_sync = changed.into();
        out.last_change_time = time;
        out.is_active = xr::TRUE;
    }
    result
}

unsafe extern "system" fn get_action_state_float(
    session: xr::Session,
    info: *const xr::ActionStateGetInfo,
    out: *mut xr::ActionStateFloat,
) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    let result = (layer.next.get_action_state_float)(session, info, out);
    if result != xr::Result::SUCCESS || out.is_null() {
        return result;
    }
    let Some((values, one_hand)) = bridge_values(&layer, session, info) else { return result };
    let out = &mut *out;
    let runtime = (!one_hand && out.is_active == xr::TRUE)
        .then_some((out.current_state, out.changed_since_last_sync == xr::TRUE, out.last_change_time));
    let read = |v| if let Value::Float(f) = v { Some(f) } else { None };
    if let Some((current, changed, time)) = combine(&values, runtime, read, |a: f32, b: f32| if b.abs() > a.abs() { b } else { a }) {
        out.current_state = current;
        out.changed_since_last_sync = changed.into();
        out.last_change_time = time;
        out.is_active = xr::TRUE;
    }
    result
}

unsafe extern "system" fn get_action_state_vector2f(
    session: xr::Session,
    info: *const xr::ActionStateGetInfo,
    out: *mut xr::ActionStateVector2f,
) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    let result = (layer.next.get_action_state_vector2f)(session, info, out);
    if result != xr::Result::SUCCESS || out.is_null() {
        return result;
    }
    let Some((values, one_hand)) = bridge_values(&layer, session, info) else { return result };
    let out = &mut *out;
    let runtime = (!one_hand && out.is_active == xr::TRUE).then_some((
        [out.current_state.x, out.current_state.y],
        out.changed_since_last_sync == xr::TRUE,
        out.last_change_time,
    ));
    let read = |v| if let Value::Vector(v) = v { Some(v) } else { None };
    let longest = |a: [f32; 2], b: [f32; 2]| if b[0].hypot(b[1]) > a[0].hypot(a[1]) { b } else { a };
    if let Some((current, changed, time)) = combine(&values, runtime, read, longest) {
        out.current_state = xr::Vector2f { x: current[0], y: current[1] };
        out.changed_since_last_sync = changed.into();
        out.last_change_time = time;
        out.is_active = xr::TRUE;
    }
    result
}

unsafe extern "system" fn get_action_state_pose(
    session: xr::Session,
    info: *const xr::ActionStateGetInfo,
    out: *mut xr::ActionStatePose,
) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    let result = (layer.next.get_action_state_pose)(session, info, out);
    if result != xr::Result::SUCCESS || out.is_null() {
        return result;
    }
    if let Some((values, _)) = bridge_values(&layer, session, info) {
        if values.iter().any(|v| v.value == Value::Pose) {
            (*out).is_active = xr::TRUE;
        }
    }
    result
}

unsafe extern "system" fn get_current_interaction_profile(
    session: xr::Session,
    user_path: xr::Path,
    out: *mut xr::InteractionProfileState,
) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    let result = (layer.next.get_current_interaction_profile)(session, user_path, out);
    if result != xr::Result::SUCCESS || out.is_null() {
        return result;
    }
    let mut state = layer.state.lock().unwrap();
    let Some(hand) = hand_of(&layer, &mut state, user_path) else { return result };
    let answered = state.sessions.get(&session.into_raw())
        .is_some_and(|s| s.connected[hand as usize] && s.profile_path != xr::Path::NULL)
        .then(|| state.sessions[&session.into_raw()].profile_path);
    if let Some(profile) = answered {
        (*out).interaction_profile = profile;
        state.stats.profile_queries += 1;
    }
    result
}

unsafe extern "system" fn poll_event(instance: xr::Instance, event: *mut xr::EventDataBuffer) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    if !event.is_null() {
        if let Some(session) = layer.state.lock().unwrap().events.pop_front() {
            let changed = event as *mut xr::EventDataInteractionProfileChanged;
            (*changed).ty = xr::EventDataInteractionProfileChanged::TYPE;
            (*changed).next = std::ptr::null();
            (*changed).session = session;
            return xr::Result::SUCCESS;
        }
    }
    (layer.next.poll_event)(instance, event)
}

// MARK: Poses

/// What locating a pair of spaces needs when either is a connected bridge controller.
struct Plan {
    view_space: xr::Space,
    local_space: xr::Space,
    space: Option<BridgePoint>,
    base: Option<BridgePoint>,
}

fn plan(layer: &Layer, space: xr::Space, base: xr::Space) -> Option<Plan> {
    let state = layer.state.lock().unwrap();
    let point = |space: xr::Space| -> Option<(u64, BridgePoint)> {
        let info = state.action_spaces.get(&space.into_raw())?;
        Some((info.session, state.sessions.get(&info.session)?.bridge_point(info)?))
    };
    let (space, base) = (point(space), point(base));
    let session = state.sessions.get(&space.as_ref().or(base.as_ref())?.0)?;
    Some(Plan {
        view_space: session.view_space,
        local_space: session.local_space,
        space: space.map(|s| s.1),
        base: base.map(|b| b.1),
    })
}

struct Location {
    pose: pose::Pose,
    flags: xr::SpaceLocationFlags,
    velocity: Option<(Vec3, Vec3, xr::SpaceVelocityFlags)>,
}

fn location_flags(point: u16, view: xr::SpaceLocationFlags) -> xr::SpaceLocationFlags {
    let view = view.into_raw() as u16;
    let both = |bit: u16| point & bit != 0 && view & bit != 0;
    let mut flags = 0u16;
    if both(tracking::ORIENTATION_VALID) {
        flags |= tracking::ORIENTATION_VALID;
    }
    if both(tracking::POSITION_VALID) && view & tracking::ORIENTATION_VALID != 0 {
        flags |= tracking::POSITION_VALID;
    }
    if both(tracking::ORIENTATION_TRACKED) {
        flags |= tracking::ORIENTATION_TRACKED;
    }
    if both(tracking::POSITION_TRACKED) {
        flags |= tracking::POSITION_TRACKED;
    }
    xr::SpaceLocationFlags::from_raw(flags as u64)
}

/// Locates a bridge point in `base` by composing it with `VIEW` there, per PROTOCOL.md.
unsafe fn locate_point(layer: &Layer, point: &BridgePoint, view_space: xr::Space, base: xr::Space, time: xr::Time) -> Result<Location, xr::Result> {
    let mut velocity = xr::SpaceVelocity {
        ty: xr::SpaceVelocity::TYPE,
        next: std::ptr::null_mut(),
        velocity_flags: xr::SpaceVelocityFlags::EMPTY,
        linear_velocity: xr::Vector3f::default(),
        angular_velocity: xr::Vector3f::default(),
    };
    let mut view = xr::SpaceLocation {
        ty: xr::SpaceLocation::TYPE,
        next: &mut velocity as *mut _ as *mut c_void,
        location_flags: xr::SpaceLocationFlags::EMPTY,
        pose: xr::Posef::IDENTITY,
    };
    let result = (layer.next.locate_space)(view_space, base, time, &mut view);
    if result != xr::Result::SUCCESS {
        return Err(result);
    }
    let reported = |flag, v: xr::Vector3f| if velocity.velocity_flags.contains(flag) { from_vector(v) } else { Vec3::default() };
    let head = pose::View {
        pose: from_posef(view.pose),
        linear_velocity: reported(xr::SpaceVelocityFlags::LINEAR_VALID, velocity.linear_velocity),
        angular_velocity: reported(xr::SpaceVelocityFlags::ANGULAR_VALID, velocity.angular_velocity),
    };
    let located = pose::locate(&head, &point.controller, HEAD_OFFSET, point.extrapolation(time));
    let flags = location_flags(point.flags, view.location_flags);
    let orientation_valid = view.location_flags.contains(xr::SpaceLocationFlags::ORIENTATION_VALID);
    let mut velocity_flags = 0;
    if orientation_valid && point.flags & tracking::LINEAR_VELOCITY_VALID != 0 {
        velocity_flags |= xr::SpaceVelocityFlags::LINEAR_VALID.into_raw();
    }
    if orientation_valid && point.flags & tracking::ANGULAR_VELOCITY_VALID != 0 {
        velocity_flags |= xr::SpaceVelocityFlags::ANGULAR_VALID.into_raw();
    }
    Ok(Location {
        pose: located.pose,
        flags,
        velocity: Some((located.linear_velocity, located.angular_velocity, xr::SpaceVelocityFlags::from_raw(velocity_flags))),
    })
}

unsafe fn locate_with_runtime(layer: &Layer, space: xr::Space, base: xr::Space, time: xr::Time) -> Result<Location, xr::Result> {
    let mut location = xr::SpaceLocation {
        ty: xr::SpaceLocation::TYPE,
        next: std::ptr::null_mut(),
        location_flags: xr::SpaceLocationFlags::EMPTY,
        pose: xr::Posef::IDENTITY,
    };
    let result = (layer.next.locate_space)(space, base, time, &mut location);
    if result != xr::Result::SUCCESS {
        return Err(result);
    }
    Ok(Location { pose: from_posef(location.pose), flags: location.location_flags, velocity: None })
}

unsafe fn locate_planned(layer: &Layer, plan: &Plan, space: xr::Space, base: xr::Space, time: xr::Time) -> Result<Location, xr::Result> {
    if let (Some(point), None) = (&plan.space, &plan.base) {
        return locate_point(layer, point, plan.view_space, base, time);
    }
    // A bridge base space: locate both in LOCAL and relate them. Velocities relative to a moving
    // controller are not reported.
    let in_local = |point: &Option<BridgePoint>, handle| match point {
        Some(point) => locate_point(layer, point, plan.view_space, plan.local_space, time),
        None => locate_with_runtime(layer, handle, plan.local_space, time),
    };
    let located = in_local(&plan.space, space)?;
    let base_located = in_local(&plan.base, base)?;
    let flags = located.flags.into_raw() & base_located.flags.into_raw();
    Ok(Location {
        pose: base_located.pose.inverse().compose(located.pose),
        flags: xr::SpaceLocationFlags::from_raw(flags),
        velocity: None,
    })
}

fn count_locate(layer: &Layer, flags: xr::SpaceLocationFlags) {
    let mut state = layer.state.lock().unwrap();
    state.stats.locates += 1;
    state.stats.last_flags = flags.into_raw();
}

/// The structure of type `ty` in an output `next` chain.
unsafe fn find_in_chain<T>(mut next: *mut c_void, ty: xr::StructureType) -> Option<*mut T> {
    while !next.is_null() {
        let header = next as *mut xr::BaseOutStructure;
        if (*header).ty == ty {
            return Some(next as *mut T);
        }
        next = (*header).next as *mut c_void;
    }
    None
}

unsafe extern "system" fn locate_space(
    space: xr::Space,
    base: xr::Space,
    time: xr::Time,
    location: *mut xr::SpaceLocation,
) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    let Some(plan) = plan(&layer, space, base).filter(|_| !location.is_null()) else {
        return (layer.next.locate_space)(space, base, time, location);
    };
    let located = match locate_planned(&layer, &plan, space, base, time) {
        Ok(located) => located,
        Err(result) => return result,
    };
    (*location).location_flags = located.flags;
    (*location).pose = to_posef(located.pose);
    count_locate(&layer, located.flags);
    if let Some(velocity) = find_in_chain::<xr::SpaceVelocity>((*location).next, xr::SpaceVelocity::TYPE) {
        let (linear, angular, flags) = located.velocity.unwrap_or_default();
        (*velocity).velocity_flags = flags;
        (*velocity).linear_velocity = to_vector(linear);
        (*velocity).angular_velocity = to_vector(angular);
    }
    xr::Result::SUCCESS
}

unsafe fn locate_many(
    layer: &Layer,
    result: xr::Result,
    info: *const xr::SpacesLocateInfo,
    locations: *mut xr::SpaceLocations,
) -> xr::Result {
    if result != xr::Result::SUCCESS || info.is_null() || locations.is_null() {
        return result;
    }
    let info = &*info;
    let locations = &mut *locations;
    let count = (info.space_count as usize).min(locations.location_count as usize);
    let spaces = std::slice::from_raw_parts(info.spaces, count);
    let velocities = find_in_chain::<xr::SpaceVelocities>(locations.next, xr::SpaceVelocities::TYPE);
    for (i, &space) in spaces.iter().enumerate() {
        let Some(plan) = plan(layer, space, info.base_space) else { continue };
        let located = match locate_planned(layer, &plan, space, info.base_space, info.time) {
            Ok(located) => located,
            Err(error) => return error,
        };
        let location = &mut *locations.locations.add(i);
        count_locate(layer, located.flags);
        location.location_flags = located.flags;
        location.pose = to_posef(located.pose);
        if let Some(velocities) = velocities.filter(|v| i < (**v).velocity_count as usize) {
            let velocity = &mut *(*velocities).velocities.add(i);
            let (linear, angular, flags) = located.velocity.unwrap_or_default();
            velocity.velocity_flags = flags;
            velocity.linear_velocity = to_vector(linear);
            velocity.angular_velocity = to_vector(angular);
        }
    }
    result
}

unsafe extern "system" fn locate_spaces(
    session: xr::Session,
    info: *const xr::SpacesLocateInfo,
    locations: *mut xr::SpaceLocations,
) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    let Some(next) = layer.next.locate_spaces else { return xr::Result::ERROR_FUNCTION_UNSUPPORTED };
    locate_many(&layer, next(session, info, locations), info, locations)
}

unsafe extern "system" fn locate_spaces_khr(
    session: xr::Session,
    info: *const xr::SpacesLocateInfo,
    locations: *mut xr::SpaceLocations,
) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    let Some(next) = layer.next.locate_spaces_khr else { return xr::Result::ERROR_FUNCTION_UNSUPPORTED };
    locate_many(&layer, next(session, info, locations), info, locations)
}

// MARK: Haptics

/// Sends `command` to every connected hand the haptic action drives.
unsafe fn send_haptic(layer: &Layer, session: xr::Session, info: *const xr::HapticActionInfo, command: HapticCommand) {
    if info.is_null() {
        return;
    }
    let info = &*info;
    let mut state = layer.state.lock().unwrap();
    let only = if info.subaction_path == xr::Path::NULL {
        None
    } else {
        match hand_of(layer, &mut state, info.subaction_path) {
            Some(hand) => Some(hand),
            None => return,
        }
    };
    let Some(session) = state.sessions.get_mut(&session.into_raw()) else { return };
    for hand in Hand::BOTH {
        let bound = session.bindings.get(&(info.action.into_raw(), hand)).is_some_and(|s| s.contains(&Source::Haptic));
        if !bound || !session.connected[hand as usize] || only.is_some_and(|h| h != hand) {
            continue;
        }
        if let Some(channel) = session.channel.as_mut() {
            channel.send(&protocol::encode_haptic(hand as u8, command));
        }
    }
}

unsafe extern "system" fn apply_haptic_feedback(
    session: xr::Session,
    info: *const xr::HapticActionInfo,
    feedback: *const xr::HapticBaseHeader,
) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    let result = (layer.next.apply_haptic_feedback)(session, info, feedback);
    if result != xr::Result::SUCCESS || feedback.is_null() || (*feedback).ty != xr::HapticVibration::TYPE {
        return result;
    }
    let vibration = &*(feedback as *const xr::HapticVibration);
    // XR_MIN_HAPTIC_DURATION and any other duration of zero or less ask for the shortest pulse.
    let duration_ns = vibration.duration.as_nanos().max(0) as u64;
    let command = HapticCommand::Apply { duration_ns, frequency: vibration.frequency, amplitude: vibration.amplitude };
    send_haptic(&layer, session, info, command);
    result
}

unsafe extern "system" fn stop_haptic_feedback(session: xr::Session, info: *const xr::HapticActionInfo) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    let result = (layer.next.stop_haptic_feedback)(session, info);
    if result == xr::Result::SUCCESS {
        send_haptic(&layer, session, info, HapticCommand::Stop);
    }
    result
}

// MARK: Conversions

fn from_vector(v: xr::Vector3f) -> Vec3 {
    Vec3([v.x, v.y, v.z])
}

fn to_vector(v: Vec3) -> xr::Vector3f {
    xr::Vector3f { x: v.0[0], y: v.0[1], z: v.0[2] }
}

fn from_posef(p: xr::Posef) -> pose::Pose {
    pose::Pose {
        position: from_vector(p.position),
        orientation: pose::Quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]),
    }
}

fn to_posef(p: pose::Pose) -> xr::Posef {
    let [x, y, z, w] = p.orientation.0;
    xr::Posef { orientation: xr::Quaternionf { x, y, z, w }, position: to_vector(p.position) }
}
