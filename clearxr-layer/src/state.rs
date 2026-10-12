//! What the layer knows about the application's actions and each session's bridge controllers.

use crate::bindings::{self, Hand, Profile, Source};
use crate::channel::Channel;
use crate::pose::{self, Controller, Quat, Vec3};
use crate::protocol::{self, ControllerPacket, FLAG_HEAD_RELATIVE};
use openxr_sys as xr;
use std::collections::{HashMap, VecDeque};
use std::time::{Duration, Instant};

/// A hand is connected while its last packet is younger than this.
pub const STALE: Duration = Duration::from_millis(100);

pub struct ActionInfo {
    pub action_set: u64,
    pub action_type: xr::ActionType,
}

pub struct ActionSpace {
    pub session: u64,
    pub action: u64,
    pub subaction: Option<Hand>,
    pub offset: pose::Pose,
}

#[derive(Default)]
pub struct State {
    pub paths: HashMap<u64, String>,
    pub actions: HashMap<u64, ActionInfo>,
    /// Suggested bindings for the profiles the layer can report: (action, hand, source).
    pub suggested: HashMap<&'static str, Vec<(u64, Hand, Source)>>,
    pub action_spaces: HashMap<u64, ActionSpace>,
    pub sessions: HashMap<u64, Session>,
    /// Sessions owed an `XrEventDataInteractionProfileChanged`.
    pub events: VecDeque<xr::Session>,
    pub stats: Stats,
}

/// What the application asked the bridge for, logged every few seconds while a hand is connected.
#[derive(Default)]
pub struct Stats {
    pub profile_queries: u32,
    pub action_queries: u32,
    pub locates: u32,
    pub last_flags: u64,
    pub last_log: Option<Instant>,
}

impl Stats {
    pub fn log_if_due(&mut self) {
        let now = Instant::now();
        if self.last_log.is_some_and(|t| now.duration_since(t) < Duration::from_secs(2)) {
            return;
        }
        if self.last_log.is_some() {
            crate::log_info(&format!(
                "Answered in 2 s: {} profile queries, {} action queries, {} locates (last flags {:#x})",
                self.profile_queries, self.action_queries, self.locates, self.last_flags));
        }
        *self = Stats { last_log: Some(now), ..Stats::default() };
    }
}

#[derive(Clone, Copy)]
pub struct Received {
    pub packet: ControllerPacket,
    pub at: Instant,
    /// The runtime's time when the packet was read, if it can tell.
    pub time: Option<xr::Time>,
}

/// Accepts packets as PROTOCOL.md's sequence rules describe.
#[derive(Default)]
pub struct Inbox {
    pub latest: Option<Received>,
    last_sequence: Option<u32>,
    rejected: u64,
}

impl Inbox {
    pub fn forget_sequence(&mut self) {
        self.last_sequence = None;
    }

    pub fn receive(&mut self, bytes: &[u8], at: Instant, time: Option<xr::Time>) {
        let packet = match protocol::decode_controller(bytes) {
            Ok(packet) => packet,
            Err(reason) => {
                self.rejected += 1;
                if self.rejected.is_power_of_two() {
                    crate::log_warn(&format!("Rejected {} packets; latest: {reason:?}", self.rejected));
                }
                return;
            }
        };
        if self.latest.is_some_and(|latest| at.duration_since(latest.at) > STALE) {
            self.last_sequence = None;
        }
        if self.last_sequence.is_some_and(|last| !protocol::is_newer(packet.sequence, last)) {
            return;
        }
        if self.latest.is_none() {
            crate::log_info(&format!("First controller packet: active hands {:#04b}", packet.active_hands));
        }
        self.last_sequence = Some(packet.sequence);
        self.latest = Some(Received { packet, at, time });
    }

    /// The latest packet while it is fresh.
    pub fn fresh(&self, now: Instant) -> Option<&Received> {
        self.latest.as_ref().filter(|r| now.duration_since(r.at) < STALE)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Value {
    Bool(bool),
    Float(f32),
    Vector([f32; 2]),
    Pose,
}

#[derive(Clone, Copy, Debug)]
pub struct ActionValue {
    pub value: Value,
    pub changed: bool,
    pub last_change_time: xr::Time,
}

pub struct Session {
    pub view_space: xr::Space,
    pub local_space: xr::Space,
    pub channel: Option<Channel>,
    pub inbox: Inbox,
    pub profile: Option<Profile>,
    pub profile_path: xr::Path,
    /// The chosen profile's bindings, for actions in attached action sets.
    pub bindings: HashMap<(u64, Hand), Vec<Source>>,
    pub connected: [bool; 2],
    /// Bridge action values as of the last `xrSyncActions`, for connected hands only.
    pub values: HashMap<(u64, Hand), ActionValue>,
    pub display_time: xr::Time,
}

impl Session {
    pub fn new(view_space: xr::Space, local_space: xr::Space, channel: Option<Channel>) -> Session {
        Session {
            view_space,
            local_space,
            channel,
            inbox: Inbox::default(),
            profile: None,
            profile_path: xr::Path::NULL,
            bindings: HashMap::new(),
            connected: [false; 2],
            values: HashMap::new(),
            display_time: xr::Time::from_nanos(0),
        }
    }

    /// Reads everything the channel has received.
    pub unsafe fn drain(&mut self, time: Option<xr::Time>) {
        let Some(channel) = self.channel.as_mut() else { return };
        let inbox = &mut self.inbox;
        let now = Instant::now();
        let receive = |bytes: &[u8]| match bytes.get(0..2).map(|m| u16::from_le_bytes([m[0], m[1]])) {
            Some(crate::config::CONFIG_MAGIC) => crate::config::handle(bytes),
            _ => inbox.receive(bytes, now, time),
        };
        match channel.poll(receive) {
            crate::channel::Poll::Unchanged => {}
            _ => inbox.forget_sequence(),
        }
    }

    /// Updates connection state and action values, as `xrSyncActions` does for the runtime's
    /// devices. Returns whether either hand connected or disconnected.
    pub fn sync(&mut self, actions: &HashMap<u64, ActionInfo>, active: &HashMap<u64, Option<Hand>>, synced: bool) -> bool {
        let now = Instant::now();
        let fresh = self.inbox.fresh(now).map(|r| r.packet);
        let connected = Hand::BOTH.map(|hand| {
            self.profile.is_some() && fresh.is_some_and(|packet| packet.is_active(hand as usize))
        });
        let changed = connected != self.connected;
        self.connected = connected;

        let mut values = HashMap::new();
        if let (Some(packet), true) = (fresh, synced) {
            for (&(action, hand), sources) in &self.bindings {
                let Some(info) = actions.get(&action) else { continue };
                let set_active = active.get(&info.action_set).is_some_and(|only| only.is_none_or(|h| h == hand));
                if !connected[hand as usize] || !set_active {
                    continue;
                }
                let Some(value) = evaluate(info.action_type, sources, &packet.hands[hand as usize]) else { continue };
                let previous = self.values.get(&(action, hand));
                let changed = previous.is_some_and(|p| p.value != value);
                let last_change_time = match previous {
                    Some(p) if !changed => p.last_change_time,
                    _ => self.display_time,
                };
                values.insert((action, hand), ActionValue { value, changed, last_change_time });
            }
        }
        self.values = values;
        changed
    }

    /// The pose action space `space` resolves to on a connected hand, from the latest packet.
    pub fn bridge_point(&self, space: &ActionSpace) -> Option<BridgePoint> {
        let hand = space.subaction.or_else(|| {
            Hand::BOTH.into_iter().find(|&h| self.bindings.contains_key(&(space.action, h)))
        })?;
        if !self.connected[hand as usize] {
            return None;
        }
        let at_grip = self.bindings.get(&(space.action, hand))?.iter().find_map(|source| match source {
            Source::GripPose => Some(true),
            Source::AimPose => Some(false),
            _ => None,
        })?;
        let received = self.inbox.latest?;
        let block = received.packet.hands[hand as usize];
        let head_relative = received.packet.flags & FLAG_HEAD_RELATIVE != 0;
        let point = if at_grip { block.grip_pose } else { block.aim_pose };
        let grip = to_pose(block.grip_pose);
        Some(BridgePoint {
            controller: Controller {
                pose: to_pose(point).compose(space.offset),
                grip_position: grip.position,
                linear_velocity: Vec3(block.linear_velocity),
                angular_velocity: Vec3(block.angular_velocity),
            },
            flags: if head_relative { block.usable_tracking_flags() } else { 0 },
            age_at_send_ns: received.packet.send_time_ns.wrapping_sub(block.sample_time_ns) as i64,
            received_time: received.time,
        })
    }
}

/// A connected bridge controller point, ready to compose with the head.
#[derive(Clone, Copy)]
pub struct BridgePoint {
    pub controller: Controller,
    /// The packet's tracking flags, with bits 0–3 equal to `XrSpaceLocationFlags`.
    pub flags: u16,
    /// `sendTimeNs − sampleTimeNs`.
    pub age_at_send_ns: i64,
    pub received_time: Option<xr::Time>,
}

impl BridgePoint {
    /// Seconds to extrapolate the sample to reach `time`.
    pub fn extrapolation(&self, time: xr::Time) -> f32 {
        let since_receive = self.received_time.map_or(0, |r| time.as_nanos() - r.as_nanos());
        (since_receive + self.age_at_send_ns) as f32 / 1e9
    }
}

pub fn to_pose(pose: protocol::Pose) -> pose::Pose {
    pose::Pose { position: Vec3(pose.position), orientation: Quat(pose.orientation) }
}

/// The value an action of `action_type` reads from one hand's sources, combined as OpenXR
/// combines several bindings: any pressed, the largest magnitude, or the longest vector.
pub fn evaluate(action_type: xr::ActionType, sources: &[Source], hand: &protocol::HandBlock) -> Option<Value> {
    match action_type {
        xr::ActionType::BOOLEAN_INPUT => sources.iter()
            .filter_map(|&s| bindings::bool_value(s, hand))
            .reduce(|a, b| a || b)
            .map(Value::Bool),
        xr::ActionType::FLOAT_INPUT => sources.iter()
            .filter_map(|&s| bindings::float_value(s, hand))
            .reduce(|a, b| if b.abs() > a.abs() { b } else { a })
            .map(Value::Float),
        xr::ActionType::VECTOR2F_INPUT => sources.iter()
            .filter_map(|&s| bindings::vector_value(s, hand))
            .reduce(|a, b| if b[0].hypot(b[1]) > a[0].hypot(a[1]) { b } else { a })
            .map(Value::Vector),
        xr::ActionType::POSE_INPUT => sources.iter()
            .any(|s| matches!(s, Source::GripPose | Source::AimPose))
            .then_some(Value::Pose),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::buttons;
    use openxr_sys::Handle;

    fn packet(sequence: u32, active_hands: u8) -> Vec<u8> {
        let mut bytes = vec![0u8; protocol::CONTROLLER_SIZE];
        bytes[0..2].copy_from_slice(&protocol::CONTROLLER_MAGIC.to_le_bytes());
        bytes[2] = protocol::VERSION;
        bytes[3] = active_hands;
        bytes[4] = FLAG_HEAD_RELATIVE as u8;
        bytes[8..12].copy_from_slice(&sequence.to_le_bytes());
        // Right hand: trigger pressed at 0.75.
        bytes[136 + 8..136 + 10].copy_from_slice(&buttons::TRIGGER.to_le_bytes());
        bytes[136 + 24..136 + 28].copy_from_slice(&0.75f32.to_le_bytes());
        bytes
    }

    #[test]
    fn inbox_drops_old_and_duplicate_packets() {
        let mut inbox = Inbox::default();
        let start = Instant::now();
        inbox.receive(&packet(10, 3), start, None);
        inbox.receive(&packet(9, 1), start, None);
        inbox.receive(&packet(10, 1), start, None);
        assert_eq!(inbox.latest.unwrap().packet.active_hands, 3);
        // After 100 ms of silence any sequence is accepted.
        inbox.receive(&packet(2, 1), start + STALE + Duration::from_millis(1), None);
        assert_eq!(inbox.latest.unwrap().packet.sequence, 2);
    }

    #[test]
    fn sync_connects_hands_and_tracks_changes() {
        let mut session = Session::new(xr::Space::NULL, xr::Space::NULL, None);
        session.profile = Some(Profile::OculusTouch);
        let (trigger_click, trigger_value) = (1u64, 2u64);
        session.bindings.insert((trigger_click, Hand::Right), vec![Source::Trigger]);
        session.bindings.insert((trigger_value, Hand::Right), vec![Source::Trigger]);
        let actions = HashMap::from([
            (trigger_click, ActionInfo { action_set: 7, action_type: xr::ActionType::BOOLEAN_INPUT }),
            (trigger_value, ActionInfo { action_set: 7, action_type: xr::ActionType::FLOAT_INPUT }),
        ]);
        let active = HashMap::from([(7u64, None)]);

        session.inbox.receive(&packet(1, 2), Instant::now(), None);
        assert!(session.sync(&actions, &active, true));
        assert_eq!(session.connected, [false, true]);
        assert_eq!(session.values[&(trigger_click, Hand::Right)].value, Value::Bool(true));
        assert_eq!(session.values[&(trigger_value, Hand::Right)].value, Value::Float(0.75));
        assert!(!session.values[&(trigger_click, Hand::Right)].changed);

        let mut released = packet(2, 2);
        released[136 + 8] = 0;
        session.inbox.receive(&released, Instant::now(), None);
        assert!(!session.sync(&actions, &active, true));
        assert_eq!(session.values[&(trigger_click, Hand::Right)].value, Value::Bool(false));
        assert!(session.values[&(trigger_click, Hand::Right)].changed);

        // An inactive action set reports nothing from the bridge.
        session.sync(&actions, &HashMap::new(), true);
        assert!(session.values.is_empty());
    }
}
