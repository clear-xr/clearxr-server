//! Clear XR's OpenXR API layer. It receives the visionOS client's packets over CloudXR's opaque
//! data channel: PSVR2 Sense controller state, which it presents to the game as Oculus Touch (or
//! Khronos simple) controllers (action states, grip and aim poses, the interaction profile, and
//! haptics), and stream configuration, which it writes to the CloudXR runtime's settings.
//! `PROTOCOL.md` specifies the controller packets.

mod bindings;
mod channel;
mod config;
mod hooks;
mod pose;
mod protocol;
mod state;

use openxr_sys as xr;
use openxr_sys::Handle;
use std::ffi::{c_char, c_void, CStr};
use std::sync::{Arc, Mutex, RwLock};

const LAYER_NAME: &str = "XR_APILAYER_CLEARXR_controller_fix";
const TIME_CONVERSION_EXTENSION: &str = "XR_KHR_win32_convert_performance_counter_time";

// Loader interface structures, which openxr-sys does not define.

const STRUCT_LOADER_INFO: u32 = 1;
const STRUCT_API_LAYER_REQUEST: u32 = 2;
const STRUCT_API_LAYER_CREATE_INFO: u32 = 4;
const STRUCT_API_LAYER_NEXT_INFO: u32 = 5;
const LOADER_INFO_VERSION: u32 = 1;
const API_LAYER_REQUEST_VERSION: u32 = 1;
const LOADER_LAYER_INTERFACE_VERSION: u32 = 1;
const MAX_LAYER_NAME: usize = 256;
const MAX_SETTINGS_PATH: usize = 512;

#[repr(C)]
pub struct NegotiateLoaderInfo {
    struct_type: u32,
    struct_version: u32,
    struct_size: usize,
    min_interface_version: u32,
    max_interface_version: u32,
    min_api_version: xr::Version,
    max_api_version: xr::Version,
}

#[repr(C)]
pub struct NegotiateApiLayerRequest {
    struct_type: u32,
    struct_version: u32,
    struct_size: usize,
    layer_interface_version: u32,
    layer_api_version: xr::Version,
    get_instance_proc_addr: xr::pfn::GetInstanceProcAddr,
    create_api_layer_instance: CreateApiLayerInstance,
}

type CreateApiLayerInstance =
    unsafe extern "system" fn(*const xr::InstanceCreateInfo, *const ApiLayerCreateInfo, *mut xr::Instance) -> xr::Result;

#[repr(C)]
struct ApiLayerCreateInfo {
    struct_type: u32,
    struct_version: u32,
    struct_size: usize,
    loader_instance: *mut c_void,
    settings_file_location: [c_char; MAX_SETTINGS_PATH],
    next_info: *const ApiLayerNextInfo,
}

#[repr(C)]
struct ApiLayerNextInfo {
    struct_type: u32,
    struct_version: u32,
    struct_size: usize,
    layer_name: [c_char; MAX_LAYER_NAME],
    next_get_instance_proc_addr: xr::pfn::GetInstanceProcAddr,
    next_create_api_layer_instance: CreateApiLayerInstance,
    next: *const ApiLayerNextInfo,
}

// Logging: the `log` crate (stderr through env_logger), `%LOCALAPPDATA%\ClearXR\layer.log`, and,
// on Windows, OutputDebugString.

fn log_file() -> &'static Mutex<Option<std::fs::File>> {
    static FILE: std::sync::OnceLock<Mutex<Option<std::fs::File>>> = std::sync::OnceLock::new();
    FILE.get_or_init(|| {
        // Unit tests run on developers' machines and must not write to the user's log.
        if cfg!(test) {
            return Mutex::new(None);
        }
        let file = std::env::var_os("LOCALAPPDATA").and_then(|dir| {
            let dir = std::path::PathBuf::from(dir).join("ClearXR");
            std::fs::create_dir_all(&dir).ok()?;
            std::fs::OpenOptions::new().create(true).append(true).open(dir.join("layer.log")).ok()
        });
        Mutex::new(file)
    })
}

fn log(level: log::Level, message: &str) {
    log::log!(level, "[ClearXR Layer] {message}");
    if let Some(file) = log_file().lock().unwrap().as_mut() {
        use std::io::Write;
        let since_epoch = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default();
        let _ = writeln!(file, "{}.{:03} pid {} {level} {message}",
            since_epoch.as_secs(), since_epoch.subsec_millis(), std::process::id());
    }
    #[cfg(windows)]
    unsafe {
        extern "system" {
            fn OutputDebugStringA(text: *const c_char);
        }
        let mut bytes: Vec<u8> = format!("[ClearXR Layer] {message}\n").into_bytes();
        bytes.retain(|b| *b != 0);
        bytes.push(0);
        OutputDebugStringA(bytes.as_ptr() as *const c_char);
    }
}

pub(crate) fn log_info(message: &str) {
    log(log::Level::Info, message);
}

pub(crate) fn log_warn(message: &str) {
    log(log::Level::Warn, message);
}

// Dispatch to the next layer or the runtime.

type ConvertTime = unsafe extern "system" fn(xr::Instance, *const i64, *mut xr::Time) -> xr::Result;

pub(crate) struct Next {
    pub get_instance_proc_addr: xr::pfn::GetInstanceProcAddr,
    pub destroy_instance: xr::pfn::DestroyInstance,
    pub create_session: xr::pfn::CreateSession,
    pub destroy_session: xr::pfn::DestroySession,
    pub create_reference_space: xr::pfn::CreateReferenceSpace,
    pub create_action_space: xr::pfn::CreateActionSpace,
    pub destroy_space: xr::pfn::DestroySpace,
    pub create_action: xr::pfn::CreateAction,
    pub suggest_interaction_profile_bindings: xr::pfn::SuggestInteractionProfileBindings,
    pub attach_session_action_sets: xr::pfn::AttachSessionActionSets,
    pub sync_actions: xr::pfn::SyncActions,
    pub get_action_state_boolean: xr::pfn::GetActionStateBoolean,
    pub get_action_state_float: xr::pfn::GetActionStateFloat,
    pub get_action_state_vector2f: xr::pfn::GetActionStateVector2f,
    pub get_action_state_pose: xr::pfn::GetActionStatePose,
    pub get_current_interaction_profile: xr::pfn::GetCurrentInteractionProfile,
    pub locate_space: xr::pfn::LocateSpace,
    pub locate_spaces: Option<xr::pfn::LocateSpaces>,
    pub locate_spaces_khr: Option<xr::pfn::LocateSpacesKHR>,
    pub poll_event: xr::pfn::PollEvent,
    pub wait_frame: xr::pfn::WaitFrame,
    pub apply_haptic_feedback: xr::pfn::ApplyHapticFeedback,
    pub stop_haptic_feedback: xr::pfn::StopHapticFeedback,
    pub path_to_string: xr::pfn::PathToString,
    pub string_to_path: xr::pfn::StringToPath,
}

unsafe fn load_optional<T>(gpa: xr::pfn::GetInstanceProcAddr, instance: xr::Instance, name: &str) -> Option<T> {
    let name = format!("{name}\0");
    let mut function: Option<xr::pfn::VoidFunction> = None;
    gpa(instance, name.as_ptr() as *const c_char, &mut function);
    function.map(|f| std::mem::transmute_copy(&f))
}

unsafe fn load<T>(gpa: xr::pfn::GetInstanceProcAddr, instance: xr::Instance, name: &str) -> Result<T, String> {
    load_optional(gpa, instance, name).ok_or_else(|| format!("the next layer lacks {name}"))
}

impl Next {
    unsafe fn load(gpa: xr::pfn::GetInstanceProcAddr, instance: xr::Instance) -> Result<Next, String> {
        Ok(Next {
            get_instance_proc_addr: gpa,
            destroy_instance: load(gpa, instance, "xrDestroyInstance")?,
            create_session: load(gpa, instance, "xrCreateSession")?,
            destroy_session: load(gpa, instance, "xrDestroySession")?,
            create_reference_space: load(gpa, instance, "xrCreateReferenceSpace")?,
            create_action_space: load(gpa, instance, "xrCreateActionSpace")?,
            destroy_space: load(gpa, instance, "xrDestroySpace")?,
            create_action: load(gpa, instance, "xrCreateAction")?,
            suggest_interaction_profile_bindings: load(gpa, instance, "xrSuggestInteractionProfileBindings")?,
            attach_session_action_sets: load(gpa, instance, "xrAttachSessionActionSets")?,
            sync_actions: load(gpa, instance, "xrSyncActions")?,
            get_action_state_boolean: load(gpa, instance, "xrGetActionStateBoolean")?,
            get_action_state_float: load(gpa, instance, "xrGetActionStateFloat")?,
            get_action_state_vector2f: load(gpa, instance, "xrGetActionStateVector2f")?,
            get_action_state_pose: load(gpa, instance, "xrGetActionStatePose")?,
            get_current_interaction_profile: load(gpa, instance, "xrGetCurrentInteractionProfile")?,
            locate_space: load(gpa, instance, "xrLocateSpace")?,
            locate_spaces: load_optional(gpa, instance, "xrLocateSpaces"),
            locate_spaces_khr: load_optional(gpa, instance, "xrLocateSpacesKHR"),
            poll_event: load(gpa, instance, "xrPollEvent")?,
            wait_frame: load(gpa, instance, "xrWaitFrame")?,
            apply_haptic_feedback: load(gpa, instance, "xrApplyHapticFeedback")?,
            stop_haptic_feedback: load(gpa, instance, "xrStopHapticFeedback")?,
            path_to_string: load(gpa, instance, "xrPathToString")?,
            string_to_path: load(gpa, instance, "xrStringToPath")?,
        })
    }
}

pub(crate) struct Layer {
    pub instance: xr::Instance,
    pub next: Next,
    /// None when the runtime has no opaque data channel; every call then passes through.
    pub channel_functions: Option<channel::Functions>,
    convert_time: Option<ConvertTime>,
    pub state: Mutex<state::State>,
}

impl Layer {
    /// The runtime's current `XrTime`, when it supports converting the performance counter.
    pub fn now(&self) -> Option<xr::Time> {
        #[cfg(windows)]
        unsafe {
            extern "system" {
                fn QueryPerformanceCounter(count: *mut i64) -> i32;
            }
            let convert = self.convert_time?;
            let mut counter = 0i64;
            QueryPerformanceCounter(&mut counter);
            let mut time = xr::Time::from_nanos(0);
            return (convert(self.instance, &counter, &mut time) == xr::Result::SUCCESS).then_some(time);
        }
        #[cfg(not(windows))]
        {
            let _ = self.convert_time;
            None
        }
    }
}

static LAYER: RwLock<Option<Arc<Layer>>> = RwLock::new(None);
static NEXT_GPA: Mutex<Option<xr::pfn::GetInstanceProcAddr>> = Mutex::new(None);

pub(crate) fn layer() -> Option<Arc<Layer>> {
    LAYER.read().unwrap().clone()
}

#[no_mangle]
pub unsafe extern "system" fn xrNegotiateLoaderApiLayerInterface(
    loader_info: *const NegotiateLoaderInfo,
    _layer_name: *const c_char,
    request: *mut NegotiateApiLayerRequest,
) -> xr::Result {
    let _ = env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .format_timestamp_millis()
        .try_init();
    if loader_info.is_null() || request.is_null() {
        return xr::Result::ERROR_INITIALIZATION_FAILED;
    }
    let info = &*loader_info;
    let request = &mut *request;
    if info.struct_type != STRUCT_LOADER_INFO
        || info.struct_version != LOADER_INFO_VERSION
        || info.struct_size != std::mem::size_of::<NegotiateLoaderInfo>()
        || request.struct_type != STRUCT_API_LAYER_REQUEST
        || request.struct_version != API_LAYER_REQUEST_VERSION
        || request.struct_size != std::mem::size_of::<NegotiateApiLayerRequest>()
        || !(info.min_interface_version..=info.max_interface_version).contains(&LOADER_LAYER_INTERFACE_VERSION)
    {
        log_warn("The loader's interface does not match this layer");
        return xr::Result::ERROR_INITIALIZATION_FAILED;
    }
    request.layer_interface_version = LOADER_LAYER_INTERFACE_VERSION;
    request.layer_api_version = xr::Version::new(1, 0, 0);
    request.get_instance_proc_addr = get_instance_proc_addr;
    request.create_api_layer_instance = create_api_layer_instance;
    xr::Result::SUCCESS
}

unsafe extern "system" fn create_api_layer_instance(
    create_info: *const xr::InstanceCreateInfo,
    layer_info: *const ApiLayerCreateInfo,
    instance_out: *mut xr::Instance,
) -> xr::Result {
    if create_info.is_null() || layer_info.is_null() || instance_out.is_null() {
        return xr::Result::ERROR_INITIALIZATION_FAILED;
    }
    let layer_info = &*layer_info;
    if layer_info.struct_type != STRUCT_API_LAYER_CREATE_INFO
        || layer_info.struct_size != std::mem::size_of::<ApiLayerCreateInfo>()
    {
        log_warn("Unexpected ApiLayerCreateInfo from the loader");
        return xr::Result::ERROR_INITIALIZATION_FAILED;
    }

    // Find this layer's entry in the chain.
    let mut entry = layer_info.next_info;
    while !entry.is_null() && (*entry).struct_type == STRUCT_API_LAYER_NEXT_INFO {
        if CStr::from_ptr((*entry).layer_name.as_ptr()).to_bytes() == LAYER_NAME.as_bytes() {
            break;
        }
        entry = (*entry).next;
    }
    if entry.is_null() || (*entry).struct_type != STRUCT_API_LAYER_NEXT_INFO {
        log_warn(&format!("{LAYER_NAME} is missing from the loader's layer chain"));
        return xr::Result::ERROR_INITIALIZATION_FAILED;
    }
    let entry = &*entry;
    let gpa = entry.next_get_instance_proc_addr;
    *NEXT_GPA.lock().unwrap() = Some(gpa);

    // Enable the opaque data channel, preferring NVX1 when both exist, and the time conversion the
    // pose extrapolation uses, on top of what the application asked for. The extensions come from
    // the bottom of the chain: a layer below this one may not answer before its own instance
    // exists.
    let mut bottom = entry;
    while !bottom.next.is_null() && (*bottom.next).struct_type == STRUCT_API_LAYER_NEXT_INFO {
        bottom = &*bottom.next;
    }
    let available = available_extensions(bottom.next_get_instance_proc_addr);
    log_info(&format!("Runtime extensions: {}", available.len()));
    let create_info = &*create_info;
    let requested: Vec<*const c_char> = (0..create_info.enabled_extension_count as usize)
        .map(|i| *create_info.enabled_extension_names.add(i))
        .collect();
    let is_requested = |name: &str| requested.iter().any(|&p| CStr::from_ptr(p).to_bytes() == name.as_bytes());
    let opaque = channel::EXTENSIONS.into_iter().rev().find(|name| available.iter().any(|a| a == name));
    let time_conversion = available.iter().any(|a| a == TIME_CONVERSION_EXTENSION);
    let added: Vec<String> = opaque.into_iter()
        .chain(time_conversion.then_some(TIME_CONVERSION_EXTENSION))
        .filter(|name| !is_requested(name))
        .map(|name| format!("{name}\0"))
        .collect();
    let mut extensions = requested.clone();
    extensions.extend(added.iter().map(|name| name.as_ptr() as *const c_char));

    let mut modified = *create_info;
    modified.enabled_extension_count = extensions.len() as u32;
    modified.enabled_extension_names = extensions.as_ptr();
    let next_layer_info = ApiLayerCreateInfo {
        struct_type: layer_info.struct_type,
        struct_version: layer_info.struct_version,
        struct_size: layer_info.struct_size,
        loader_instance: layer_info.loader_instance,
        settings_file_location: layer_info.settings_file_location,
        next_info: entry.next,
    };
    let result = (entry.next_create_api_layer_instance)(&modified, &next_layer_info, instance_out);
    if result != xr::Result::SUCCESS {
        return result;
    }

    let instance = *instance_out;
    let next = match Next::load(gpa, instance) {
        Ok(next) => next,
        Err(error) => {
            log_warn(&format!("Layer disabled: {error}"));
            return result;
        }
    };
    let channel_functions = if opaque.is_some() { channel::Functions::load(gpa, instance) } else { None };
    match &channel_functions {
        Some(functions) => log_info(&format!("Active; opaque data channel entry points use the {} suffix", functions.suffix)),
        None => log_info("Inactive: the runtime has no opaque data channel, so every call passes through"),
    }
    let convert_time = if time_conversion {
        load_optional(gpa, instance, "xrConvertWin32PerformanceCounterToTimeKHR")
    } else {
        log_warn("The runtime cannot convert the performance counter to XrTime; poses are extrapolated less");
        None
    };
    *LAYER.write().unwrap() = Some(Arc::new(Layer {
        instance,
        next,
        channel_functions,
        convert_time,
        state: Mutex::new(state::State::default()),
    }));
    result
}

unsafe fn available_extensions(gpa: xr::pfn::GetInstanceProcAddr) -> Vec<String> {
    let Some(enumerate) = load_optional::<xr::pfn::EnumerateInstanceExtensionProperties>(
        gpa, xr::Instance::NULL, "xrEnumerateInstanceExtensionProperties")
    else {
        return Vec::new();
    };
    let mut count = 0u32;
    if enumerate(std::ptr::null(), 0, &mut count, std::ptr::null_mut()) != xr::Result::SUCCESS {
        return Vec::new();
    }
    let empty = xr::ExtensionProperties {
        ty: xr::ExtensionProperties::TYPE,
        next: std::ptr::null_mut(),
        extension_name: [0; xr::MAX_EXTENSION_NAME_SIZE],
        extension_version: 0,
    };
    let mut properties = vec![empty; count as usize];
    if enumerate(std::ptr::null(), count, &mut count, properties.as_mut_ptr()) != xr::Result::SUCCESS {
        return Vec::new();
    }
    properties.iter()
        .take(count as usize)
        .filter_map(|p| CStr::from_ptr(p.extension_name.as_ptr()).to_str().ok().map(str::to_owned))
        .collect()
}

pub(crate) unsafe extern "system" fn get_instance_proc_addr(
    instance: xr::Instance,
    name: *const c_char,
    function: *mut Option<xr::pfn::VoidFunction>,
) -> xr::Result {
    if name.is_null() || function.is_null() {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }
    let name = CStr::from_ptr(name);
    if let Some(hook) = hooks::lookup(name.to_bytes(), layer().as_deref()) {
        *function = Some(hook);
        return xr::Result::SUCCESS;
    }
    if let Some(layer) = layer() {
        return (layer.next.get_instance_proc_addr)(instance, name.as_ptr(), function);
    }
    match *NEXT_GPA.lock().unwrap() {
        Some(gpa) => gpa(instance, name.as_ptr(), function),
        None => xr::Result::ERROR_HANDLE_INVALID,
    }
}

pub(crate) unsafe extern "system" fn destroy_instance(instance: xr::Instance) -> xr::Result {
    let Some(layer) = layer() else { return xr::Result::ERROR_HANDLE_INVALID };
    let result = (layer.next.destroy_instance)(instance);
    *LAYER.write().unwrap() = None;
    result
}
