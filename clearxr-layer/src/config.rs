//! Stream configuration packets the headset sends on the Clear XR channel: a 6-byte header
//! (magic "CF", little-endian u32 JSON length) followed by UTF-8 JSON. The layer writes the
//! settings to `Server/cloudxr-runtime.yaml` next to this DLL and to `clearxr-settings.json`.

use serde::Deserialize;
use std::path::PathBuf;

pub const CONFIG_MAGIC: u16 = 0x4346; // "CF"
const HEADER_SIZE: usize = 6;

/// Parsed stream configuration received from the headset.
#[derive(Clone, Debug, Deserialize)]
pub struct StreamConfig {
    #[serde(rename = "RenderedResolution")]
    pub rendered_resolution: i64,
    #[serde(rename = "EncodedResolution")]
    pub encoded_resolution: i64,
    #[serde(rename = "FoveationInsetRatio")]
    pub foveation_inset_ratio: f64,
    #[serde(rename = "DefaultAppEnabled")]
    pub default_app_enabled: bool,
    #[serde(rename = "AlphaTransparencyEnabled")]
    pub alpha_transparency_enabled: bool,
}

/// Parses a configuration packet and applies it.
pub fn handle(bytes: &[u8]) {
    if bytes.len() < HEADER_SIZE {
        crate::log_warn(&format!("Config packet too short ({} bytes)", bytes.len()));
        return;
    }
    let json_len = u32::from_le_bytes([bytes[2], bytes[3], bytes[4], bytes[5]]) as usize;
    let Some(json_bytes) = bytes.get(HEADER_SIZE..HEADER_SIZE + json_len) else {
        crate::log_warn(&format!(
            "Config packet truncated: expected {} bytes, got {}",
            HEADER_SIZE + json_len,
            bytes.len()
        ));
        return;
    };
    let json = match std::str::from_utf8(json_bytes) {
        Ok(json) => json,
        Err(e) => {
            crate::log_warn(&format!("Config JSON is not valid UTF-8: {e}"));
            return;
        }
    };
    crate::log_info(&format!("Received stream config: {json}"));
    match serde_json::from_str::<StreamConfig>(json) {
        Ok(config) => apply(&config),
        Err(e) => crate::log_warn(&format!("Failed to parse stream config JSON: {e}")),
    }
}

fn apply(config: &StreamConfig) {
    match get_dll_dir() {
        Some(dll_dir) => {
            let yaml_path = dll_dir.join("Server").join("cloudxr-runtime.yaml");
            match update_runtime_yaml(&yaml_path, config) {
                Ok(()) => crate::log_info(&format!("Updated {}", yaml_path.display())),
                Err(e) => crate::log_warn(&format!("Failed to update {}: {e}", yaml_path.display())),
            }
        }
        None => crate::log_warn("Could not determine DLL directory for YAML update"),
    }
    match update_settings_json(config.default_app_enabled) {
        Ok(path) => crate::log_info(&format!(
            "Updated default app setting to {} in {}",
            config.default_app_enabled,
            path.display()
        )),
        Err(e) => crate::log_warn(&format!("Failed to update settings: {e}")),
    }
}

fn update_runtime_yaml(path: &std::path::Path, config: &StreamConfig) -> Result<(), String> {
    let contents = std::fs::read_to_string(path).map_err(|e| format!("read {}: {}", path.display(), e))?;
    let mut doc: serde_yaml::Value =
        serde_yaml::from_str(&contents).map_err(|e| format!("parse {}: {}", path.display(), e))?;

    let inset = (config.foveation_inset_ratio * 100.0).round() as i64;
    set_yaml_value(&mut doc, "runtimeFoveationUnwarpedWidth", config.rendered_resolution.into());
    set_yaml_value(&mut doc, "runtimeFoveationWarpedWidth", config.encoded_resolution.into());
    set_yaml_value(&mut doc, "runtimeFoveationInset", inset.into());
    set_yaml_value(&mut doc, "disableAlpha", (!config.alpha_transparency_enabled).into());

    let output = serde_yaml::to_string(&doc).map_err(|e| format!("serialize: {}", e))?;
    std::fs::write(path, output).map_err(|e| format!("write {}: {}", path.display(), e))
}

/// Recursively finds and updates a key in a YAML document.
fn set_yaml_value(doc: &mut serde_yaml::Value, key: &str, val: serde_yaml::Value) {
    if let Some(mapping) = doc.as_mapping_mut() {
        let yaml_key = serde_yaml::Value::String(key.to_string());
        if mapping.contains_key(&yaml_key) {
            mapping[&yaml_key] = val;
            return;
        }
        for (_, v) in mapping.iter_mut() {
            set_yaml_value(v, key, val.clone());
        }
    }
}

fn update_settings_json(launch_default_app: bool) -> Result<PathBuf, String> {
    let path = settings_json_path().ok_or_else(|| "could not determine settings path".to_string())?;

    let mut settings: serde_json::Value = if path.exists() {
        let contents = std::fs::read_to_string(&path).map_err(|e| format!("read {}: {}", path.display(), e))?;
        serde_json::from_str(&contents).map_err(|e| format!("parse {}: {}", path.display(), e))?
    } else {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).map_err(|e| format!("create {}: {}", parent.display(), e))?;
        }
        serde_json::json!({
            "clearxrExePath": "clear-xr.exe",
            "clearxrLaunchDelaySeconds": 3
        })
    };

    settings["launchDefaultApp"] = serde_json::json!(launch_default_app);

    let output = serde_json::to_string_pretty(&settings).map_err(|e| format!("serialize: {}", e))?;
    std::fs::write(&path, output + "\n").map_err(|e| format!("write {}: {}", path.display(), e))?;
    Ok(path)
}

fn settings_json_path() -> Option<PathBuf> {
    if let Some(local) = std::env::var_os("LOCALAPPDATA") {
        return Some(PathBuf::from(local).join("ClearXR").join("clearxr-settings.json"));
    }
    if let Some(appdata) = std::env::var_os("APPDATA") {
        return Some(PathBuf::from(appdata).join("ClearXR").join("clearxr-settings.json"));
    }
    None
}

#[cfg(windows)]
fn get_dll_dir() -> Option<PathBuf> {
    use std::ffi::OsString;
    use std::os::windows::ffi::OsStringExt;

    unsafe extern "system" {
        fn GetModuleHandleExW(flags: u32, module_name: *const u16, module: *mut *mut std::ffi::c_void) -> i32;
        fn GetModuleFileNameW(module: *mut std::ffi::c_void, filename: *mut u16, size: u32) -> u32;
    }

    const FLAG_FROM_ADDRESS: u32 = 0x00000004;
    const FLAG_UNCHANGED_REFCOUNT: u32 = 0x00000002;

    unsafe {
        let mut module: *mut std::ffi::c_void = std::ptr::null_mut();
        // An address inside this DLL identifies the module.
        let addr = get_dll_dir as *const u16;
        if GetModuleHandleExW(FLAG_FROM_ADDRESS | FLAG_UNCHANGED_REFCOUNT, addr, &mut module) == 0 {
            return None;
        }

        let mut buf = [0u16; 512];
        let len = GetModuleFileNameW(module, buf.as_mut_ptr(), buf.len() as u32);
        if len == 0 || len >= buf.len() as u32 {
            return None;
        }

        let path = PathBuf::from(OsString::from_wide(&buf[..len as usize]));
        path.parent().map(|p| p.to_path_buf())
    }
}

#[cfg(not(windows))]
fn get_dll_dir() -> Option<PathBuf> {
    None
}

#[cfg(test)]
mod tests {
    #[test]
    fn rejects_truncated_packets_without_panicking() {
        let mut packet = vec![0x46, 0x43, 100, 0, 0, 0];
        packet.extend_from_slice(b"{}");
        super::handle(&packet);
        super::handle(&packet[..3]);
    }
}
