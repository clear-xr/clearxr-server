//! CloudXR's opaque data channel (`XR_NV_opaque_data_channel`, or its experimental
//! `XR_NVX1_` form), which carries Clear XR's packets between the headset and this layer.

use openxr_sys as xr;
use std::ffi::{c_char, c_void};
use std::time::{Duration, Instant};

/// Clear XR's channel UUID as raw bytes, as `PROTOCOL.md` lists them. They are the in-memory form
/// of the Windows `GUID` {12345678-1234-1234-1234-56789ABCDEF0}, so visionOS reports the channel
/// as 78563412-3412-3412-1234-56789ABCDEF0.
pub const CHANNEL_UUID: [u8; 16] = [
    0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
];

pub const EXTENSIONS: [&str; 2] = ["XR_NV_opaque_data_channel", "XR_NVX1_opaque_data_channel"];

const TYPE_CREATE_INFO: i32 = 1000500000;
const TYPE_STATE: i32 = 1000500001;
const STATUS_CONNECTED: i32 = 1;
const STATUS_DISCONNECTED: i32 = 3;
const REOPEN_INTERVAL: Duration = Duration::from_secs(2);

#[repr(C)]
struct CreateInfo {
    ty: xr::StructureType,
    next: *const c_void,
    system_id: xr::SystemId,
    uuid: [u8; 16],
}

#[repr(C)]
struct State {
    ty: xr::StructureType,
    next: *mut c_void,
    status: i32,
}

type Handle = u64;
type FnCreate = unsafe extern "system" fn(xr::Instance, *const CreateInfo, *mut Handle) -> xr::Result;
type FnHandle = unsafe extern "system" fn(Handle) -> xr::Result;
type FnGetState = unsafe extern "system" fn(Handle, *mut State) -> xr::Result;
type FnSend = unsafe extern "system" fn(Handle, u32, *const u8) -> xr::Result;
type FnReceive = unsafe extern "system" fn(Handle, u32, *mut u32, *mut u8) -> xr::Result;

#[derive(Clone, Copy)]
pub struct Functions {
    create: FnCreate,
    destroy: FnHandle,
    get_state: FnGetState,
    shutdown: FnHandle,
    send: FnSend,
    receive: FnReceive,
    /// The suffix the entry points were found under, for the log.
    pub suffix: &'static str,
}

impl Functions {
    /// Resolves the channel functions from the next layer, trying the `NV` names and then the
    /// `NVX1` names.
    pub unsafe fn load(gpa: xr::pfn::GetInstanceProcAddr, instance: xr::Instance) -> Option<Functions> {
        let load = |name: String| -> Option<xr::pfn::VoidFunction> {
            let name = format!("{name}\0");
            let mut function = None;
            gpa(instance, name.as_ptr() as *const c_char, &mut function);
            function
        };
        ["NV", "NVX1"].into_iter().find_map(|suffix| {
            let f = |stem: &str| load(format!("xr{stem}OpaqueDataChannel{suffix}"));
            Some(Functions {
                create: std::mem::transmute(f("Create")?),
                destroy: std::mem::transmute(f("Destroy")?),
                get_state: std::mem::transmute(load(format!("xrGetOpaqueDataChannelState{suffix}"))?),
                shutdown: std::mem::transmute(f("Shutdown")?),
                send: std::mem::transmute(f("Send")?),
                receive: std::mem::transmute(f("Receive")?),
                suffix,
            })
        })
    }
}

pub enum Poll {
    Unchanged,
    /// The channel became connected; packets from before this belong to another connection.
    Connected,
    Disconnected,
}

pub struct Channel {
    functions: Functions,
    instance: xr::Instance,
    system_id: xr::SystemId,
    handle: Handle,
    connected: bool,
    last_status: i32,
    next_open: Instant,
    buffer: Vec<u8>,
}

impl Channel {
    pub unsafe fn open(functions: Functions, instance: xr::Instance, system_id: xr::SystemId) -> Channel {
        let mut channel = Channel {
            functions,
            instance,
            system_id,
            handle: 0,
            connected: false,
            last_status: -1,
            next_open: Instant::now(),
            buffer: vec![0; 4096],
        };
        channel.create();
        channel
    }

    unsafe fn create(&mut self) {
        self.next_open = Instant::now() + REOPEN_INTERVAL;
        let info = CreateInfo {
            ty: xr::StructureType::from_raw(TYPE_CREATE_INFO),
            next: std::ptr::null(),
            system_id: self.system_id,
            uuid: CHANNEL_UUID,
        };
        let mut handle = 0;
        let result = (self.functions.create)(self.instance, &info, &mut handle);
        if result == xr::Result::SUCCESS {
            self.handle = handle;
            crate::log_info(&format!("Opened the bridge channel ({}) for system {}, UUID bytes {:02X?}",
                self.functions.suffix, self.system_id.into_raw(), CHANNEL_UUID));
        } else {
            crate::log_warn(&format!("Opening the bridge channel failed: {result:?}"));
        }
    }

    unsafe fn close(&mut self) {
        if self.handle != 0 {
            (self.functions.shutdown)(self.handle);
            (self.functions.destroy)(self.handle);
            self.handle = 0;
        }
        self.connected = false;
    }

    unsafe fn status(&self) -> i32 {
        let mut state = State { ty: xr::StructureType::from_raw(TYPE_STATE), next: std::ptr::null_mut(), status: -1 };
        (self.functions.get_state)(self.handle, &mut state);
        state.status
    }

    /// Tracks the connection, reopening a disconnected channel, and hands every received message
    /// to `receive`.
    pub unsafe fn poll(&mut self, mut receive: impl FnMut(&[u8])) -> Poll {
        if self.handle == 0 {
            if Instant::now() >= self.next_open {
                self.create();
            }
            return Poll::Unchanged;
        }
        let mut change = Poll::Unchanged;
        let status = self.status();
        if status != self.last_status {
            crate::log_info(&format!("Bridge channel status {} -> {status} (0 connecting, 1 connected, 2 shutting down, 3 disconnected)", self.last_status));
            self.last_status = status;
        }
        match status {
            STATUS_CONNECTED if !self.connected => {
                self.connected = true;
                crate::log_info("Bridge channel connected");
                change = Poll::Connected;
            }
            STATUS_DISCONNECTED => {
                if self.connected {
                    crate::log_info("Bridge channel disconnected; reopening");
                    change = Poll::Disconnected;
                }
                if Instant::now() >= self.next_open {
                    self.close();
                    self.create();
                }
                self.connected = false;
                return change;
            }
            _ => {}
        }
        if !self.connected {
            return change;
        }
        loop {
            let mut received = 0u32;
            let result = (self.functions.receive)(self.handle, self.buffer.len() as u32, &mut received, self.buffer.as_mut_ptr());
            if result != xr::Result::SUCCESS || received == 0 {
                break;
            }
            receive(&self.buffer[..(received as usize).min(self.buffer.len())]);
        }
        change
    }

    pub unsafe fn send(&mut self, bytes: &[u8]) -> bool {
        if !self.connected {
            return false;
        }
        (self.functions.send)(self.handle, bytes.len() as u32, bytes.as_ptr()) == xr::Result::SUCCESS
    }
}

impl Drop for Channel {
    fn drop(&mut self) {
        unsafe { self.close() }
    }
}
