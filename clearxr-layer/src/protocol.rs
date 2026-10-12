//! Packet formats from `PROTOCOL.md`: the controller packet the layer
//! decodes and the haptic packet it encodes. Version 2 only.

pub const CONTROLLER_MAGIC: u16 = 0x5343;
pub const HAPTIC_MAGIC: u16 = 0x4856;
pub const VERSION: u8 = 2;
pub const CONTROLLER_SIZE: usize = 248;
pub const HAPTIC_SIZE: usize = 24;

pub const FLAG_HEAD_RELATIVE: u16 = 1 << 0;
const FLAGS_DEFINED: u16 = 0x0001;
const ACTIVE_HANDS_DEFINED: u8 = 0x03;

#[allow(dead_code)]
pub mod buttons {
    pub const BUTTON_A: u16 = 1 << 0;
    pub const BUTTON_B: u16 = 1 << 1;
    pub const TRIGGER: u16 = 1 << 2;
    pub const GRIP: u16 = 1 << 3;
    pub const THUMBSTICK_CLICK: u16 = 1 << 4;
    pub const MENU: u16 = 1 << 5;
    pub const TOUCH_BUTTON_A: u16 = 1 << 6;
    pub const TOUCH_BUTTON_B: u16 = 1 << 7;
    pub const TOUCH_TRIGGER: u16 = 1 << 8;
    pub const TOUCH_GRIP: u16 = 1 << 9;
    pub const TOUCH_THUMBSTICK: u16 = 1 << 10;
    pub(super) const DEFINED: u16 = 0x07FF;
}

/// Bits 0–3 equal `XrSpaceLocationFlags`.
#[allow(dead_code)]
pub mod tracking {
    pub const ORIENTATION_VALID: u16 = 1 << 0;
    pub const POSITION_VALID: u16 = 1 << 1;
    pub const ORIENTATION_TRACKED: u16 = 1 << 2;
    pub const POSITION_TRACKED: u16 = 1 << 3;
    pub const LINEAR_VELOCITY_VALID: u16 = 1 << 4;
    pub const ANGULAR_VELOCITY_VALID: u16 = 1 << 5;
    pub const LOW_ACCURACY: u16 = 1 << 6;
    pub(super) const DEFINED: u16 = 0x007F;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub position: [f32; 3],
    /// x, y, z, w
    pub orientation: [f32; 4],
}

impl Default for Pose {
    fn default() -> Self {
        Pose { position: [0.0; 3], orientation: [0.0, 0.0, 0.0, 1.0] }
    }
}

/// One hand's block. Poses and velocities are in the head frame.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HandBlock {
    pub sample_time_ns: u64,
    pub buttons: u16,
    pub tracking_flags: u16,
    pub thumbstick: [f32; 2],
    pub trigger: f32,
    pub grip: f32,
    pub grip_pose: Pose,
    pub aim_pose: Pose,
    pub linear_velocity: [f32; 3],
    pub angular_velocity: [f32; 3],
}

impl HandBlock {
    /// Tracking flags with the location bits cleared when any pose or velocity field is not
    /// finite, which the receiving rules treat as no valid pose.
    pub fn usable_tracking_flags(&self) -> u16 {
        let fields = self.grip_pose.position.iter()
            .chain(&self.grip_pose.orientation)
            .chain(&self.aim_pose.position)
            .chain(&self.aim_pose.orientation)
            .chain(&self.linear_velocity)
            .chain(&self.angular_velocity);
        let mut finite = true;
        for value in fields {
            finite &= value.is_finite();
        }
        if finite { self.tracking_flags } else { 0 }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControllerPacket {
    pub active_hands: u8,
    pub flags: u16,
    pub sequence: u32,
    pub send_time_ns: u64,
    /// Left, then right.
    pub hands: [HandBlock; 2],
}

impl ControllerPacket {
    pub fn is_active(&self, hand: usize) -> bool {
        self.active_hands & (1 << hand) != 0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reject {
    TooShort,
    NotController,
    UnknownVersion,
}

pub fn decode_controller(bytes: &[u8]) -> Result<ControllerPacket, Reject> {
    if bytes.len() < 4 {
        return Err(Reject::TooShort);
    }
    let r = Reader(bytes);
    if r.u16(0) != CONTROLLER_MAGIC {
        return Err(Reject::NotController);
    }
    if r.u8(2) != VERSION {
        return Err(Reject::UnknownVersion);
    }
    if bytes.len() < CONTROLLER_SIZE {
        return Err(Reject::TooShort);
    }
    let hand = |base: usize| HandBlock {
        sample_time_ns: r.u64(base),
        buttons: r.u16(base + 8) & buttons::DEFINED,
        tracking_flags: r.u16(base + 10) & tracking::DEFINED,
        thumbstick: [r.f32(base + 16), r.f32(base + 20)],
        trigger: r.f32(base + 24),
        grip: r.f32(base + 28),
        grip_pose: r.pose(base + 32),
        aim_pose: r.pose(base + 60),
        linear_velocity: r.vec3(base + 88),
        angular_velocity: r.vec3(base + 100),
    };
    Ok(ControllerPacket {
        active_hands: r.u8(3) & ACTIVE_HANDS_DEFINED,
        flags: r.u16(4) & FLAGS_DEFINED,
        sequence: r.u32(8),
        send_time_ns: r.u64(16),
        hands: [hand(24), hand(136)],
    })
}

/// Whether `sequence` is newer than `last`, with wrapping arithmetic.
pub fn is_newer(sequence: u32, last: u32) -> bool {
    (sequence.wrapping_sub(last) as i32) > 0
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum HapticCommand {
    Apply { duration_ns: u64, frequency: f32, amplitude: f32 },
    Stop,
}

/// `hand` is 0 for left, 1 for right.
pub fn encode_haptic(hand: u8, command: HapticCommand) -> [u8; HAPTIC_SIZE] {
    let (code, duration_ns, frequency, amplitude) = match command {
        HapticCommand::Apply { duration_ns, frequency, amplitude } => (0u8, duration_ns, frequency, amplitude),
        HapticCommand::Stop => (1u8, 0, 0.0, 0.0),
    };
    let mut bytes = [0u8; HAPTIC_SIZE];
    bytes[0..2].copy_from_slice(&HAPTIC_MAGIC.to_le_bytes());
    bytes[2] = VERSION;
    bytes[3] = hand;
    bytes[4] = code;
    bytes[8..16].copy_from_slice(&duration_ns.to_le_bytes());
    bytes[16..20].copy_from_slice(&frequency.to_le_bytes());
    bytes[20..24].copy_from_slice(&amplitude.to_le_bytes());
    bytes
}

struct Reader<'a>(&'a [u8]);

impl Reader<'_> {
    fn array<const N: usize>(&self, offset: usize) -> [u8; N] {
        self.0[offset..offset + N].try_into().unwrap()
    }
    fn u8(&self, offset: usize) -> u8 {
        self.0[offset]
    }
    fn u16(&self, offset: usize) -> u16 {
        u16::from_le_bytes(self.array(offset))
    }
    fn u32(&self, offset: usize) -> u32 {
        u32::from_le_bytes(self.array(offset))
    }
    fn u64(&self, offset: usize) -> u64 {
        u64::from_le_bytes(self.array(offset))
    }
    fn f32(&self, offset: usize) -> f32 {
        f32::from_le_bytes(self.array(offset))
    }
    fn vec3(&self, offset: usize) -> [f32; 3] {
        [self.f32(offset), self.f32(offset + 4), self.f32(offset + 8)]
    }
    fn pose(&self, offset: usize) -> Pose {
        Pose {
            position: self.vec3(offset),
            orientation: [self.f32(offset + 12), self.f32(offset + 16), self.f32(offset + 20), self.f32(offset + 24)],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A packet with both hands tracked, every button bit used, and exactly representable floats.
    const BOTH_HANDS: &str = "43530203010000000700000000000000cb04fb711f0100008009ea701f01000051033f0000000000000080be0000003f\
        000000000000803f000080be0000c0be000000bf0000003f000000bf0000003f0000003f000060be0000b0be000010bf\
        0000003f000000bf0000003f0000003f0000003e000080bd00000000000000000000c03f000040bf00a082711f010000\
        ff073f00000000000000803f000080bf0000403f0000003f0000803e0000c0be000000bf000000000000000000000000\
        0000803f0000803e0000b0be000010bf000000bf0000003f000000bf0000003f000000bf0000803e00001040000040c0\
        000000000000003e";

    fn bytes(hex: &str) -> Vec<u8> {
        (0..hex.len()).step_by(2).map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap()).collect()
    }

    fn both_hands() -> ControllerPacket {
        ControllerPacket {
            active_hands: 3,
            flags: FLAG_HEAD_RELATIVE,
            sequence: 7,
            send_time_ns: 1234567890123,
            hands: [
                HandBlock {
                sample_time_ns: 1234550000000,
                buttons: 849,
                tracking_flags: 63,
                thumbstick: [-0.25, 0.5],
                trigger: 0.0,
                grip: 1.0,
                grip_pose: Pose { position: [-0.25, -0.375, -0.5], orientation: [0.5, -0.5, 0.5, 0.5] },
                aim_pose: Pose { position: [-0.21875, -0.34375, -0.5625], orientation: [0.5, -0.5, 0.5, 0.5] },
                linear_velocity: [0.125, -0.0625, 0.0],
                angular_velocity: [0.0, 1.5, -0.75],
            },
                HandBlock {
                sample_time_ns: 1234560000000,
                buttons: 2047,
                tracking_flags: 63,
                thumbstick: [1.0, -1.0],
                trigger: 0.75,
                grip: 0.5,
                grip_pose: Pose { position: [0.25, -0.375, -0.5], orientation: [0.0, 0.0, 0.0, 1.0] },
                aim_pose: Pose { position: [0.25, -0.34375, -0.5625], orientation: [-0.5, 0.5, -0.5, 0.5] },
                linear_velocity: [-0.5, 0.25, 2.25],
                angular_velocity: [-3.0, 0.0, 0.125],
            },
            ],
        }
    }

    #[test]
    fn decodes_both_hands() {
        assert_eq!(decode_controller(&bytes(BOTH_HANDS)), Ok(both_hands()));
    }

    #[test]
    fn ignores_reserved_bits_and_trailing_bytes() {
        let mut packet = bytes(BOTH_HANDS);
        packet[3] |= 0xFC; // activeHands bits 2–7
        packet[4] |= 0xFE; // flags bits 1–7
        packet[5] = 0xFF; // flags bits 8–15
        packet[6..8].fill(0xFF);
        packet[12..16].fill(0xFF);
        for base in [24, 136] {
            packet[base + 9] |= 0xF8; // buttons bits 11–15
            packet[base + 10] |= 0x80; // trackingFlags bits 7–15
            packet[base + 11] = 0xFF;
            packet[base + 12..base + 16].fill(0xFF);
        }
        packet.extend_from_slice(&[0xAB; 8]);
        assert_eq!(decode_controller(&packet), Ok(both_hands()));
    }

    #[test]
    fn rejects_malformed_packets() {
        let packet = bytes(BOTH_HANDS);
        assert_eq!(decode_controller(&[]), Err(Reject::TooShort));
        assert_eq!(decode_controller(&packet[..3]), Err(Reject::TooShort));
        assert_eq!(decode_controller(&packet[..CONTROLLER_SIZE - 1]), Err(Reject::TooShort));
        for version in [0, 1, 3] {
            let mut other = packet.clone();
            other[2] = version;
            assert_eq!(decode_controller(&other), Err(Reject::UnknownVersion));
        }
        let mut haptic = packet.clone();
        haptic[0..2].copy_from_slice(&HAPTIC_MAGIC.to_le_bytes());
        assert_eq!(decode_controller(&haptic), Err(Reject::NotController));
    }

    #[test]
    fn encodes_haptics() {
        let apply = HapticCommand::Apply { duration_ns: 100000000, frequency: 160.0, amplitude: 0.75 };
        assert_eq!(encode_haptic(0, apply).to_vec(), bytes("564802000000000000e1f50500000000000020430000403f"));
        assert_eq!(encode_haptic(1, HapticCommand::Stop).to_vec(), bytes("564802010100000000000000000000000000000000000000"));
    }

    #[test]
    fn sequence_wraps() {
        assert!(is_newer(0, u32::MAX));
        assert!(is_newer(5, 4));
        assert!(!is_newer(4, 4));
        assert!(!is_newer(u32::MAX, 0));
    }

    #[test]
    fn non_finite_pose_has_no_location() {
        let mut hand = HandBlock { tracking_flags: 0x3F, ..Default::default() };
        assert_eq!(hand.usable_tracking_flags(), 0x3F);
        hand.angular_velocity[1] = f32::NAN;
        assert_eq!(hand.usable_tracking_flags(), 0);
    }
}
