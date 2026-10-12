//! Which packet field drives each input path of the interaction profiles the layer reports.

use crate::protocol::{buttons, HandBlock};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Hand {
    Left = 0,
    Right = 1,
}

impl Hand {
    pub const BOTH: [Hand; 2] = [Hand::Left, Hand::Right];

    pub fn path(self) -> &'static str {
        match self {
            Hand::Left => "/user/hand/left",
            Hand::Right => "/user/hand/right",
        }
    }

    pub fn from_path(path: &str) -> Option<Hand> {
        Hand::BOTH.into_iter().find(|hand| path == hand.path())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Profile {
    OculusTouch,
    SimpleController,
}

impl Profile {
    /// In order of preference.
    pub const ALL: [Profile; 2] = [Profile::OculusTouch, Profile::SimpleController];

    pub fn path(self) -> &'static str {
        match self {
            Profile::OculusTouch => "/interaction_profiles/oculus/touch_controller",
            Profile::SimpleController => "/interaction_profiles/khr/simple_controller",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Source {
    Bit(u16),
    /// Analog trigger; pressed is the `TRIGGER` bit.
    Trigger,
    /// Analog squeeze; pressed is the `GRIP` bit.
    Grip,
    Thumbstick,
    ThumbstickX,
    ThumbstickY,
    /// An input PSVR2 Sense lacks, such as the thumb rest; always reads as released.
    Absent,
    GripPose,
    AimPose,
    Haptic,
}

/// Splits `/user/hand/left/input/x/click` into the hand and `/input/x/click`.
pub fn split_binding(path: &str) -> Option<(Hand, &str)> {
    Hand::BOTH.into_iter().find_map(|hand| {
        let rest = path.strip_prefix(hand.path())?;
        rest.starts_with('/').then_some((hand, rest))
    })
}

/// The packet field behind `component` (such as `/input/x/click`) on `hand` in `profile`.
/// A path without a component (`/input/trigger`) names the input's default component.
pub fn source(profile: Profile, hand: Hand, component: &str) -> Option<Source> {
    use Source::*;
    let shared = match component {
        "/input/grip/pose" | "/input/grip" => Some(GripPose),
        "/input/aim/pose" | "/input/aim" => Some(AimPose),
        "/output/haptic" => Some(Haptic),
        _ => None,
    };
    if shared.is_some() {
        return shared;
    }
    match profile {
        Profile::SimpleController => match component {
            "/input/select/click" | "/input/select" => Some(Bit(buttons::TRIGGER)),
            "/input/menu/click" | "/input/menu" => Some(Bit(buttons::MENU)),
            _ => None,
        },
        Profile::OculusTouch => {
            // The lower face button is X on the left controller and A on the right.
            let (lower, upper) = match hand {
                Hand::Left => ("x", "y"),
                Hand::Right => ("a", "b"),
            };
            let (input, part) = match component.strip_prefix("/input/")?.split_once('/') {
                Some((input, part)) => (input, Some(part)),
                None => (component.strip_prefix("/input/")?, None),
            };
            match (input, part) {
                (i, Some("click") | None) if i == lower => Some(Bit(buttons::BUTTON_A)),
                (i, Some("touch")) if i == lower => Some(Bit(buttons::TOUCH_BUTTON_A)),
                (i, Some("click") | None) if i == upper => Some(Bit(buttons::BUTTON_B)),
                (i, Some("touch")) if i == upper => Some(Bit(buttons::TOUCH_BUTTON_B)),
                ("menu", Some("click") | None) if hand == Hand::Left => Some(Bit(buttons::MENU)),
                ("trigger", Some("value") | None) => Some(Trigger),
                ("trigger", Some("touch")) => Some(Bit(buttons::TOUCH_TRIGGER)),
                ("squeeze", Some("value") | None) => Some(Grip),
                ("thumbstick", None) => Some(Thumbstick),
                ("thumbstick", Some("x")) => Some(ThumbstickX),
                ("thumbstick", Some("y")) => Some(ThumbstickY),
                ("thumbstick", Some("click")) => Some(Bit(buttons::THUMBSTICK_CLICK)),
                ("thumbstick", Some("touch")) => Some(Bit(buttons::TOUCH_THUMBSTICK)),
                ("thumbrest", Some("touch") | None) => Some(Absent),
                _ => None,
            }
        }
    }
}

pub fn bool_value(source: Source, hand: &HandBlock) -> Option<bool> {
    match source {
        Source::Bit(bit) => Some(hand.buttons & bit != 0),
        Source::Trigger => Some(hand.buttons & buttons::TRIGGER != 0),
        Source::Grip => Some(hand.buttons & buttons::GRIP != 0),
        Source::Absent => Some(false),
        _ => None,
    }
}

pub fn float_value(source: Source, hand: &HandBlock) -> Option<f32> {
    match source {
        Source::Trigger => Some(hand.trigger),
        Source::Grip => Some(hand.grip),
        Source::ThumbstickX => Some(hand.thumbstick[0]),
        Source::ThumbstickY => Some(hand.thumbstick[1]),
        _ => bool_value(source, hand).map(|pressed| if pressed { 1.0 } else { 0.0 }),
    }
}

pub fn vector_value(source: Source, hand: &HandBlock) -> Option<[f32; 2]> {
    match source {
        Source::Thumbstick => Some(hand.thumbstick),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn touch_face_buttons_follow_the_hand() {
        let touch = Profile::OculusTouch;
        assert_eq!(source(touch, Hand::Left, "/input/x/click"), Some(Source::Bit(buttons::BUTTON_A)));
        assert_eq!(source(touch, Hand::Right, "/input/b/touch"), Some(Source::Bit(buttons::TOUCH_BUTTON_B)));
        assert_eq!(source(touch, Hand::Right, "/input/x/click"), None);
        assert_eq!(source(touch, Hand::Left, "/input/y"), Some(Source::Bit(buttons::BUTTON_B)));
        assert_eq!(source(touch, Hand::Right, "/input/menu/click"), None);
    }

    #[test]
    fn analog_and_pose_inputs() {
        let touch = Profile::OculusTouch;
        assert_eq!(source(touch, Hand::Right, "/input/trigger/value"), Some(Source::Trigger));
        assert_eq!(source(touch, Hand::Right, "/input/squeeze"), Some(Source::Grip));
        assert_eq!(source(touch, Hand::Left, "/input/thumbstick"), Some(Source::Thumbstick));
        assert_eq!(source(touch, Hand::Left, "/input/thumbstick/y"), Some(Source::ThumbstickY));
        assert_eq!(source(touch, Hand::Left, "/input/thumbrest/touch"), Some(Source::Absent));
        assert_eq!(source(touch, Hand::Left, "/input/aim/pose"), Some(Source::AimPose));
        assert_eq!(source(Profile::SimpleController, Hand::Right, "/input/select/click"), Some(Source::Bit(buttons::TRIGGER)));
        assert_eq!(source(Profile::SimpleController, Hand::Right, "/input/trigger/value"), None);
    }

    #[test]
    fn values_by_action_type() {
        let hand = HandBlock {
            buttons: buttons::BUTTON_A | buttons::TRIGGER,
            trigger: 0.75,
            thumbstick: [0.5, -0.25],
            ..Default::default()
        };
        assert_eq!(bool_value(Source::Trigger, &hand), Some(true));
        assert_eq!(float_value(Source::Trigger, &hand), Some(0.75));
        assert_eq!(float_value(Source::Bit(buttons::BUTTON_A), &hand), Some(1.0));
        assert_eq!(float_value(Source::Bit(buttons::BUTTON_B), &hand), Some(0.0));
        assert_eq!(vector_value(Source::Thumbstick, &hand), Some([0.5, -0.25]));
        assert_eq!(bool_value(Source::Thumbstick, &hand), None);
    }

    #[test]
    fn splits_binding_paths() {
        assert_eq!(split_binding("/user/hand/left/input/x/click"), Some((Hand::Left, "/input/x/click")));
        assert_eq!(split_binding("/user/hand/right/output/haptic"), Some((Hand::Right, "/output/haptic")));
        assert_eq!(split_binding("/user/hand/leftover/input"), None);
        assert_eq!(split_binding("/user/gamepad/input/a/click"), None);
    }
}
