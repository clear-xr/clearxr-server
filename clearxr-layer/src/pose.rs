//! Pose math for composing a head-relative controller pose with the runtime's `VIEW` space,
//! as "On the layer" in `PROTOCOL.md` describes.

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Vec3(pub [f32; 3]);

/// x, y, z, w
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat(pub [f32; 4]);

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub position: Vec3,
    pub orientation: Quat,
}

impl Vec3 {
    pub fn add(self, o: Vec3) -> Vec3 {
        Vec3([self.0[0] + o.0[0], self.0[1] + o.0[1], self.0[2] + o.0[2]])
    }
    pub fn sub(self, o: Vec3) -> Vec3 {
        Vec3([self.0[0] - o.0[0], self.0[1] - o.0[1], self.0[2] - o.0[2]])
    }
    pub fn scale(self, s: f32) -> Vec3 {
        Vec3([self.0[0] * s, self.0[1] * s, self.0[2] * s])
    }
    pub fn cross(self, o: Vec3) -> Vec3 {
        let (a, b) = (self.0, o.0);
        Vec3([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])
    }
    pub fn length(self) -> f32 {
        (self.0[0] * self.0[0] + self.0[1] * self.0[1] + self.0[2] * self.0[2]).sqrt()
    }
}

impl Quat {
    pub const IDENTITY: Quat = Quat([0.0, 0.0, 0.0, 1.0]);

    pub fn mul(self, o: Quat) -> Quat {
        let [ax, ay, az, aw] = self.0;
        let [bx, by, bz, bw] = o.0;
        Quat([
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ])
    }
    pub fn conjugate(self) -> Quat {
        Quat([-self.0[0], -self.0[1], -self.0[2], self.0[3]])
    }
    pub fn rotate(self, v: Vec3) -> Vec3 {
        let u = Vec3([self.0[0], self.0[1], self.0[2]]);
        let w = self.0[3];
        let t = u.cross(v).scale(2.0);
        v.add(t.scale(w)).add(u.cross(t))
    }
    pub fn normalized(self) -> Quat {
        let n = self.0.iter().map(|c| c * c).sum::<f32>().sqrt();
        if n > 0.0 { Quat(self.0.map(|c| c / n)) } else { Quat::IDENTITY }
    }
    /// The rotation by `rotation_vector` (axis × angle, radians).
    pub fn from_rotation_vector(rotation_vector: Vec3) -> Quat {
        let angle = rotation_vector.length();
        if angle < 1e-9 {
            return Quat::IDENTITY;
        }
        let axis = rotation_vector.scale(1.0 / angle);
        let (s, c) = (angle / 2.0).sin_cos();
        Quat([axis.0[0] * s, axis.0[1] * s, axis.0[2] * s, c])
    }
}

impl Pose {
    #[cfg(test)]
    pub const IDENTITY: Pose = Pose { position: Vec3([0.0; 3]), orientation: Quat::IDENTITY };

    /// `self ∘ child`: `child` expressed in the frame `self` describes.
    pub fn compose(self, child: Pose) -> Pose {
        Pose {
            position: self.position.add(self.orientation.rotate(child.position)),
            orientation: self.orientation.mul(child.orientation).normalized(),
        }
    }
    pub fn inverse(self) -> Pose {
        let q = self.orientation.conjugate();
        Pose { position: q.rotate(self.position).scale(-1.0), orientation: q }
    }
}

/// The runtime's `VIEW` space located in the base space at the requested time. Velocities are
/// zero when the runtime did not report them.
#[derive(Clone, Copy, Debug)]
pub struct View {
    pub pose: Pose,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
}

/// A point on a controller, from one packet.
#[derive(Clone, Copy, Debug)]
pub struct Controller {
    /// The located pose in the head frame (grip or aim, with any action-space offset applied).
    pub pose: Pose,
    /// Where the packet's velocities apply, in the head frame.
    pub grip_position: Vec3,
    /// The grip's velocity relative to the world, in head axes.
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
}

pub struct Located {
    pub pose: Pose,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
}

/// Longest extrapolation, in either direction.
pub const MAX_EXTRAPOLATION_S: f32 = 0.1;

/// Locates `controller` in the base space `view` is located in, extrapolated by `dt` seconds.
/// `head_offset` is the device anchor's position in `VIEW` space.
pub fn locate(view: &View, controller: &Controller, head_offset: Vec3, dt: f32) -> Located {
    let dt = dt.clamp(-MAX_EXTRAPOLATION_S, MAX_EXTRAPOLATION_S);
    let r_v = view.pose.orientation;
    let r_v_inv = r_v.conjugate();

    // World-relative velocities of the grip, in base axes.
    let v = r_v.rotate(controller.linear_velocity);
    let w = r_v.rotate(controller.angular_velocity);

    // Velocities relative to the head, in head axes, at the grip.
    let grip_from_view = head_offset.add(controller.grip_position);
    let w_rel = r_v_inv.rotate(w.sub(view.angular_velocity));
    let v_rel = r_v_inv.rotate(v.sub(view.linear_velocity).sub(view.angular_velocity.cross(r_v.rotate(grip_from_view))));

    // Extrapolate the located point, whose velocity differs from the grip's by ω × lever arm.
    let lever = controller.pose.position.sub(controller.grip_position);
    let point_v_rel = v_rel.add(w_rel.cross(lever));
    let in_view = Pose {
        position: head_offset.add(controller.pose.position).add(point_v_rel.scale(dt)),
        orientation: Quat::from_rotation_vector(w_rel.scale(dt)).mul(controller.pose.orientation).normalized(),
    };

    Located {
        pose: view.pose.compose(in_view),
        linear_velocity: v.add(w.cross(r_v.rotate(lever))),
        angular_velocity: w,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: Vec3, b: Vec3) -> bool {
        a.sub(b).length() < 1e-4
    }

    fn same_rotation(a: Quat, b: Quat) -> bool {
        let dot: f32 = a.0.iter().zip(b.0).map(|(x, y)| x * y).sum();
        dot.abs() > 1.0 - 1e-5
    }

    fn yaw(radians: f32) -> Quat {
        Quat::from_rotation_vector(Vec3([0.0, radians, 0.0]))
    }

    #[test]
    fn compose_and_inverse() {
        let a = Pose { position: Vec3([1.0, 2.0, 3.0]), orientation: yaw(0.7) };
        let b = Pose { position: Vec3([-0.5, 0.25, 2.0]), orientation: Quat::from_rotation_vector(Vec3([0.3, 0.0, 0.4])) };
        let identity = a.compose(b).compose(b.inverse()).compose(a.inverse());
        assert!(close(identity.position, Vec3::default()));
        assert!(same_rotation(identity.orientation, Quat::IDENTITY));
    }

    #[test]
    fn static_controller_composes_with_view() {
        let view = View {
            pose: Pose { position: Vec3([0.0, 1.6, 0.0]), orientation: yaw(std::f32::consts::FRAC_PI_2) },
            linear_velocity: Vec3::default(),
            angular_velocity: Vec3::default(),
        };
        let pose = Pose { position: Vec3([0.2, -0.3, -0.4]), orientation: Quat::IDENTITY };
        let controller = Controller { pose, grip_position: pose.position, linear_velocity: Vec3::default(), angular_velocity: Vec3::default() };
        let located = locate(&view, &controller, Vec3::default(), 0.05);
        // Turned 90° left, the head's forward (−Z) is the base's −X and its right (+X) is −Z.
        assert!(close(located.pose.position, Vec3([-0.4, 1.3, -0.2])));
        assert!(same_rotation(located.pose.orientation, view.pose.orientation));
    }

    #[test]
    fn hand_still_in_world_while_head_turns() {
        // The head yaws at 1 rad/s; the hand is still in the world, so in the head frame it swings
        // the other way, and extrapolating with the head's motion keeps it fixed in the base.
        let w_head = Vec3([0.0, 1.0, 0.0]);
        let grip = Vec3([0.0, 0.0, -0.5]);
        let view_at = |t: f32| View {
            pose: Pose { position: Vec3([0.0, 1.6, 0.0]), orientation: yaw(t) },
            linear_velocity: Vec3::default(),
            angular_velocity: w_head,
        };
        let world_grip = view_at(0.0).pose.compose(Pose { position: grip, orientation: Quat::IDENTITY });
        let controller = Controller {
            pose: Pose { position: grip, orientation: Quat::IDENTITY },
            grip_position: grip,
            linear_velocity: Vec3::default(),
            angular_velocity: Vec3::default(),
        };
        let located = locate(&view_at(0.05), &controller, Vec3::default(), 0.05);
        // Linear extrapolation leaves a second-order error: |grip| (ω dt)² / 2 ≈ 0.6 mm here.
        assert!(located.pose.position.sub(world_grip.position).length() < 1e-3, "{:?}", located.pose.position);
        assert!(same_rotation(located.pose.orientation, world_grip.orientation));
    }

    #[test]
    fn moving_hand_with_still_head() {
        let view = View { pose: Pose::IDENTITY, linear_velocity: Vec3::default(), angular_velocity: Vec3::default() };
        let grip = Vec3([0.1, 0.0, -0.3]);
        let aim = Pose { position: Vec3([0.1, 0.0, -0.4]), orientation: Quat::IDENTITY };
        let controller = Controller {
            pose: aim,
            grip_position: grip,
            linear_velocity: Vec3([1.0, 0.0, 0.0]),
            angular_velocity: Vec3([0.0, 2.0, 0.0]),
        };
        let located = locate(&view, &controller, Vec3::default(), 0.01);
        // The aim point 0.1 m ahead of the grip moves at v + ω × r = (1, 0, 0) + (−0.2, 0, 0).
        assert!(close(located.linear_velocity, Vec3([0.8, 0.0, 0.0])), "{:?}", located.linear_velocity);
        assert!(close(located.pose.position, Vec3([0.108, 0.0, -0.4])), "{:?}", located.pose.position);
        assert!(same_rotation(located.pose.orientation, yaw(0.02)));
    }

    #[test]
    fn extrapolation_is_capped() {
        let view = View { pose: Pose::IDENTITY, linear_velocity: Vec3::default(), angular_velocity: Vec3::default() };
        let controller = Controller {
            pose: Pose::IDENTITY,
            grip_position: Vec3::default(),
            linear_velocity: Vec3([1.0, 0.0, 0.0]),
            angular_velocity: Vec3::default(),
        };
        let located = locate(&view, &controller, Vec3::default(), 5.0);
        assert!(close(located.pose.position, Vec3([MAX_EXTRAPOLATION_S, 0.0, 0.0])));
    }
}
