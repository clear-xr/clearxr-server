# Controller protocol

Packets between the Clear XR visionOS client and `clearxr-layer`, on Clear XR's Foveated Streaming message channel. The client sends one controller packet per rendered frame; the layer sends a haptic packet whenever the game applies or stops a vibration.

## Channel

| | |
|---|---|
| UUID, as visionOS reports it | `78563412-3412-3412-1234-56789ABCDEF0` |
| Raw bytes, in order | `78 56 34 12 34 12 34 12 12 34 56 78 9A BC DE F0` |

Both sides use these 16 raw bytes: Foundation's `UUID(uuidString:)` on visionOS, the `XrUuidEXT` byte array on the PC. They are the in-memory form of the Windows `GUID` `{12345678-1234-1234-1234-56789ABCDEF0}`.

The channel also carries the client's stream configuration packets (magic `0x4346`, "CF": a little-endian u32 JSON length, then UTF-8 JSON), so receivers dispatch on the magic at offset 0.

## Conventions

- Little-endian integers and IEEE 754 binary32 floats. Every field sits at its natural alignment, so plain `#[repr(C)]` structs match the layouts.
- **Times** are headset monotonic nanoseconds (the clock behind `CACurrentMediaTime()` and ARKit timestamps). Only differences between two headset times mean anything on the PC.
- **Head frame**: the ARKit `DeviceAnchor` at the hand's `sampleTimeNs`. Meters, +X right, +Y up, −Z forward, the same axes as OpenXR `VIEW` space.
- Quaternions are x, y, z, w.
- Senders zero reserved fields and bits; receivers ignore them.
- Receivers reject a packet with an unknown version, a short packet, or a haptic packet with an undefined `hand` or `command`, and ignore bytes past the version's size. A layout change gets a new version number.

## Controller packet (client → layer), version 2, 248 bytes

| Offset | Type | Field |
|---:|---|---|
| 0 | u16 | `magic` = 0x5343 |
| 2 | u8 | `version` = 2 |
| 3 | u8 | `activeHands`: bit 0 left, bit 1 right |
| 4 | u16 | `flags`: bit 0 `HEAD_RELATIVE`, always set |
| 6 | u16 | reserved |
| 8 | u32 | `sequence` |
| 12 | u32 | reserved |
| 16 | u64 | `sendTimeNs`: when the client built the packet |
| 24 | 112 bytes | left hand |
| 136 | 112 bytes | right hand |

Each hand:

| Offset | Type | Field |
|---:|---|---|
| 0 | u64 | `sampleTimeNs`: the time the pose describes |
| 8 | u16 | `buttons` |
| 10 | u16 | `trackingFlags` |
| 12 | u32 | reserved |
| 16 | f32 × 2 | `thumbstick` x, y in [−1, 1]; +y pushed away |
| 24 | f32 | `trigger` in [0, 1] |
| 28 | f32 | `grip` in [0, 1] |
| 32 | f32 × 7 | `gripPose`: position, then orientation |
| 60 | f32 × 7 | `aimPose`: position, then orientation |
| 88 | f32 × 3 | `linearVelocity`, m/s |
| 100 | f32 × 3 | `angularVelocity`, rad/s |

`activeHands` marks the hands with a connected controller; the block of an inactive hand is ignored.

`sequence` increases by one per packet and wraps. The layer accepts the first packet after opening the channel or after 100 ms of silence; otherwise it drops a packet unless `(int32)(sequence − last) > 0`.

`gripPose` and `aimPose` are the controller's OpenXR grip and aim poses in the head frame. The velocities are the grip's velocity relative to the world, in head-frame axes. The client converts ARKit's `AccessoryAnchor.velocity` and `angularVelocity`, which are in the anchor's own axes: `ω = R_anchor · angularVelocity`, `v = R_anchor · velocity + ω × (p_grip − p_anchor)`, then both rotated by `R_headᵀ`.

`trackingFlags` bits 0–3 equal OpenXR's `XrSpaceLocationFlags`: 0 orientation valid, 1 position valid, 2 orientation tracked, 3 position tracked. Bit 4 is linear velocity valid, bit 5 angular velocity valid, bit 6 low accuracy. The client sets bits 0–5 for `positionOrientationTracked` (plus 6 for low accuracy), bits 0 and 2 for `orientationTracked`, and none when the controller or the head is untracked. A connected but untracked hand still reports its buttons.

`buttons` name the Oculus Touch inputs they drive; the Clear XR client maps GameController's `Button A` to bit 0 and `Button B` to bit 1.

| Bit | Left hand | Right hand |
|---:|---|---|
| 0 | `x/click` | `a/click` |
| 1 | `y/click` | `b/click` |
| 2 | trigger pressed | trigger pressed |
| 3 | squeeze pressed | squeeze pressed |
| 4 | `thumbstick/click` | `thumbstick/click` |
| 5 | `menu/click` | (no Touch input) |
| 6 | `x/touch` | `a/touch` |
| 7 | `y/touch` | `b/touch` |
| 8 | `trigger/touch` | `trigger/touch` |
| 9 | squeeze touched | squeeze touched |
| 10 | `thumbstick/touch` | `thumbstick/touch` |

### On the layer

A hand is connected while the last accepted packet is under 100 ms old and has the hand's `activeHands` bit. A connected hand's actions and poses come from the packets; a hand that isn't connected passes through to the runtime. A client that pauses or disconnects just stops sending.

To locate a grip or aim action space at time T, the layer locates `VIEW` at T with its velocity, rotates the hand's velocities into the base space, extrapolates the head-relative pose by `(T − receive time) + (sendTimeNs − sampleTimeNs)` (capped at 100 ms) using the hand's velocity relative to the head, and composes the result with `VIEW`.

## Haptic packet (layer → client), version 2, 24 bytes

| Offset | Type | Field |
|---:|---|---|
| 0 | u16 | `magic` = 0x4856 |
| 2 | u8 | `version` = 2 |
| 3 | u8 | `hand`: 0 left, 1 right |
| 4 | u8 | `command`: 0 apply, 1 stop |
| 5 | 3 bytes | reserved |
| 8 | u64 | `durationNs`: 0 means the shortest vibration |
| 16 | f32 | `frequency` in Hz: 0 means unspecified |
| 20 | f32 | `amplitude` in [0, 1] |

An apply replaces whatever plays on that hand; an apply with amplitude 0, or a stop, ends it. The client caps durations at 10 seconds and stops all vibration when the channel closes, the session pauses, or the controller disconnects.
