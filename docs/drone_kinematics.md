# PARKDRONE kinematics overview

## Scope and terminology

PARKDRONE currently simulates a **DJI Mavic 2 Pro-style quadrotor** in Webots.
It has four fixed, vertical propellers: front-left, front-right, rear-left and
rear-right. This note describes the aircraft body; the downward camera gimbal is
called out separately because its motion affects image georeferencing but does
not propel the drone.

Kinematics describes the geometry of position, orientation, velocity and their
rates. The reason a quadrotor moves is dynamics: rotor thrust, gravity,
inertia and aerodynamic effects. Keeping those ideas separate is useful: a
route is kinematic, while the motor mixer and attitude controller are dynamic.

For project documentation, use this state convention:

- Position: `p` denotes the position vector; `x` and `y` are the horizontal
  world coordinates emitted by the controller and `h` is altitude (`alt` in
  `poses.json`). Its notation is `p = [x, y, h]ᵀ`.
- Attitude: roll `φ`, pitch `θ`, yaw `ψ`, obtained from the IMU.
  Roll is rotation about the body forward axis, pitch about the lateral axis,
  and yaw about the vertical axis.
- Body axes: forward, left/right and up/down must be kept consistent in any
  new code. The current controller transforms horizontal GPS velocity into
  forward and lateral components using `yaw`; do not silently substitute a
  different ENU/NED convention.

## Degrees of freedom

A free rigid drone has **six configuration degrees of freedom (6-DoF)**:

| Translation | Rotation |
| --- | --- |
| `x`: east/west-like horizontal position | roll `φ`: bank left/right |
| `y`: north/south-like horizontal position | pitch `θ`: nose up/down |
| `h`: altitude | yaw `ψ`: heading |

The pose vector `q` combines those six coordinates: `q = [x, y, h, φ, θ, ψ]ᵀ`.
Velocity adds six
rates: three linear and three angular. The simulated vehicle has four motor
inputs, so it cannot command all six motion coordinates independently. It is
an **underactuated** system: it first changes attitude, and the tilted total
thrust then accelerates it horizontally.

The gimbal adds camera-joint DoF (pitch and roll, with a yaw joint recorded as
well). Those are not airframe DoF. They keep the optical axis near nadir while
the aircraft banks, pitches or turns.

## From rotors to motion

For rotor *i*, `ωᵢ` is its angular speed, `Tᵢ` is its thrust, and `k_T` is the
thrust coefficient. Its thrust is approximately proportional to the square of
its speed: `Tᵢ = k_T ωᵢ²`. `Qᵢ` is the reaction torque and `k_Q` is the
reaction-torque coefficient; the corresponding relation is `Qᵢ = k_Q ωᵢ²`.
Opposite rotor spin directions make those torques cancel in hover.

The four motor speeds are mixed into four useful virtual controls:

| Virtual control | Rotor pattern | Physical result |
| --- | --- | --- |
| Collective thrust | Raise/lower all four together | Climb, descend, or hover |
| Roll moment | Increase one side and decrease the opposite side | Bank, then gain lateral acceleration |
| Pitch moment | Increase front/rear pair differentially | Tilt nose, then gain forward/backward acceleration |
| Yaw moment | Increase one spin-direction pair and decrease the other | Rotate heading with roughly unchanged total lift |

In a hover, `m` is the drone mass, `g` is gravitational acceleration, and
`Tᵢ` is the thrust of rotor *i*; total thrust balances weight
(`Σ Tᵢ ≈ mg`) and the net roll,
pitch and yaw moments are near zero. To fly forward, the controller pitches the
body slightly; the thrust vector then gains a forward horizontal component. To
stop, it pitches the opposite way to create braking acceleration. Side motion
works identically through roll.

Conceptually, `R(φ, θ, ψ)` is the rotation matrix from body to world frame,
`p̈` is the position acceleration, `T` is total thrust, `m` is mass, `g` is
gravitational acceleration, and `d` represents disturbances. If the body
thrust is along its vertical axis, the translational model is:

`m p̈ = R(φ, θ, ψ)[0, 0, T]ᵀ − [0, 0, mg]ᵀ + d`.

Thus yaw alone changes where “forward” points; it does not directly create
translation. Pitch/roll tilt the thrust vector; collective retains sufficient
vertical thrust while tilted.

## How this drone turns and moves today

The patrol is deliberately **yaw-then-forward**, like a vehicle that points its
nose along its route. At each waypoint it:

1. Calculates the desired bearing and heading error. Here `dx` and `dy` are
   the horizontal displacement to the target, `atan2(dy, dx)` is its bearing,
   `ψ` is current yaw, and `wrap(·)` normalizes an angle to the chosen range:
   `wrap(atan2(dy, dx) − ψ)`.
2. Uses a yaw PD command to rotate toward that bearing. Forward speed is held
   at zero while the heading error exceeds about `0.5 rad` (29 degrees).
3. Commands a forward velocity target once aligned. Pitch is generated from
   forward-velocity error; roll damps lateral drift. It does not intentionally
   strafe, because prior lateral position-to-roll commands destabilised the
   model.
4. Blends the aim point toward the next waypoint within 15 m, which makes
   gentle route segments flow through rather than stop at every waypoint.

Consequently a normal corner is a coordinated sequence: brake, yaw toward the
new path, bank/pitch enough to follow it, then accelerate. A sharp route
reversal (>1.5 rad) is explicitly handled as **stop, turn in place, go**. The
drone is physically capable of combined translation and yaw, but this is the
safer behaviour selected by the present navigator.

Current survey operating points in `parkdrone.py` are a 30 m target altitude,
5.0 m/s maximum straight-line speed, 2.0 m/s sharp-corner arrival speed, and
0.8 m/s for reversals. They are controller limits rather than general Mavic
performance claims.

## Braking and turning kinematics used by the controller

The route planner assigns a slower allowable speed at corners from the change
in segment heading, then propagates that limit backwards along the route.
Here `v(d)` is the allowed speed with remaining distance `d`, `v_target` is
the target speed at the waypoint, and `a_brake` is the available deceleration:

`v(d) = √(v_target² + 2 a_brake d)`.
PARKDRONE uses `a_brake = 0.22 m/s^2`, conservatively below roughly
`0.29 m/s^2` measured in its 1 km simulation runs. This is why it starts
braking far before a sharp waypoint: distance-to-stop scales with speed squared.

For obstacle stopping, the same principle caps forward speed using the sensed
range, a 12 m stand-off, a reaction allowance, and an even more conservative
`0.15 m/s^2` deceleration. Once halted, position error is converted to a small
velocity target (maximum 0.3 m/s) for station keeping; it is not sent directly
to roll.

## Camera kinematics and mapping relevance

The camera is gimbal-stabilized to look down. With `θ` as body pitch, `φ` as
body roll, and `π/2` as the nadir pitch angle, the controller commands gimbal
pitch near `π/2 − θ` and roll near `−φ`, then records actual gimbal
pitch, roll and yaw with every captured pose when position sensors are
available. This matters because at 30 m altitude even a small off-nadir angle
shifts the ground footprint and therefore projected parking-bay polygons.

For a frame-to-ground transform, retain the full chain:

`world -> drone position/attitude -> body -> gimbal joints -> camera -> image`.

Do not assume camera yaw always equals body yaw at corners; the current pose
format records `cam_yaw` specifically to avoid that assumption.

## Useful concepts for future work

- **Reference frames and homogeneous transforms:** make every sensor and map
  transform explicit (`T_world_body`, `T_body_camera`). Essential for accurate
  image projection and sensor fusion.
- **Rotation representations:** roll/pitch/yaw are intuitive for operators but
  have singularities. Prefer rotation matrices or quaternions internally for
  filtering, interpolation and future aggressive motion.
- **Differential flatness:** a quadrotor trajectory can be planned from smooth
  position and yaw curves, then converted to desired thrust and attitude. It is
  valuable if waypoint following evolves into smooth survey trajectories.
- **Curvature and lateral acceleration:** `a_lat` is lateral acceleration, `v`
  is speed, and `R` is turn radius. For a smooth turn, `a_lat = v²/R`. This
  gives a direct speed limit from a route’s local turn radius and a chosen
  safe bank/acceleration limit, complementing the current corner-angle rule.
- **Actuator allocation (motor mixing):** maps collective/roll/pitch/yaw
  demands back to four rotor speeds while respecting saturation. Saturation is
  important because a large tilt command reduces vertical lift margin.
- **State estimation:** GPS finite differences are sufficient for the present
  simulation, but an EKF combining IMU, GNSS, barometer and visual cues is the
  usual real-flight approach. Attitude and velocity estimates must be expressed
  in clearly named frames.
- **Trajectory feasibility:** bound speed, acceleration, jerk, yaw rate,
  angular acceleration, tilt and climb rate. A route that is geometrically
  valid can still be impossible or unsafe to fly.

## Practical implications

- Plan coverage routes in position and yaw, but verify the implied curvature,
  braking distance and camera footprint before flying.
- Preserve `x`, `y`, `alt`, `yaw`, `roll`, `pitch`, and the gimbal angles in
  telemetry; they are the minimum pose information needed for reproducible
  projection.
- If adding obstacle steering, use a bounded yaw bias and retain the
  yaw-then-forward safety gate until a complete coupled lateral controller is
  tested.
- For a real aircraft, re-identify thrust, torque, motor lag, mass, inertia,
  tilt limits and achievable accelerations. Simulation tuning values must not
  be treated as hardware specifications.

## Project sources

- `sim/controllers/parkdrone/parkdrone.py` — waypoint navigation, velocity and
  attitude control, motor mixing, gimbal compensation, and measured braking
  limits.
- `sim/README.md` — simulated Mavic 2 Pro configuration and downward-camera
  behaviour.
- `sim/output/<survey_area>/poses.json` — captured pose and actual gimbal
  angles used downstream for georeferencing.
