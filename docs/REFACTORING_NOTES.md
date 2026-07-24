# Self-Balancing Robot: Architecture and Refactoring Notes

Last updated: 2026-07-24

## Purpose of this document

This document is the durable handoff for the architectural refactoring of the
ESP32-C6 self-balancing robot firmware. It records:

- the intended architecture;
- decisions already made;
- completed and current work;
- future extensions;
- the working method to follow;
- safety and real-time constraints that must not be lost during refactoring.

The refactoring must proceed in small, behaviour-preserving steps. After each
step:

1. Build the project.
2. Flash and physically verify it when the change can affect runtime behaviour.
3. Review the dependency boundary, not only compiler success.
4. Create a focused Git commit.

The user is implementing the changes. Codex should act as an architectural
mentor: analyze the existing code, explain the problem and the reason for the
next change, answer questions, and review results. Codex should not write the
implementation unless the user explicitly changes this agreement.

## Product direction

The final robot should:

- balance using the MPU6050, Kalman filter and PID controller;
- use one DRV8825 per motor;
- move forward and backward while continuing to balance;
- turn left and right using differential wheel commands;
- operate as a Wi-Fi access point and self-contained server;
- serve its HTML, CSS and JavaScript interface from ESP32 storage;
- permit live PID and target-angle configuration;
- save accepted configuration in NVS;
- stream live telemetry, including measured angle, through WebSocket;
- stop remote motion safely if the client disconnects or stops sending commands.

Networking and motion commands are intentionally deferred until the existing
firmware has clearer ownership and module boundaries.

## Architectural style

This project is evolving toward a modular, event-driven embedded application.
It combines several patterns:

- **Finite-state machine:** `controller.c` coordinates robot lifecycle and
  handles events.
- **Context objects:** structures such as `controller_ctx_t`,
  `balance_control_t` and `drv8825_t` own persistent state.
- **Facade:** `drivebase` hides the number, orientation and implementation of
  physical motor drivers.
- **Message passing:** FreeRTOS queues and notifications communicate between
  tasks instead of allowing arbitrary shared-state writes.
- **Control pipeline:** sensor acquisition, angle estimation, balance control,
  wheel mixing and motor actuation remain distinct stages.
- **Dependency direction:** the high-level controller depends on subsystem
  interfaces; low-level drivers do not depend on the controller.

The intended direction is:

```text
Web client
    |
    v
HTTP/WebSocket adapter
    |
    +----> Configuration service ----> NVS
    |
    +----> Transient motion commands
                  |
                  v
MPU reader --> Motion/angle control --> Balance control --> Drivebase
                                                        |       |
                                                        v       v
                                                    left     right
                                                    DRV8825  DRV8825
```

The application controller coordinates these modules but should not implement
their low-level behaviour.

## Decisions already made

### PID and target-angle configuration

- `Kp`, `Ki` and `Kd` are each valid from `0.0f` through `50.0f`.
- The nominal target angle is `90.0f`.
- The configurable target-angle range is `70.0f` through `110.0f`.
- A configuration update contains all three PID coefficients together.
- Updates may be applied while the robot is in `CTRL_BALANCING`.
- Applying and saving are one user action.
- PID history (`integral` and `prev_error`) is reset when coefficients change.
- ESP-IDF NVS will provide persistence; no external EEPROM is required.
- The ESP32 must validate every update. JavaScript validation is only a
  usability feature and is not a safety boundary.

Because applying also saves, a web slider must not write NVS continuously for
every movement event. It should submit on release or explicit confirmation.

### Motion commands

Persistent configuration and transient motion commands are different data:

```text
Persistent: Kp, Ki, Kd, target angle, configured limits
Transient:  forward/backward request, turn request, stop/enable
```

Transient motion commands must never be restored from NVS after reboot.

Forward/backward motion should eventually be produced through a limited
target-angle offset or an outer velocity controller, not by blindly adding a
constant to both motor outputs. Turning should use differential left/right
commands.

Remote control must operate while balancing. `CTRL_REMOTE_CONTROL` should not
replace the balance loop. It may be removed or reinterpreted as an operating
mode within `CTRL_BALANCING`.

A remote-command watchdog/dead-man timeout is mandatory. If commands stop
arriving, forward and turn requests return to zero while balancing continues.

### Configuration organization

`main/robot_config.h` is the central location for values that configure this
robot or its externally meaningful behaviour:

- GPIO assignments;
- driver speed/acceleration limits;
- default PID values and permitted ranges;
- target-angle default and range;
- balance safety angles;
- MPU queue, timing and task settings where these are system tuning choices.

Implementation-only constants remain private to their modules. Examples:

- log tags;
- timer slot count;
- event-bit positions;
- protocol byte sizes;
- local diagnostic logging intervals.

All macros should have a descriptive namespace, such as `ROBOT_`,
`MPU_READER_` or `DRV8825_`.

## Git and repository status

Repository:

```text
https://github.com/JuliuChe/selfBalancingRobot.git
```

Branch:

```text
main
```

The latest committed checkpoint at the time of this document is:

```text
c5391b0 added config to robot config.h
```

That checkpoint includes the current `robot_config.h`, initial `drivebase`
files, DRV8825 cleanup, and MPU/I2C configuration cleanup. Some of those
features remain incomplete even though their current state is committed. At
the time this document was created, only this documentation file was
untracked. Always run `git status` before continuing.

Generated paths such as `build/` and `managed_components/` are intentionally
ignored. `dependencies.lock`, `sdkconfig`, and the customized local MPU6050
component are retained.

## Completed refactoring

### Controller event contract

Controller event types and messages were extracted into:

```text
main/controller_events.h
```

The timer no longer includes the complete controller header. This removed a
circular header dependency.

### Private controller state

`controller_ctx_t` was moved out of the public header and into
`controller.c`. `controller.h` now exposes only the controller task entry
point.

This prevents future modules, especially networking code, from directly
modifying controller internals.

### Balance-control calculation

PID calculation, target angle and output clamping were extracted into:

```text
main/balance_control.h
main/balance_control.c
```

`balance_control_t` owns PID runtime state, target angle and maximum output.
The module computes one scalar balance command and has no dependency on
FreeRTOS, MPU6050, DRV8825, Wi-Fi or NVS.

This separation is foundational for the future pipeline:

```text
balance command + steering command --> wheel mixer --> left/right motors
```

## Work currently in progress

### `robot_config.h`

`main/robot_config.h` exists and already contains most intended categories:

- PID defaults and permitted ranges;
- target-angle default and range;
- safety-angle limits;
- DRV8825 GPIO and speed/acceleration settings;
- I2C GPIO and port settings;
- MPU interrupt, queue, period, timeout, stack and priority settings.

Migration is incomplete. Search for obsolete or duplicate names after every
edit. In particular, at the time of this document:

- `controller.c` still contains commented former configuration macros;
- `controller.c` still uses `TARGET_ANGLE` in one initialization and should
  use `ROBOT_TARGET_ANGLE_DEFAULT`;
- generic and duplicated configuration definitions may still exist in
  `i2c_devices.h`, `mpu_reader.h` and driver sources.

Do not move every numeric literal into `robot_config.h`. Protocol facts and
module implementation details should remain local and be given focused names
only when that improves meaning.

### Drivebase facade

The intended first interface is:

```text
controller --> drivebase --> current shared DRV8825
```

`main/drivebase.h` exists and currently wraps one `drv8825_t` named
`shared_driver`. However, `drivebase.c` is still only a stub and
`controller.c` still owns `drv8825_t one_driver` and directly calls
`drv8825_*`.

The current drivebase step is complete only when:

- `drivebase.c` implements init, start, stop, deinit and balance application;
- motor GPIO and hardware configuration are consumed by `drivebase.c`;
- `controller_ctx_t` owns `drivebase_t`, not `drv8825_t`;
- `controller.c` contains no direct `drv8825_*` calls;
- current one-driver balancing behaviour remains unchanged.

Do not add the second driver in the same step. First establish and test the
facade with the existing hardware.

### MPU reader cleanup

`mpu_reader.c` is being reviewed for hard-coded values and ownership. Current
cleanup includes moving user/system tuning values to `robot_config.h` and
keeping protocol/internal constants local.

Known items to review:

- remove duplicated, unused `MPU_FRAME_SIZE` definitions;
- name the private FIFO capacity and performance-log interval;
- use named event bits;
- use `uint32_t` for the result of `ulTaskNotifyTake()`, which returns a
  32-bit value;
- keep timeout values and their log messages consistent;
- express FIFO frame sizes as sensor values, for example
  `3U * sizeof(int16_t)` for three accelerometer axes;
- later extract a private big-endian `int16_t` frame-decoding helper;
- remove dead and commented legacy blocks after Git preserves them;
- review the shared output-buffer/semaphore design separately.

A likely bug was identified in `mpu_reader_stop()`: forced deletion appears to
be performed when a stopped bit is present instead of when it is absent. Fix
and test that behaviour in its own commit rather than mixing it with constant
cleanup.

The duplicated controller event-queue creation was also identified:
`controller_task()` must create and validate the queue because it is required
to enqueue the initial `EV_INIT`; the `CTRL_INIT` handler should not attempt to
create it again.

## Planned refactoring sequence

### 1. Finish configuration centralization

Finish replacing robot-level hard-coded values with names from
`robot_config.h`. Remove redundant definitions and build.

Success criterion: each configurable robot value has one definition, and
private implementation constants remain owned by their modules.

### 2. Complete the one-driver drivebase facade

Implement `drivebase` as a behaviour-preserving wrapper around the existing
shared DRV8825. Replace all direct controller-to-DRV8825 interactions.

Success criterion:

```text
rg "drv8825" main/controller.c
```

returns no matches, and the physical robot behaves as before.

### 3. Clean the current DRV8825 module

Review dead and overlapping APIs such as `drv8825_task()` and old speed-setting
paths. Keep the currently useful sine diagnostic temporarily.

`drv8825_sine_task()` is a manual hardware diagnostic, not production driver
behaviour. It should eventually move to:

```text
test_apps/drv8825_smoke/
```

Do not leave a motor-starting diagnostic entry point in production firmware
long term.

### 4. Extract DRV8825 into its own ESP-IDF component

Target structure:

```text
components/
└── drv8825/
    ├── CMakeLists.txt
    ├── include/
    │   └── drv8825.h
    └── drv8825.c

test_apps/
└── drv8825_smoke/
    ├── CMakeLists.txt
    └── main/
        └── motor_sine_test.c
```

The production drivebase and standalone test firmware will reuse the same
component.

### 5. Add the second motor driver inside drivebase

After the facade is stable, change drivebase internals from:

```text
shared_driver
```

to:

```text
left_driver
right_driver
```

Drivebase will own:

- each driver's pins and lifetime;
- physical direction inversion caused by opposite motor mounting;
- start, stop and deinitialization of both drivers;
- output clamping;
- left/right differential mixing.

The controller interface should remain stable.

### 6. Separate the real-time balance loop from the FSM

`motor_control_task()` still lives in `controller.c` and receives the entire
controller context. Create a narrower balance-loop context or service that
receives only:

- measured angle and angular velocity;
- balance controller;
- drivebase;
- fault/stop notification mechanism.

The FSM owns lifecycle policy; the balance loop owns deterministic repeated
control.

### 7. Continue disentangling `controller.c`

Keep the FSM as the application orchestrator, but move subsystem behaviour
behind focused interfaces. Review:

- initialization and cleanup ownership;
- error propagation;
- task creation and lifetime;
- event posting;
- timer ownership;
- LED/status policy;
- state handlers that perform too many unrelated operations.

Avoid splitting code merely to reduce line count. Extract a module only when
it has a coherent responsibility and a clear interface.

### 8. Introduce authoritative runtime configuration

Create one configuration service for:

```c
kp
ki
kd
target_angle
```

It must:

- validate finite values and configured ranges;
- accept all PID coefficients atomically;
- synchronize updates with the active balance loop;
- reset PID history on accepted coefficient changes;
- apply updates during `CTRL_BALANCING`;
- expose a snapshot for telemetry and the web API;
- reject the whole update if any value is invalid.

Networking must never write directly into `balance_control_t` or
`controller_ctx_t`.

### 9. Add NVS persistence

Load saved configuration at boot, validate it, and fall back to compiled
defaults if it is absent or corrupt. Save only accepted user updates.

Required actions:

- apply and save;
- restore compiled defaults;
- retain a version/schema number for future configuration changes.

### 10. Add transient motion/angle commands

Define a bounded command such as:

```text
forward: -1.0 through +1.0
turn:    -1.0 through +1.0
```

Initially, forward/backward may become a limited target-angle offset. Later an
outer velocity loop may calculate that offset. Turning becomes a differential
left/right command in drivebase.

Motion commands are never stored in NVS. Add a watchdog that returns commands
to zero when the client stops refreshing them.

### 11. Add the self-contained web server

The ESP32 should create its own Wi-Fi access point and serve:

- an embedded HTML page;
- CSS and JavaScript assets;
- HTTP endpoints for configuration/status;
- a WebSocket endpoint for telemetry and transient commands.

For a small interface, embed static web assets into the firmware binary. Add a
filesystem only if asset size or independent web updates justify it.

Suggested responsibilities:

```text
HTTP GET  /api/config       read current configuration
HTTP PUT  /api/config       validate, apply and save configuration
WebSocket /ws               telemetry and transient motion commands
```

The JavaScript page will provide PID controls, target-angle controls,
forward/back/left/right controls and live plots.

### 12. Add telemetry and live angle view

Publish telemetry snapshots at a bounded rate independent of the fast balance
loop. Likely fields:

- measured roll angle;
- angular velocity;
- target angle;
- PID output/balance command;
- left and right motor commands;
- controller state;
- active PID values;
- fault/connection status;
- configuration revision.

Do not send WebSocket messages from the real-time balance loop. The loop should
update a snapshot or overwrite queue; a lower-priority networking task sends
it. Slow clients must never delay balancing.

### 13. Safety and verification

Add focused tests or diagnostic applications for:

- PID reset and output clamping;
- invalid/NaN/infinite configuration rejection;
- target-angle limits;
- left/right mixing and motor inversion;
- loss of MPU data;
- client disconnect and motion-command timeout;
- NVS corruption and fallback;
- safe task shutdown and resource cleanup;
- standalone DRV8825 hardware smoke testing.

## Build-system warning

This is an ESP-IDF project. Microsoft CMake Tools must be disabled for this
workspace. Generic CMake previously configured `build/` with the Visual Studio
2019 generator and corrupted the ESP-IDF cache.

Use:

```text
ESP-IDF: Build Your Project
```

or an initialized ESP-IDF terminal:

```powershell
idf.py reconfigure
idf.py build
```

Do not use:

```text
CMake: Configure
CMake: Build
```

If the build directory is contaminated or incomplete, remove generated
`build/` and regenerate it with ESP-IDF.

## Guidance for a new Codex conversation

Start a new conversation from the repository root and use:

> Read `docs/REFACTORING_NOTES.md` and inspect the current code and Git status.
> Act as an architectural professor and mentor. The user writes the code; do
> not implement changes unless explicitly asked. Work one small,
> behaviour-preserving refactoring at a time. Explain what is wrong, what the
> next change should focus on, and why. Review the result after each successful
> build before proceeding. Preserve the real-time balance loop and safety
> constraints.

Always inspect the current Git status and source before assuming that an
in-progress item in this document has been completed.
