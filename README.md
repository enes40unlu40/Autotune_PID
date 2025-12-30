# PID Autotune (Relay Feedback) – Embedded C

Relay feedback (bang-bang) based PID auto-tuning for a position loop.  
The tuner drives the actuator with `+h` / `-h` around a reference angle, measures the resulting oscillation amplitude and period, computes the ultimate gain `Kc`, and derives `Kp/Ki/Kd` using mode-dependent tuning coefficients.

> Typical use-case: quick on-device tuning for a motorized pan/tilt axis.

---

## How it works (Algorithm Summary)

This implementation is a **relay feedback autotune**:

1. Move the plant to the target reference `refDeg`.
2. Apply relay excitation:
   - If error `e = (AcDeg - refDeg)` is positive, command `-h`
   - Else command `+h`
3. Sample error at a fixed sampling period `Ts` (driven by `g_autoTunePIDTimer.Flag`).
4. During sampling, track:
   - `TempErrorMax` (max error)
   - `TempErrorMin` (min error)
   - Peak timestamps to estimate oscillation period `T`
5. Compute oscillation amplitude:
   - `A = (TempErrorMax - TempErrorMin) / 2`
6. Compute ultimate gain (Åström–Hägglund relay method):
   - `Kc = 4*h / (pi*A)`
7. Convert the measured period to seconds:
   - `T_sec = T_ticks * Ts`
8. Compute PID parameters using a mode table:
   - `Kp = KpFilter(mode) * Kc`
   - `Ti = KiFilter(mode) * T_sec`
   - `Td = KdFilter(mode) * T_sec`
   - `Ki = Kp / Ti`
   - `Kd = Kp * Td`

---

## State Machine

The autotune runs as a small finite-state machine:

### Case 0 – Init & Move to Reference
- Resets internal autotune accumulators
- Commands the axis to move to `refDeg`
- Transition to Case 1 when: `|ErrVal| < REF_TOL`

### Case 1 – Relay Test (Oscillation Capture)
- Starts a periodic sampling timer (Timer3)
- Applies relay excitation (`±h`)
- Each sample tick:
  - Updates min/max error
  - Detects peaks and estimates period `T`
- Transition to Case 2 when: `Counter >= MAX_SAMPLES`

### Case 2 – Compute Gains
- Stops the sampling timer
- Computes `A`, `Kc`, `T_sec`, then `Kp/Ki/Kd`
- Transition to Case 3

### Case 3 – Publish Gains & Stop
- Copies tuned gains to the active PID controller (e.g., global `Kp/Ki/Kd`)
- Ends autotune

---

## Key Parameters & Units

| Parameter | Meaning | Unit / Notes |
|---|---|---|
| `refDeg` | target reference angle | degrees |
| `AcDeg` | measured angle (feedback) | degrees |
| `ErrVal` | error = `AcDeg - refDeg` | degrees |
| `h` | relay excitation magnitude | **must match actuator command units** used by `SendMovePacket()` |
| `Ts` | sampling period | seconds (derived from timer flag period) |
| `Counter` | sample index | ticks |
| `T` | oscillation period estimate | ticks (convert via `Ts`) |
| `A` | oscillation amplitude | degrees (from error min/max) |
| `Kc` | ultimate gain | depends on scaling of `h` vs `A` |
| `mode` | coefficient set selection | selects row from `tuningTable[]` |

---

## Integration Requirements (Dependencies)

You must provide these platform-specific functions/objects:

- `float GetFDegData(void);`  
  Returns the measured position in **degrees**.

- `void GoToPosPIDParam(float refDeg, uint32_t timeout_ms);`  
  Moves axis to `refDeg` (blocking or non-blocking, but logic assumes it will converge).

- `void SendMovePacket(float cmd);`  
  Applies actuator command. Autotune uses `+h` / `-h`.

- `void SetTimer3State(timer_state_t state);`  
  Starts/stops a periodic timer that sets `g_autoTunePIDTimer.Flag`.

- `volatile struct { uint8_t Flag; } g_autoTunePIDTimer;`  
  `Flag` must be set to `1` exactly every sampling period.

- `tuningTable[]` and `GainFilter`  
  Table of tuning coefficients:
  - `KpFilter`, `KiFilter`, `KdFilter` (naming can differ)
  Interpreted as multipliers for `Kc` and `T_sec`.

---

## Example Usage Flow

1. Set target reference:
   - `atPID.refDeg = 30.0f;`
2. Select tuning mode:
   - `atPID.mode = 0; // e.g., conservative/aggressive`
3. Set relay amplitude:
   - `atPID.h = 5000.0f; // actuator units`
4. Call `AutoTuneHandler(&atPID)` periodically in your main loop/task until it reaches Case 3.
5. Read resulting gains:
   - `atPID.Kp`, `atPID.Ki`, `atPID.Kd`

---

## Practical Notes / Safety

- **Clamp output:** `SendMovePacket(±h)` should be saturated to safe limits.
- **Avoid division blow-up:** if `A` is too small (no oscillation), abort or clamp `A`.
- **Mechanical limits:** ensure the axis can oscillate around `refDeg` without hitting end-stops.
- **Noise sensitivity:** peak detection using raw error derivative can be noisy; consider filtering `ErrVal` or adding hysteresis.
- **Sampling period accuracy:** `Ts` must be stable; jitter directly distorts `T_sec`.

---

## Known Implementation Pitfalls (Fix before publishing)

If your code matches the snippet structure:

- Do **not** reset `mode` inside `ResetAutoPidParams()` if you want the caller-selected mode to persist.
- Ensure `currentFilter` is available in Case 2 (store it in `atPID`, make it static, or re-load see below).
- Ensure `AcDeg` is declared in the correct scope (shared across cases).

Recommended pattern:
- Declare temporary variables before the `switch`.
- Load `currentFilter = tuningTable[atPID->mode]` in Case 2 as well, or store `mode` and re-load.

---

## Files

- `autotune_pid.c/.h` – autotune state machine and gain computation
- `pid_controller.c/.h` – runtime PID control loop (not included here)
- `platform_motor_if.c/.h` – hardware-specific glue: timer, motor packets, sensors

---

## License

Add your preferred license (MIT/BSD-3/Apache-2.0).  
If this is part of a company/internal project, clarify redistribution rules here.
