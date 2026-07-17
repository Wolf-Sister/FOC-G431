# Current Loop Auto-Tuning — Python Tool

## Overview

Python script that auto-tunes the FOC current-loop PI controller by communicating
with the STM32G431 over the existing UART2 VOFA+ protocol.  The motor must be
locked (stalled) during tuning so the electrical dynamics reduce to a pure
RL circuit.

The tool reuses the **existing PI gains** (Kp=1.485, Ki=371.25) as a known-good
safety baseline and derives motor parameters (R, L) from step-response data
before computing optimal PI gains.

---

## Architecture

```
auto_tuner.py              CLI entry point, orchestrates the tuning pipeline
├── serial_iface.py         UART2 send/receive, telemetry line parsing
├── step_test.py            Step injection + R/L extraction from Vq/Iq data
├── prbs_id.py              PRBS generation + RLS system identification
├── pi_calc.py              R, L → Kp, Ki  (pole-zero cancellation design)
└── validate.py             Verification step + before/after comparison
```

```
 PC (Python)                          MCU (STM32G431)
 ─────────────────────────────────────────────────────
 TX: "T=1.0\n"          ──────►      sets motor_control.set_torque
 TX: "P=2.0,I=400\n"    ──────►      foc_set_current_pid(...)
 RX: "channels: ...\n"  ◄──────      14-ch telemetry @ 100 Hz
                                      (iq_meas, vq_cmd, …)
```

All communication goes through a USB-TTL adapter connected to UART2
(PA15 RX / PB3 TX, 115200 8N1).  No MCU firmware changes required.

---

## Tuning Pipeline

### Phase 0 — Connect & Setup

1. Open serial port, verify MCU is alive by reading telemetry.
2. Set mode to torque: `M=0`.
3. Confirm existing PI gains are loaded (`P=1.485, I=371.25`).
4. Existing gains are kept as-is — stable current loop is the safety net
   for every test step.

### Phase 1 — Step-Response Coarse ID (~0.3 s)

1. Send `T=1.0` (step from idle 0 → 1 A).
2. Record 100 Hz telemetry: `iq_meas`, `vq_cmd` for ~100 samples.
3. Extract:
   - **Steady-state R**: `R = Vq_ss / Iq_ss`
   - **Time constant τ** from 10%→63% rise, smoothed with a 3-sample median.
   - **Inductance L**: `L = τ × R`
4. Take 3 steps (up / down / up) and median the results for robustness.
5. Sanity check: `(L, R)` must be within 5× of the values implied by the
   existing PI gains.  Outlier → flag, use median of inlier steps only.

### Phase 2 — PRBS Refined ID (~2 s) *(optional, enabled by default)*

1. Generate a 127-point maximum-length PRBS sequence with clock period 50 ms
   (i.e. hold each PRBS bit for 5 telemetry frames).
2. Amplitude ±1 A (centred on 0 A; D-axis stays 0).
3. Collect 200+ (Vq, Iq) pairs.
4. Run **Recursive Least Squares** (forgetting factor 0.98) to fit the ARX model:

   ```
   Iq[k] = a1·Iq[k-1] + b1·Vq[k-1]
   ```

5. Convert discrete → continuous (zero-order-hold inverse):

   ```
   R = (1 - a1) / b1
   L = -Ts · (1 + a1) / (2 · b1)     ← approximate for small Ts
   ```

6. Cross-check PRBS result against Phase-1 step result — both should agree
   within 30%.  Large discrepancy → warn; use step result as fallback.

### Phase 3 — PI Calculation

Design method: **pole-zero cancellation** — place the PI zero at the plant pole.

```
ωc   = target bandwidth (rad/s), default 3000 (matched to current 473 Hz)
Kp   = ωc × L
Ki   = ωc × R
```

Target bandwidth is configurable via CLI flag `--bandwidth` / `--bw`.
Default 3000 rad/s ≈ 477 Hz — well within the 20 kHz PWM limit and aligned
with what the manual-tuned gains already achieve.

### Phase 4 — Verification (~0.3 s)

1. Apply the new PI gains to the MCU: `P=<new_kp>,I=<new_ki>`.
2. Run one 0→1 A step and record:
   - **Rise time** (10% → 90%)
   - **Overshoot** (%)
   - **Steady-state error** (A)
   - **Settling time** (within 5%)
3. Compare against the same step run with the original gains (A/B comparison).
4. **If new gains perform worse** (overshoot >50% OR rise time >2× original)
   → automatically roll back to original gains and warn.
5. Print summary table.

---

## Key Parameters & Defaults

| Parameter | Default | Rationale |
|-----------|---------|-----------|
| Step amplitude | 1.0 A | 10% of LIMIT_CURRENT, safe w/ existing PI |
| Pre-test PI gains | Kp=1.485, Ki=371.25 | Already confirmed stable |
| PRBS amplitude | ±1.0 A | Same as step, balanced excitation |
| PRBS length | 127 | MLS-7, sufficient for RLS convergence |
| PRBS clock | 50 ms | 5 telemetry frames, well above RL time constant |
| RLS forgetting factor | 0.98 | Smooth convergence |
| R/L sanity bounds | 0.2× ~ 5× of existing-PI-implied values | Catches bad data |
| Target bandwidth ωc | 3000 rad/s (477 Hz) | Matches current tuning; safe under 20 kHz PWM |
| Serial port | auto-detect or `--port COM3` | |
| Baud rate | 115200 | Matches MCU config |
| Telemetry rate | 100 Hz | MCU-side, fixed |

---

## CLI Interface

```
usage: auto_tuner.py [-h] [--port COMx] [--bw RAD_S] [--no-prbs]
                     [--step-amps A] [--prbs-amps A] [--prbs-length N]
                     [--dry-run] [--save JSON_PATH]

FOC Current-Loop Auto-Tuner
  --port       Serial port (auto-detect by default)
  --bw         Target bandwidth in rad/s (default 3000)
  --no-prbs    Skip PRBS phase, use step-response only
  --step-amps  Step current amplitude (default 1.0 A)
  --prbs-amps  PRBS current amplitude (default 1.0 A)
  --prbs-length PRBS sequence length (default 127, must be 2ⁿ-1)
  --dry-run    Connect & validate but don't change PI gains
  --save       Save result to JSON file
```

---

## Output

### Console
```
=== FOC Current Loop Auto-Tuner ===
Port: COM3 @ 115200 | MCU detected | Mode: TORQUE

[Phase 1] Step-Response ID
  Step 1 (0→1A): R=0.118Ω  L=482µH  τ=4.08ms
  Step 2 (1→0A): R=0.122Ω  L=491µH  τ=4.02ms
  Step 3 (0→1A): R=0.115Ω  L=475µH  τ=4.13ms
  → Median: R=0.118Ω  L=482µH  τ=4.08ms

[Phase 2] PRBS System ID
  RLS converged at iteration 87
  → R=0.120Ω  L=488µH  (agrees w/ step within 2%)

[Phase 3] PI Calculation (ωc=3000 rad/s)
  Kp = 3000 × 0.000488 = 1.464
  Ki = 3000 × 0.120   = 360.0
  → Downloading to MCU...

[Phase 4] Verification
  ┌──────────┬──────────┬───────────┬──────────┬──────────┐
  │          │ Rise(ms) │ Overshoot │ Settle   │ SteadyErr│
  ├──────────┼──────────┼───────────┼──────────┼──────────┤
  │ Original │   1.2    │   3.1%    │  2.1ms   │ 0.008A   │
  │ New      │   1.1    │   2.8%    │  1.9ms   │ 0.006A   │
  └──────────┴──────────┴───────────┴──────────┴──────────┘
  → New gains equivalent to original (both are well-tuned)
  → Saved to auto_tune_result.json
```

### JSON output (`--save auto_tune_result.json`)
```json
{
  "R_ohm": 0.120,
  "L_uh": 488.0,
  "Kp": 1.464,
  "Ki": 360.0,
  "bandwidth_rad_s": 3000,
  "method": "prbs",
  "verification": {
    "rise_time_ms": 1.1,
    "overshoot_pct": 2.8,
    "settling_time_ms": 1.9,
    "steady_state_error_a": 0.006
  }
}
```

---

## File Structure

```
Core/Src/vofa.c              ← MCU-side protocol (NO changes needed)
Core/Src/pid.c               ← PI controller (NO changes needed)

tools/                        ← new Python package directory
├── __init__.py
├── auto_tuner.py             CLI entry
├── serial_iface.py           Serial TX/RX + telemetry frame parser
├── step_test.py              Step injection & R/L extraction
├── prbs_id.py                PRBS generator + RLS estimator
├── pi_calc.py                PI gain computation
├── validate.py               Verification step & A/B comparison
└── requirements.txt          pyserial (≥3.5)
```

---

## MCU-Side Protocol Details (no changes required)

### TX (PC → MCU)
Send ASCII commands terminated by `\n`:
```
T=1.0\n           set Iq current command to 1.0 A
D=0.0\n           set Id current command to 0 A (SPM)
P=1.46\n          set Iq Kp
I=360.0\n         set Iq Ki
M=0\n             set torque mode
```

Commands can be combined: `P=1.46,I=360.0\n`

### RX (MCU → PC)
Continuous 100 Hz telemetry lines, parsed as:
```
"channels: id_target,id_meas,set_torque,iq_meas,vd_cmd,vq_cmd,velocity,...\n"
      0         1       2          3       4      5      6
```
The script reads `iq_meas` (index 3) and `vq_cmd` (index 5).

### Step Sync
After each command the MCU sets `motor_control.status_flag=1`.
The Python script waits for this flag to toggle in telemetry before
collecting the next data point — this guarantees command/response alignment
without requiring precise timing.

---

## Error Handling

| Scenario | Response |
|----------|----------|
| Serial port not found | List available ports, exit with hint |
| No telemetry within 2 s | Timeout, suggest checking power/UART wiring |
| Step current doesn't rise | Warn: "Motor may not be stalled / phase disconnected" |
| R/L outside sanity bounds | Flag outlier, fall back to step median or existing PI |
| PRBS RLS fails to converge | Fall back to step-identified R/L |
| Verification shows worse performance | Auto-rollback to original gains, warn |
| MCU in wrong mode (SPEED/POSITION) | Auto-switch to TORQUE mode (`M=0`) |

---

## Non-Goals (Out of Scope)

- Speed-loop or position-loop tuning (current loop only for this iteration).
- Real-time plotting / GUI (pure CLI; pipe telemetry to VOFA+ or SerialPlot
  manually if visual monitoring is desired).
- Flash parameter persistence (MCU firmware change — separate task).
- Id loop independent tuning (Id uses the same R, L as Iq for an SPM motor).
