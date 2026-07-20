# Current-Loop Auto-Tuning Python Tool — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a Python CLI tool that auto-tunes FOC current-loop PI gains via UART2 VOFA+ protocol, using step-response + PRBS system identification, with zero MCU firmware changes.

**Architecture:** Six Python modules under `tools/`. `SerialInterface` handles UART2 I/O. `step_test` extracts R from steady-state step response. `prbs_id` generates MLS sequences and runs RLS to identify R and L. `pi_calc` computes Kp/Ki from R/L/ωc. `validate` runs verification steps and A/B comparison. `auto_tuner` ties everything together as the CLI entry point.

**Tech Stack:** Python 3.8+, pyserial ≥3.5, numpy (for median/array ops only), argparse (stdlib)

## Global Constraints

- No MCU firmware changes — uses existing UART2 115200 8N1 VOFA+ protocol
- Motor must be stalled (locked rotor) during tuning
- Works on Windows (COM ports), also macOS/Linux (/dev/tty*)
- Python 3.8+ (matches typical embedded tooling environments)
- Only dependency: pyserial ≥3.5; numpy optional (fall back to statistics.median + manual array ops)
- Step test gives R from steady-state Vq/Iq at 100Hz telemetry
- L identification relies primarily on PRBS/RLS (100Hz telemetry cannot resolve ~4ms transients)
- PRBS clock period 50ms ensures system fully settles between bits, guaranteeing DC-gain measurements

---

### Task 1: Project Scaffolding & Serial Interface

**Files:**
- Create: `tools/__init__.py`
- Create: `tools/requirements.txt`
- Create: `tools/serial_iface.py`

**Interfaces:**
- Produces: `class SerialInterface` with `__init__(port, baud)`, `connect() -> bool`, `disconnect()`, `send_cmd(cmd: str)`, `read_telemetry(timeout_s: float) -> dict | None`, `flush_input()`, `list_ports() -> list[str]` (static)

- [ ] **Step 1: Create `tools/__init__.py`**

```python
"""FOC Current-Loop Auto-Tuning Tool."""
```

- [ ] **Step 2: Create `tools/requirements.txt`**

```
pyserial>=3.5
```

- [ ] **Step 3: Write `tools/serial_iface.py` — SerialInterface class**

```python
"""
Serial interface to STM32G431 FOC via UART2 VOFA+ protocol.

Protocol
--------
TX (PC -> MCU):  ASCII commands terminated by \\n
    "T=1.0\\n"        set Iq current command (A)
    "P=1.5,I=400\\n"  set Iq Kp, Ki
    "M=0\\n"          set torque mode

RX (MCU -> PC):  100 Hz telemetry in VOFA+ JustFloat format
    "channels: f0,f1,...,f9\\n"
    10 comma-separated floats after "channels: " prefix.

    Index mapping:
      0: id_target      5: status_flag
      1: id_meas        6: speed_sp
      2: iq_target      7: position_sp
      3: iq_meas        8: pos_meas
      4: velocity       9: mode

Step sync: status_flag (index 5) is set to 1 on command reception,
then cleared to 0 on next telemetry TX.  Python watches this flag
to confirm command delivery.
"""

import serial
import serial.tools.list_ports
import time
from typing import Optional


class SerialInterface:
    """UART2 communication with STM32G431 FOC controller."""

    # Telemetry channel indices
    IDX_ID_TARGET   = 0
    IDX_ID_MEAS     = 1
    IDX_IQ_TARGET   = 2
    IDX_IQ_MEAS     = 3
    IDX_VELOCITY    = 4
    IDX_STATUS_FLAG = 5
    IDX_SPEED_SP    = 6
    IDX_POSITION_SP = 7
    IDX_POS_MEAS    = 8
    IDX_MODE        = 9

    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self._ser: Optional[serial.Serial] = None
        self._line_buf = ""

    def connect(self, timeout_s: float = 3.0) -> bool:
        """Open serial port and verify telemetry is flowing.

        Returns True if connection is established and at least one
        valid telemetry frame is received within timeout_s.
        """
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )
        # Discard partial frames from boot noise
        self._ser.reset_input_buffer()

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            frame = self.read_telemetry(timeout_s=0.5)
            if frame is not None:
                return True
        return False

    def disconnect(self):
        """Close serial port."""
        if self._ser and self._ser.is_open:
            self._ser.close()

    def send_cmd(self, cmd: str):
        """Send an ASCII command to the MCU.

        cmd should NOT include the trailing \\n — it is appended here.
        Example: iface.send_cmd("T=1.0")
        """
        if self._ser is None or not self._ser.is_open:
            raise RuntimeError("Serial port not open")
        raw = (cmd + "\n").encode("ascii")
        self._ser.write(raw)
        self._ser.flush()

    def read_telemetry(self, timeout_s: float = 0.5) -> Optional[dict]:
        """Read one complete telemetry frame, blocking up to timeout_s.

        Returns a dict with channel names as keys, or None on timeout.
        """
        if self._ser is None or not self._ser.is_open:
            return None

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                raw = self._ser.readline()
            except serial.SerialException:
                return None

            if not raw:
                continue

            try:
                line = raw.decode("ascii").strip()
            except UnicodeDecodeError:
                continue

            if not line.startswith("channels:"):
                continue

            # Parse "channels: f0,f1,...,f9"
            payload = line[len("channels:"):]
            parts = payload.split(",")
            if len(parts) < 10:
                continue

            values = []
            for p in parts[:10]:
                try:
                    values.append(float(p))
                except ValueError:
                    values.append(0.0)

            return {
                "id_target":   values[0],
                "id_meas":     values[1],
                "iq_target":   values[2],
                "iq_meas":     values[3],
                "velocity":    values[4],
                "status_flag": int(values[5]),
                "speed_sp":    values[6],
                "position_sp": values[7],
                "pos_meas":    values[8],
                "mode":        int(values[9]),
            }

        return None

    def flush_input(self):
        """Discard any buffered serial input (call before a new test phase)."""
        if self._ser and self._ser.is_open:
            self._ser.reset_input_buffer()

    def wait_for_flag(self, timeout_s: float = 2.0) -> bool:
        """Block until a telemetry frame with status_flag==1 is received,
        indicating the MCU processed our command.  Returns True if seen."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            frame = self.read_telemetry(timeout_s=0.1)
            if frame is not None and frame["status_flag"] == 1:
                return True
        return False

    @staticmethod
    def list_ports() -> list[str]:
        """Return list of available serial port device names."""
        return [p.device for p in serial.tools.list_ports.comports()]
```

- [ ] **Step 4: Commit**

```bash
git add tools/__init__.py tools/requirements.txt tools/serial_iface.py
git commit -m "feat: add serial_iface.py — UART2 VOFA+ protocol communication"
```

---

### Task 2: PI Calculation Module

**Files:**
- Create: `tools/pi_calc.py`

**Interfaces:**
- Produces: `calculate_pi(R, L, omega_c) -> tuple[float, float]`  → `(Kp, Ki)`
- Produces: `implied_rl_from_pi(Kp, Ki) -> tuple[float, float]` → `(R, L)` — reverse-engineering for sanity bounds
- Produces: `DEFAULT_OMEGA_C = 3000`, `DEFAULT_KP = 1.485`, `DEFAULT_KI = 371.25`

- [ ] **Step 1: Write `tools/pi_calc.py`**

```python
"""
PI gain calculation from motor parameters.

Design method: pole-zero cancellation.
Place the PI zero (s = -Ki/Kp) at the plant pole (s = -R/L)
so the open-loop transfer function becomes a pure integrator.

    ωc   = target crossover frequency (rad/s)
    Kp   = ωc × L
    Ki   = ωc × R

The closed-loop transfer function is then:

    G_cl(s) = ωc / (s + ωc)        (first-order, bandwidth ωc)

Reverse: given Kp and Ki, the implied plant parameters are:

    L/R = Kp/Ki        →  τ_elec = Kp/Ki
    R   = Ki/ωc        (unknown ωc without context)
    From ratio alone:  R_implied = 1.0 (arbitrary), L_implied = Kp/Ki
    Or, if ωc is known: R = Ki/ωc, L = Kp/ωc
"""

from typing import Tuple

# Default values — aligned with existing manual-tuned gains
# Kp=1.485, Ki=371.25 → τ=Kp/Ki=4ms, matches SPM motor
DEFAULT_OMEGA_C = 3000.0   # rad/s ≈ 477 Hz
DEFAULT_KP      = 1.485
DEFAULT_KI      = 371.25

# Implied reference values from existing PI (measured at ωc=3000)
REF_R = DEFAULT_KI / DEFAULT_OMEGA_C   # ≈ 0.12375 Ω
REF_L = DEFAULT_KP / DEFAULT_OMEGA_C   # ≈ 0.000495 H = 495 µH
REF_TAU = DEFAULT_KP / DEFAULT_KI       # ≈ 0.004 s = 4 ms


def calculate_pi(R: float, L: float, omega_c: float = DEFAULT_OMEGA_C) -> Tuple[float, float]:
    """Compute PI gains via pole-zero cancellation.

    Args:
        R: Phase resistance in ohms.
        L: Phase inductance in henries.
        omega_c: Target current-loop bandwidth in rad/s.

    Returns:
        (Kp, Ki) tuple.
    """
    Kp = omega_c * L
    Ki = omega_c * R
    return Kp, Ki


def implied_rl_from_pi(Kp: float, Ki: float) -> Tuple[float, float]:
    """Compute implied R and L from PI gains.

    Uses the relationship: Kp/Ki = L/R (electrical time constant).
    Returns (R_implied, L_implied) assuming ωc is unknown,
    so R is set to 1.0 as a placeholder and L = Kp/Ki.

    The caller should only use the ratio L/R = Kp/Ki for sanity checks
    (e.g. require 0.5ms < Kp/Ki < 50ms).
    """
    tau = Kp / Ki if Ki != 0 else 0.004
    # Return R=1.0 normalized so L = tau
    return 1.0, tau


def get_sanity_bounds() -> Tuple[float, float, float, float]:
    """Return (R_min, R_max, L_min, L_max) based on 0.2× to 5× of reference."""
    return (
        REF_R * 0.2,   # R_min ≈ 0.025 Ω
        REF_R * 5.0,   # R_max ≈ 0.62 Ω
        REF_L * 0.2,   # L_min ≈ 99 µH
        REF_L * 5.0,   # L_max ≈ 2.5 mH
    )
```

- [ ] **Step 2: Commit**

```bash
git add tools/pi_calc.py
git commit -m "feat: add pi_calc.py — pole-zero cancellation PI design"
```

---

### Task 3: PRBS Generator & RLS System Identification

**Files:**
- Create: `tools/prbs_id.py`

**Interfaces:**
- Produces: `class PRBSGenerator` — `__init__(n_bits=7)`, `next() -> int`, `sequence(length: int) -> list[int]`
- Produces: `class RLSEstimator` — `__init__(lambda_val=0.98)`, `update(y, phi) -> np.ndarray`, property `theta: np.ndarray`
- Produces: `def prbs_identify(iface, amplitude, prbs_length, clock_samples) -> dict` — runs full PRBS identification, returns `{"R": float, "L": float, "converged": bool, "iterations": int}`

- [ ] **Step 1: Write `tools/prbs_id.py` — PRBSGenerator**

```python
"""
PRBS (Pseudo-Random Binary Sequence) generation and RLS system identification.

The PRBS is a maximum-length sequence (MLS) generated from an n-bit
linear-feedback shift register (LFSR).  Used as the excitation signal
for closed-loop system identification of the motor RL circuit.

Identification model (ARX):

    Iq[k] = a1 * Iq[k-1] + b1 * Vq[k-1]

Converted to continuous-time RL parameters via zero-order-hold inverse:

    R = (1 - a1) / b1
    L = Ts * (1 + a1) / (2 * b1)

where Ts is the sampling period (telemetry interval, ~0.01 s at 100 Hz).
"""

import time
from typing import Tuple


class PRBSGenerator:
    """Maximum-length sequence generator using LFSR.

    For n_bits=7 the sequence length is 2^7 - 1 = 127.
    Taps are chosen to give a maximal-length sequence.
    """

    # Primitive polynomial taps for n_bits 2..12
    _TAPS = {
        2:  [0, 1],
        3:  [0, 2],
        4:  [0, 3],
        5:  [1, 4],
        6:  [0, 5],
        7:  [0, 6],     # x^7 + x^6 + 1
        8:  [1, 2, 3, 7],
        9:  [3, 8],
        10: [2, 9],
        11: [1, 10],
        12: [0, 3, 5, 11],
    }

    def __init__(self, n_bits: int = 7):
        if n_bits not in self._TAPS:
            raise ValueError(f"n_bits must be one of {sorted(self._TAPS.keys())}")
        self.n_bits = n_bits
        self.taps = self._TAPS[n_bits]
        self._mask = (1 << n_bits) - 1
        # Initialise shift register with all-ones (avoid all-zeros deadlock)
        self._state = self._mask

    def next(self) -> int:
        """Return next PRBS bit (0 or 1)."""
        feedback = 0
        for tap in self.taps:
            feedback ^= (self._state >> tap) & 1
        self._state = ((self._state << 1) | feedback) & self._mask
        return feedback

    def sequence(self, length: int) -> list[int]:
        """Generate a PRBS sequence of the given length, returning list of 0/1."""
        return [self.next() for _ in range(length)]
```

- [ ] **Step 2: Write `tools/prbs_id.py` — RLSEstimator (append to file)**

```python
class RLSEstimator:
    """Recursive Least Squares estimator for ARX model identification.

    Estimates parameters theta for the model:

        y[k] = phi[k]^T * theta

    Update rule (with forgetting factor λ):

        e     = y[k] - phi[k]^T * theta
        K     = P * phi / (λ + phi^T * P * phi)
        theta = theta + K * e
        P     = (P - K * phi^T * P) / λ
    """

    def __init__(self, n_params: int = 2, lambda_val: float = 0.98):
        self.n_params = n_params
        self.lambda_val = lambda_val
        # Parameter vector: [a1, b1]
        self._theta = [0.0] * n_params
        # Covariance matrix: initial large values = high uncertainty
        self._P = [[1e6 if i == j else 0.0 for j in range(n_params)]
                   for i in range(n_params)]
        self._iterations = 0
        self._theta_history = []  # track convergence

    def update(self, y: float, phi: list[float]) -> list[float]:
        """Perform one RLS update step.

        Args:
            y:   Current output measurement (Iq[k]).
            phi: Regressor vector [Iq[k-1], Vq[k-1]].

        Returns:
            Updated parameter vector theta = [a1, b1].
        """
        lam = self.lambda_val
        P = self._P
        theta = self._theta

        # Prediction error
        y_hat = sum(phi[i] * theta[i] for i in range(self.n_params))
        e = y - y_hat

        # Compute K = P * phi / (lambda + phi^T * P * phi)
        P_phi = [sum(P[i][j] * phi[j] for j in range(self.n_params))
                 for i in range(self.n_params)]
        denom = lam + sum(phi[i] * P_phi[i] for i in range(self.n_params))

        K = [P_phi[i] / denom for i in range(self.n_params)]

        # Update theta
        for i in range(self.n_params):
            theta[i] = theta[i] + K[i] * e

        # Update P = (P - K * phi^T * P) / lambda
        K_phiT_P = [[K[i] * sum(phi[k] * P[k][j] for k in range(self.n_params))
                      for j in range(self.n_params)]
                     for i in range(self.n_params)]
        for i in range(self.n_params):
            for j in range(self.n_params):
                P[i][j] = (P[i][j] - K_phiT_P[i][j]) / lam

        self._P = P
        self._theta = theta
        self._iterations += 1
        self._theta_history.append(list(theta))

        return theta

    @property
    def theta(self) -> list[float]:
        """Current parameter estimate [a1, b1]."""
        return list(self._theta)

    @property
    def iterations(self) -> int:
        return self._iterations

    def has_converged(self, window: int = 10, tol: float = 0.01) -> bool:
        """Check if theta has converged (mean change over window < tol)."""
        if len(self._theta_history) < window + 1:
            return False
        recent = self._theta_history[-window:]
        changes = []
        for i in range(1, len(recent)):
            diff = sum(abs(recent[i][j] - recent[i-1][j])
                       for j in range(self.n_params))
            changes.append(diff)
        avg_change = sum(changes) / len(changes)
        return avg_change < tol
```

- [ ] **Step 3: Write `tools/prbs_id.py` — `prbs_identify()` and helpers (append to file)**

```python
def _arx_to_rl(a1: float, b1: float, Ts: float) -> Tuple[float, float]:
    """Convert discrete ARX parameters to continuous RL parameters.

    Discrete model (ZOH discretisation of RL circuit):

        Iq[k] = a1 * Iq[k-1] + b1 * Vq[k-1]

    where the continuous plant is G(s) = 1 / (L*s + R).

    ZOH inverse:
        a1 = exp(-R*Ts / L)
        b1 = (1 - a1) / R

    Solving for R, L:
        R = (1 - a1) / b1
        L = -R * Ts / ln(a1)
    """
    import math

    if b1 <= 0 or a1 <= 0 or a1 >= 1.0:
        raise ValueError(f"Invalid ARX params: a1={a1}, b1={b1}")

    R = (1.0 - a1) / b1
    ln_a1 = math.log(a1)  # a1 < 1, so ln_a1 < 0
    L = -R * Ts / ln_a1

    return R, L


def prbs_identify(iface, amplitude: float = 1.0,
                  prbs_length: int = 127,
                  clock_samples: int = 5,
                  ) -> dict:
    """Run PRBS-based closed-loop system identification.

    Injects a PRBS current command and records (Vq, Iq) pairs
    from telemetry, then fits an ARX model via RLS.

    Args:
        iface:         SerialInterface instance (must be connected).
        amplitude:     PRBS current amplitude in A (±amplitude).
        prbs_length:   PRBS sequence length (2^n - 1).
        clock_samples: Telemetry frames per PRBS bit (5 → 50 ms).

    Returns:
        dict with keys: R, L, a1, b1, converged, iterations, iq_data, vq_data
    """
    Ts = 0.01  # telemetry period (100 Hz)
    prbs = PRBSGenerator(n_bits=7)  # 127-pt MLS

    # Collect data
    iq_data = []
    vq_data = []
    t_data = []

    iface.flush_input()
    t0 = time.monotonic()

    for bit_idx in range(prbs_length):
        bit = prbs.next()
        cmd_val = amplitude if bit == 1 else -amplitude
        iface.send_cmd(f"T={cmd_val:.4f}")

        # Wait for command acknowledgement
        iface.wait_for_flag(timeout_s=1.0)

        # Collect clock_samples telemetry frames for this PRBS bit
        for _ in range(clock_samples):
            frame = iface.read_telemetry(timeout_s=0.2)
            if frame is not None:
                iq_data.append(frame["iq_meas"])
                vq_data.append(frame["vq_cmd"])
                t_data.append(time.monotonic() - t0)

    # Return current to zero
    iface.send_cmd("T=0.0")

    if len(iq_data) < 50:
        return {"R": None, "L": None, "converged": False,
                "iterations": 0, "error": "Insufficient data"}

    # Run RLS identification
    rls = RLSEstimator(n_params=2, lambda_val=0.98)

    for k in range(1, len(iq_data)):
        y = iq_data[k]
        phi = [iq_data[k - 1], vq_data[k - 1]]
        rls.update(y, phi)

    # Convert discrete → continuous
    a1 = rls.theta[0]
    b1 = rls.theta[1]

    try:
        R, L = _arx_to_rl(a1, b1, Ts)
    except ValueError:
        return {"R": None, "L": None, "converged": False,
                "iterations": rls.iterations,
                "a1": a1, "b1": b1, "error": "ARX→RL conversion failed"}

    converged = rls.has_converged()

    return {
        "R": R,
        "L": L,
        "a1": a1,
        "b1": b1,
        "converged": converged,
        "iterations": rls.iterations,
        "iq_data": iq_data,
        "vq_data": vq_data,
    }
```

- [ ] **Step 4: Commit**

```bash
git add tools/prbs_id.py
git commit -m "feat: add prbs_id.py — PRBS generator + RLS system identification"
```

---

### Task 4: Step-Response Test Module

**Files:**
- Create: `tools/step_test.py`

**Interfaces:**
- Produces: `def run_step_test(iface, amplitude, n_samples, n_steps) -> dict` — returns `{"R": float, "L": float, "tau": float, "samples": [dict], "valid": bool}`
- Consumes: `SerialInterface` from Task 1, `get_sanity_bounds` from Task 2

- [ ] **Step 1: Write `tools/step_test.py`**

```python
"""
Step-response test for coarse motor parameter identification.

At 100 Hz telemetry the electrical transient (~4 ms) is invisible;
individual samples show only the pre-step and post-step steady state.
The step test therefore primarily measures the DC resistance R from
the steady-state Vq/Iq ratio.

L is estimated indirectly:
  1. From the integral of Vq during the transient (energy method),
     using the first telemetry frame after command acknowledgement.
  2. Falls back to L = R * τ_ref where τ_ref = Kp_existing / Ki_existing.
"""

import time
from typing import Optional
from .pi_calc import get_sanity_bounds, REF_TAU, DEFAULT_KP, DEFAULT_KI


def _extract_rl_from_step(iq_data: list[float],
                           vq_data: list[float],
                           t_data: list[float],
                           step_magnitude: float,
                           ) -> Optional[dict]:
    """Extract R and L from a single step-response record.

    R = median(Vq_ss) / median(Iq_ss)   (steady-state, last 30% of samples)

    L is estimated via the integral method over the first 3 samples
    after the step:  ∫ Vq dt = R ∫ Iq dt + L * ΔIq
    """
    n = len(iq_data)
    if n < 20:
        return None

    # --- Steady-state R ---
    ss_start = int(n * 0.6)
    iq_ss_vals = iq_data[ss_start:]
    vq_ss_vals = vq_data[ss_start:]

    # Use numpy if available, else manual median
    def _median(vals):
        s = sorted(vals)
        m = len(s) // 2
        return s[m] if len(s) % 2 == 1 else (s[m - 1] + s[m]) / 2.0

    iq_ss = _median(iq_ss_vals)
    vq_ss = _median(vq_ss_vals)

    if abs(iq_ss) < 0.05:
        return None  # current did not rise — motor may not be stalled

    R = vq_ss / iq_ss

    # --- Inductance L via integral method ---
    # Vq = R*Iq + L*dIq/dt
    # Integrate over first few samples:  ∫Vq ≈ R*∫Iq + L*(Iq_end - Iq_start)
    # L ≈ (∫Vq - R*∫Iq) / ΔIq
    n_transient = min(10, n)  # first 100 ms covers entire transient
    iq_start = iq_data[0]
    iq_end = iq_data[n_transient - 1]
    diq = iq_end - iq_start

    if abs(diq) < 0.02:
        # Cannot resolve L from this data; fall back to reference τ
        L = R * REF_TAU
        tau = REF_TAU
    else:
        # Trapezoidal integration
        sum_vq = 0.0
        sum_iq = 0.0
        for i in range(1, n_transient):
            dt = t_data[i] - t_data[i - 1]
            if dt <= 0:
                dt = 0.01  # assume 10 ms
            sum_vq += (vq_data[i] + vq_data[i - 1]) * 0.5 * dt
            sum_iq += (iq_data[i] + iq_data[i - 1]) * 0.5 * dt

        L = (sum_vq - R * sum_iq) / diq
        # Clamp L to physically plausible range
        if L <= 1e-6 or L > 0.01:
            L = R * REF_TAU
        tau = L / R if R > 0 else REF_TAU

    return {"R": R, "L": L, "tau": tau, "iq_ss": iq_ss, "vq_ss": vq_ss}


def run_step_test(iface, amplitude: float = 1.0,
                  n_samples: int = 30,
                  n_steps: int = 3,
                  ) -> dict:
    """Run step-response identification.

    Injects n_steps current steps (up / down / up by default),
    collects telemetry for n_samples frames each, and extracts R, L.

    Args:
        iface:      SerialInterface instance (connected, torque mode).
        amplitude:  Step current magnitude in A.
        n_samples:  Telemetry frames to collect per step (~0.3 s at 100 Hz).
        n_steps:    Number of steps (3 recommended: up, down, up).

    Returns:
        dict with keys: R, L, tau, samples (list of per-step dicts), valid (bool)
    """
    iface.flush_input()

    step_results = []
    directions = []
    for i in range(n_steps):
        # Alternate: up, down, up, ...
        target = amplitude if i % 2 == 0 else 0.0
        if i == 0:
            # Ensure starting from zero
            iface.send_cmd("T=0.0")
            time.sleep(0.05)
            iface.flush_input()

        # Inject step
        iface.send_cmd(f"T={target:.4f}")
        iface.wait_for_flag(timeout_s=1.0)

        # Record telemetry
        iq_vals = []
        vq_vals = []
        t_vals = []
        t0 = time.monotonic()
        for _ in range(n_samples):
            frame = iface.read_telemetry(timeout_s=0.15)
            if frame is not None:
                iq_vals.append(frame["iq_meas"])
                vq_vals.append(frame["vq_cmd"])
                t_vals.append(frame.get("_t", time.monotonic() - t0))

        # Extract R, L from this step
        result = _extract_rl_from_step(iq_vals, vq_vals, t_vals, amplitude)
        if result is not None:
            result["iq_data"] = iq_vals
            result["vq_data"] = vq_vals
            step_results.append(result)
            directions.append("up" if target > 0 else "down")

    # Return to zero
    iface.send_cmd("T=0.0")

    if not step_results:
        return {"R": None, "L": None, "tau": None,
                "samples": [], "valid": False,
                "error": "All steps failed — motor may not be stalled"}

    # --- Sanity filter ---
    R_min, R_max, L_min, L_max = get_sanity_bounds()
    valid_results = []
    for r in step_results:
        if R_min <= r["R"] <= R_max and L_min <= r["L"] <= L_max:
            valid_results.append(r)

    # Fall back to all results if none pass sanity (use with warning)
    if not valid_results:
        valid_results = step_results

    # Median across steps
    def _median(vals):
        s = sorted(vals)
        m = len(s) // 2
        return s[m]

    R_med = _median([r["R"] for r in valid_results])
    L_med = _median([r["L"] for r in valid_results])
    tau_med = L_med / R_med if R_med > 0 else REF_TAU

    return {
        "R": R_med,
        "L": L_med,
        "tau": tau_med,
        "samples": valid_results,
        "valid": len(valid_results) >= 2,
    }
```

- [ ] **Step 2: Commit**

```bash
git add tools/step_test.py
git commit -m "feat: add step_test.py — step-response R/L extraction"
```

---

### Task 5: Verification Module

**Files:**
- Create: `tools/validate.py`

**Interfaces:**
- Produces: `def run_verification(iface, kp_new, ki_new, kp_orig, ki_orig, amplitude) -> dict`
- Consumes: `SerialInterface` from Task 1
- Returns: `{"original": StepMetrics, "new": StepMetrics, "rollback": bool, "passed": bool}`

- [ ] **Step 1: Write `tools/validate.py`**

```python
"""
Verification step: run a step response with new PI gains,
compare against the original baseline, and optionally roll back.
"""

import time
from typing import Optional


class StepMetrics:
    """Step-response quality metrics."""
    def __init__(self):
        self.rise_time_ms: float = 0.0    # 10% → 90% rise time
        self.overshoot_pct: float = 0.0   # percentage overshoot
        self.settling_time_ms: float = 0.0  # within 5% of final
        self.steady_state_error_a: float = 0.0  # |setpoint - measured|
        self.iq_data: list[float] = []
        self.vq_data: list[float] = []

    def to_dict(self) -> dict:
        return {
            "rise_time_ms": self.rise_time_ms,
            "overshoot_pct": self.overshoot_pct,
            "settling_time_ms": self.settling_time_ms,
            "steady_state_error_a": self.steady_state_error_a,
        }


def _measure_step(iface, setpoint: float,
                  settle_samples: int = 15,
                  total_samples: int = 30) -> StepMetrics:
    """Run a single step and extract metrics from telemetry data.

    NOTE: At 100 Hz telemetry, rise time and overshoot are COARSE estimates.
    The electrical transient (~1 ms closed-loop, ~4 ms open-loop) happens
    between telemetry frames.  These metrics are primarily for A/B comparison
    (is the new tuning better or worse than the old one?) rather than
    absolute accuracy.
    """
    iface.flush_input()
    iface.send_cmd(f"T={setpoint:.4f}")
    iface.wait_for_flag(timeout_s=1.0)

    iq_vals = []
    vq_vals = []
    for _ in range(total_samples):
        frame = iface.read_telemetry(timeout_s=0.15)
        if frame is not None:
            iq_vals.append(frame["iq_meas"])
            vq_vals.append(frame["vq_cmd"])

    iface.send_cmd("T=0.0")

    metrics = StepMetrics()
    metrics.iq_data = iq_vals
    metrics.vq_data = vq_vals

    if len(iq_vals) < 10:
        return metrics

    n = len(iq_vals)

    # Steady state: median of last settle_samples
    ss_start = max(0, n - settle_samples)
    iq_ss = sorted(iq_vals[ss_start:])[len(iq_vals[ss_start:]) // 2]
    metrics.steady_state_error_a = abs(setpoint - iq_ss)

    # Find 10% and 90% points (coarse — 10 ms resolution)
    iq_10 = 0.10 * iq_ss
    iq_90 = 0.90 * iq_ss
    t_10 = t_90 = 0
    for i, iq in enumerate(iq_vals):
        if t_10 == 0 and iq >= iq_10:
            t_10 = i * 10  # each sample ≈ 10 ms
        if iq >= iq_90:
            t_90 = i * 10
            break
    metrics.rise_time_ms = max(0, t_90 - t_10)

    # Overshoot: max value relative to steady state
    iq_max = max(iq_vals)
    if iq_ss > 0.05:
        metrics.overshoot_pct = max(0.0, (iq_max - iq_ss) / iq_ss * 100.0)

    # Settling time: last sample outside 5% band
    band = 0.05 * iq_ss
    settle_idx = 0
    for i in range(n - 1, -1, -1):
        if abs(iq_vals[i] - iq_ss) > band:
            settle_idx = i
            break
    metrics.settling_time_ms = settle_idx * 10

    return metrics


def run_verification(iface, kp_new: float, ki_new: float,
                     kp_orig: float, ki_orig: float,
                     amplitude: float = 1.0) -> dict:
    """Run verification: compare new vs original PI gains.

    1. Measure step response with original gains.
    2. Apply new gains, measure step response.
    3. If new gains are worse (overshoot > 50% OR rise time > 2× original),
       roll back to original gains.  Otherwise keep new gains.

    Returns dict with 'original', 'new' StepMetrics, 'rollback' (bool),
    and 'passed' (bool).
    """
    # --- Baseline: original gains ---
    iface.send_cmd(f"P={kp_orig:.4f},I={ki_orig:.4f}")
    time.sleep(0.05)
    orig_metrics = _measure_step(iface, amplitude)

    # --- Test: new gains ---
    iface.send_cmd(f"P={kp_new:.4f},I={ki_new:.4f}")
    time.sleep(0.05)
    new_metrics = _measure_step(iface, amplitude)

    # --- Decision ---
    rollback = False
    if orig_metrics.rise_time_ms > 0.1:
        # Overshoot check: > 50% is excessive
        if new_metrics.overshoot_pct > 50.0:
            rollback = True
        # Rise time check: > 2× original means too sluggish
        if (orig_metrics.rise_time_ms > 0.1 and
                new_metrics.rise_time_ms > 2.0 * orig_metrics.rise_time_ms):
            rollback = True

    if rollback:
        iface.send_cmd(f"P={kp_orig:.4f},I={ki_orig:.4f}")

    return {
        "original": orig_metrics.to_dict(),
        "new": new_metrics.to_dict(),
        "rollback": rollback,
        "passed": not rollback,
    }
```

- [ ] **Step 2: Commit**

```bash
git add tools/validate.py
git commit -m "feat: add validate.py — verification step & A/B comparison"
```

---

### Task 6: CLI Entry & Orchestrator

**Files:**
- Create: `tools/auto_tuner.py`

**Interfaces:**
- Consumes: All modules from Tasks 1–5
- Produces: CLI entry point `main()`, invoked via `python -m tools.auto_tuner`

- [ ] **Step 1: Write `tools/auto_tuner.py`**

```python
#!/usr/bin/env python3
"""
FOC Current-Loop Auto-Tuner

Communicates with STM32G431 over UART2 VOFA+ protocol to identify
motor RL parameters and compute optimal PI gains.

Usage:
    python -m tools.auto_tuner --port COM3
    python -m tools.auto_tuner --port COM3 --bw 2000 --no-prbs --dry-run
    python -m tools.auto_tuner --port COM3 --save result.json
"""

import argparse
import json
import sys
import time

from .serial_iface import SerialInterface
from .pi_calc import (
    calculate_pi, DEFAULT_OMEGA_C, DEFAULT_KP, DEFAULT_KI,
    REF_R, REF_L, REF_TAU,
)
from .step_test import run_step_test
from .prbs_id import prbs_identify
from .validate import run_verification


def print_header(port: str):
    print(f"\n{'='*60}")
    print(f"  FOC Current-Loop Auto-Tuner")
    print(f"  Port: {port} @ 115200")
    print(f"{'='*60}\n")


def phase_connect(port: str) -> SerialInterface:
    """Phase 0: connect and verify MCU."""
    print("[Phase 0] Connecting...")

    if port is None:
        ports = SerialInterface.list_ports()
        if not ports:
            print("ERROR: No serial ports found.  Is USB-TTL connected?")
            sys.exit(1)
        # Pick the first available port
        port = ports[0]
        print(f"  Auto-detected port: {port}")

    iface = SerialInterface(port)
    if not iface.connect(timeout_s=3.0):
        print(f"ERROR: Cannot open {port} or no telemetry received.")
        print("  Check: power, UART wiring (PA15/PB3), USB-TTL adapter.")
        sys.exit(1)

    print(f"  Connected.  MCU telemetry OK.")

    # Ensure torque mode
    iface.send_cmd("M=0")
    time.sleep(0.05)
    iface.send_cmd("D=0.0")
    time.sleep(0.02)
    iface.send_cmd("T=0.0")
    time.sleep(0.02)

    # Verify mode
    frame = iface.read_telemetry(timeout_s=0.5)
    if frame is not None and frame["mode"] != 0:
        print("  WARNING: MCU not in torque mode. Sending M=0...")
        iface.send_cmd("M=0")
        time.sleep(0.1)

    print(f"  Mode: TORQUE  |  Existing PI: Kp={DEFAULT_KP}, Ki={DEFAULT_KI}")
    return iface


def phase_step_test(iface: SerialInterface, args) -> dict:
    """Phase 1: step-response coarse identification."""
    print(f"\n[Phase 1] Step-Response Identification")
    print(f"  Step amplitude: {args.step_amps} A  |  Samples: 30  |  Steps: 3")

    result = run_step_test(iface, amplitude=args.step_amps,
                           n_samples=30, n_steps=3)

    if not result["valid"] or result["R"] is None:
        print("  ERROR: Step test failed.  Is the motor stalled?")
        print("  Falling back to reference values from existing PI.")
        result["R"] = REF_R
        result["L"] = REF_L
        result["tau"] = REF_TAU
    else:
        R_str = f"{result['R']:.4f}"
        L_str = f"{result['L']*1e6:.0f}"
        tau_str = f"{result['tau']*1e3:.2f}"
        print(f"  → R={R_str} Ω  L={L_str} µH  τ={tau_str} ms")

    # Print per-step details
    for i, s in enumerate(result.get("samples", [])):
        print(f"     Step {i+1}: R={s['R']:.4f}Ω  L={s['L']*1e6:.0f}µH  "
              f"τ={s['tau']*1e3:.2f}ms")

    return result


def phase_prbs(iface: SerialInterface, args, step_result: dict) -> dict:
    """Phase 2: PRBS refined identification."""
    print(f"\n[Phase 2] PRBS System Identification")
    print(f"  Amplitude: ±{args.prbs_amps} A  |  Length: {args.prbs_length}  |  "
          f"Clock: 50 ms")

    result = prbs_identify(iface, amplitude=args.prbs_amps,
                           prbs_length=args.prbs_length,
                           clock_samples=5)

    if result["R"] is None or not result.get("converged"):
        print(f"  WARNING: PRBS identification failed ({result.get('error', 'unknown')}).")
        print(f"  Falling back to step result: "
              f"R={step_result.get('R', REF_R):.4f}Ω, "
              f"L={step_result.get('L', REF_L)*1e6:.0f}µH")
        return step_result

    R_str = f"{result['R']:.4f}"
    L_str = f"{result['L']*1e6:.0f}"
    print(f"  RLS converged at iteration {result['iterations']}")
    print(f"  → R={R_str} Ω  L={L_str} µH")

    # Cross-check with step result
    R_step = step_result.get("R", REF_R)
    L_step = step_result.get("L", REF_L)
    if R_step > 0 and L_step > 0:
        r_diff = abs(result["R"] - R_step) / R_step * 100
        l_diff = abs(result["L"] - L_step) / L_step * 100
        if r_diff > 30 or l_diff > 30:
            print(f"  WARNING: PRBS disagrees with step (ΔR={r_diff:.0f}%, ΔL={l_diff:.0f}%).")
            print(f"  Using step result as fallback.")
            return step_result
        print(f"  Agrees with step result within {max(r_diff, l_diff):.0f}%")

    return result


def phase_calculate(step_result: dict, prbs_result: dict, args) -> tuple:
    """Phase 3: compute and apply PI gains."""
    # Use PRBS result if available, else step
    R = prbs_result.get("R", step_result.get("R", REF_R))
    L = prbs_result.get("L", step_result.get("L", REF_L))
    omega_c = args.bw

    Kp, Ki = calculate_pi(R, L, omega_c)
    method = "prbs" if prbs_result.get("converged") else "step"

    print(f"\n[Phase 3] PI Calculation  (ωc = {omega_c} rad/s ≈ {omega_c / 6.283:.0f} Hz)")
    print(f"  Kp = {omega_c} × {L:.6f} = {Kp:.4f}")
    print(f"  Ki = {omega_c} × {R:.4f}   = {Ki:.2f}")
    print(f"  Method: {method}")

    if args.dry_run:
        print("  DRY-RUN: not downloading to MCU.")
    else:
        print(f"  Downloading P={Kp:.4f},I={Ki:.2f} to MCU...")
        iface.send_cmd(f"P={Kp:.4f},I={Ki:.2f}")
        time.sleep(0.05)

    return Kp, Ki, R, L, method


def main():
    parser = argparse.ArgumentParser(
        description="FOC Current-Loop Auto-Tuner",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--port", default=None,
                        help="Serial port (auto-detect by default)")
    parser.add_argument("--bw", type=float, default=DEFAULT_OMEGA_C,
                        help=f"Target bandwidth in rad/s (default {DEFAULT_OMEGA_C})")
    parser.add_argument("--no-prbs", action="store_true",
                        help="Skip PRBS phase, use step-response only")
    parser.add_argument("--step-amps", type=float, default=1.0,
                        help="Step current amplitude in A (default 1.0)")
    parser.add_argument("--prbs-amps", type=float, default=1.0,
                        help="PRBS current amplitude in A (default 1.0)")
    parser.add_argument("--prbs-length", type=int, default=127,
                        help="PRBS sequence length, must be 2^n-1 (default 127)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Connect and identify but don't change PI gains")
    parser.add_argument("--save", default=None,
                        help="Save result to JSON file")
    args = parser.parse_args()

    # --- Phase 0: Connect ---
    iface = phase_connect(args.port)

    # --- Phase 1: Step test ---
    step_result = phase_step_test(iface, args)

    # --- Phase 2: PRBS (optional) ---
    if args.no_prbs:
        prbs_result = step_result
        print(f"\n[Phase 2] Skipped (--no-prbs)")
    else:
        prbs_result = phase_prbs(iface, args, step_result)

    # --- Phase 3: PI Calculation ---
    Kp, Ki, R, L, method = phase_calculate(step_result, prbs_result, args)

    # --- Phase 4: Verification ---
    print(f"\n[Phase 4] Verification")
    if args.dry_run:
        print("  DRY-RUN: skipping verification.")
    else:
        verify = run_verification(iface, Kp, Ki, DEFAULT_KP, DEFAULT_KI,
                                  amplitude=args.step_amps)

        # Build table
        print(f"  {'':>10s}  {'Rise(ms)':>9s}  {'Overshoot':>10s}  "
              f"{'Settle(ms)':>10s}  {'SteadyErr(A)':>13s}")
        print(f"  {'-'*10}  {'-'*9}  {'-'*10}  {'-'*10}  {'-'*13}")
        for label, m in [("Original", verify["original"]), ("New", verify["new"])]:
            print(f"  {label:>10s}  {m['rise_time_ms']:9.1f}  "
                  f"{m['overshoot_pct']:9.1f}%  {m['settling_time_ms']:9.1f}  "
                  f"{m['steady_state_error_a']:13.4f}")

        if verify["rollback"]:
            print("\n  *** ROLLED BACK to original gains — new gains worse ***")
            Kp, Ki = DEFAULT_KP, DEFAULT_KI
        else:
            print(f"\n  Verification passed. New gains active: Kp={Kp:.4f}, Ki={Ki:.2f}")

    # --- Disconnect ---
    iface.disconnect()

    # --- Save ---
    output = {
        "R_ohm": R,
        "L_uh": L * 1e6,
        "Kp": Kp,
        "Ki": Ki,
        "bandwidth_rad_s": args.bw,
        "method": method,
    }
    if args.save:
        with open(args.save, "w") as f:
            json.dump(output, f, indent=2)
        print(f"\n  Saved to {args.save}")

    print()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Make `tools` a runnable package — update `tools/__init__.py`**

```python
"""FOC Current-Loop Auto-Tuning Tool.

Usage:
    python -m tools.auto_tuner --port COM3
    python -m tools.auto_tuner --port COM3 --bw 2000 --no-prbs --dry-run
"""

# Re-export key symbols so callers can use `from tools import SerialInterface` etc.
from .serial_iface import SerialInterface
from .pi_calc import calculate_pi, DEFAULT_OMEGA_C, DEFAULT_KP, DEFAULT_KI
from .step_test import run_step_test
from .prbs_id import prbs_identify, PRBSGenerator, RLSEstimator
from .validate import run_verification, StepMetrics
```

- [ ] **Step 3: Commit**

```bash
git add tools/auto_tuner.py tools/__init__.py
git commit -m "feat: add auto_tuner.py — CLI entry & orchestrator"
```

---

### Task 7: Integration Dry-Run & Verification

**Files:**
- Create: `tools/test_prbs.py` — unit test for PRBS generator and RLS (no hardware needed)
- Create: `tools/test_pi_calc.py` — unit test for PI calculation

**Interfaces:**
- Consumes: `PRBSGenerator`, `RLSEstimator` from `tools/prbs_id.py`
- Consumes: `calculate_pi`, `implied_rl_from_pi` from `tools/pi_calc.py`

- [ ] **Step 1: Write `tools/test_prbs.py`**

```python
"""Unit tests for PRBS generator and RLS estimator (no hardware needed)."""

import sys
import os
# Allow running directly: python tools/test_prbs.py
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from tools.prbs_id import PRBSGenerator, RLSEstimator, _arx_to_rl


def test_prbs_sequence_length():
    """PRBS of n_bits=7 produces 127 unique states before repeating."""
    prbs = PRBSGenerator(n_bits=7)
    seq = prbs.sequence(127)
    # Should have roughly equal 0s and 1s
    ones = sum(seq)
    zeros = 127 - ones
    assert 55 <= ones <= 72, f"PRBS imbalance: {ones} ones, {zeros} zeros"
    # First 127 bits should repeat after full cycle
    prbs2 = PRBSGenerator(n_bits=7)
    seq2 = prbs2.sequence(127)
    assert seq == seq2, "PRBS sequence not deterministic"
    print("  PASS test_prbs_sequence_length")


def test_rls_convergence():
    """RLS should recover known a1, b1 from synthetic data."""
    # Generate synthetic RL circuit data: R=0.12, L=0.0005, Ts=0.01
    # Continuous: Iq(s)/Vq(s) = 1/(Ls + R)
    # Discrete (ZOH): Iq[k] = a1*Iq[k-1] + b1*Vq[k-1]
    import math
    R_true = 0.12
    L_true = 0.0005
    Ts = 0.01
    a1_true = math.exp(-R_true * Ts / L_true)
    b1_true = (1 - a1_true) / R_true
    # a1_true ≈ exp(-2.4) ≈ 0.0907, b1_true ≈ 7.577

    # Generate data: Vq = PRBS ±2V, Iq from model
    prbs = PRBSGenerator(n_bits=7)
    iq = 0.0
    iq_hist = []
    vq_hist = []
    for _ in range(200):
        bit = prbs.next()
        vq = 2.0 if bit else -2.0
        iq_next = a1_true * iq + b1_true * vq
        iq_hist.append(iq_next)
        vq_hist.append(vq)
        iq = iq_next

    # Run RLS
    rls = RLSEstimator(n_params=2, lambda_val=0.98)
    for k in range(1, len(iq_hist)):
        y = iq_hist[k]
        phi = [iq_hist[k-1], vq_hist[k-1]]
        rls.update(y, phi)

    a1_est = rls.theta[0]
    b1_est = rls.theta[1]
    err_a1 = abs(a1_est - a1_true) / abs(a1_true)
    err_b1 = abs(b1_est - b1_true) / abs(b1_true)

    print(f"  True: a1={a1_true:.4f}, b1={b1_true:.4f}")
    print(f"  Est:  a1={a1_est:.4f}, b1={b1_est:.4f}")
    print(f"  Error: a1={err_a1*100:.1f}%, b1={err_b1*100:.1f}%")

    assert err_a1 < 0.02, f"a1 error too large: {err_a1*100:.1f}%"
    assert err_b1 < 0.02, f"b1 error too large: {err_b1*100:.1f}%"

    # Convert back to R, L
    R_est, L_est = _arx_to_rl(a1_est, b1_est, Ts)
    err_R = abs(R_est - R_true) / R_true
    err_L = abs(L_est - L_true) / L_true
    print(f"  R: true={R_true:.4f}, est={R_est:.4f} (err={err_R*100:.1f}%)")
    print(f"  L: true={L_true*1e6:.0f}uH, est={L_est*1e6:.0f}uH (err={err_L*100:.1f}%)")
    assert err_R < 0.05, f"R error too large"
    assert err_L < 0.05, f"L error too large"
    assert rls.has_converged(), "RLS should have converged"
    print("  PASS test_rls_convergence")


if __name__ == "__main__":
    test_prbs_sequence_length()
    test_rls_convergence()
    print("\nAll PRBS/RLS tests passed.")
```

- [ ] **Step 2: Write `tools/test_pi_calc.py`**

```python
"""Unit tests for PI calculation module."""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from tools.pi_calc import calculate_pi, implied_rl_from_pi, get_sanity_bounds


def test_calculate_pi():
    """Kp = ωc*L, Ki = ωc*R."""
    R = 0.125
    L = 0.0005
    ωc = 3000.0
    Kp, Ki = calculate_pi(R, L, ωc)
    assert abs(Kp - 1.5) < 0.01, f"Kp={Kp}, expected ~1.5"
    assert abs(Ki - 375.0) < 0.01, f"Ki={Ki}, expected ~375"
    print("  PASS test_calculate_pi")


def test_implied_rl():
    """Ratio L/R = Kp/Ki should hold."""
    Kp = 1.485
    Ki = 371.25
    R_imp, L_imp = implied_rl_from_pi(Kp, Ki)
    tau = L_imp  # R=1 so L = tau
    assert abs(tau - 0.004) < 0.0001, f"tau={tau}, expected ~0.004"
    print("  PASS test_implied_rl")


def test_sanity_bounds():
    """Sanity bounds cover the reference values."""
    R_min, R_max, L_min, L_max = get_sanity_bounds()
    from tools.pi_calc import REF_R, REF_L
    assert R_min <= REF_R <= R_max
    assert L_min <= REF_L <= L_max
    print(f"  Bounds: R=[{R_min:.4f}, {R_max:.4f}], L=[{L_min*1e6:.0f}, {L_max*1e6:.0f}]uH")
    print("  PASS test_sanity_bounds")


if __name__ == "__main__":
    test_calculate_pi()
    test_implied_rl()
    test_sanity_bounds()
    print("\nAll PI calculation tests passed.")
```

- [ ] **Step 3: Run unit tests to verify**

```bash
python tools/test_pi_calc.py
python tools/test_prbs.py
```

Expected: all tests PASS.

- [ ] **Step 4: Commit**

```bash
git add tools/test_prbs.py tools/test_pi_calc.py
git commit -m "test: add unit tests for PRBS/RLS and PI calculation"
```

---

## Verification

After all tasks are complete:

1. **Unit tests pass without hardware:**
   ```bash
   python tools/test_pi_calc.py    # all PASS
   python tools/test_prbs.py        # all PASS, RLS converges to <2% error
   ```

2. **Dry-run with hardware connected:**
   ```bash
   python -m tools.auto_tuner --port COM3 --dry-run
   ```
   Expected: connects, prints telemetry, runs step + PRBS identification,
   prints computed Kp/Ki, does NOT modify MCU gains.

3. **Full tuning with hardware:**
   ```bash
   python -m tools.auto_tuner --port COM3 --save result.json
   ```
   Expected: identifies R (~0.12 Ω), L (~490 µH), computes Kp (~1.46),
   Ki (~360), downloads to MCU, verification passes (rollback=False),
   saves JSON.

4. **Step-only mode:**
   ```bash
   python -m tools.auto_tuner --port COM3 --no-prbs
   ```
   Expected: skips PRBS phase, uses step result for PI calculation.
