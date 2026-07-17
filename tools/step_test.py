"""Step-response test for motor R/L parameter extraction.

Measures motor resistance from steady-state Vq/Iq and inductance via
the integral method over the step transient.  Falls back to the reference
time constant (REF_TAU) when the inductance estimate is unreliable.

Dependencies: stdlib only (time, typing).
"""

from __future__ import annotations

import time
from typing import Any, Dict, List, Optional

from .pi_calc import get_sanity_bounds, REF_TAU


# ---------------------------------------------------------------------------
# Median helper (no numpy)
# ---------------------------------------------------------------------------

def _median(vals: List[float]) -> float:
    """Return the median of a list of numbers.

    Sorts the list and picks the middle element (odd length) or the
    average of the two middle elements (even length).

    Args:
        vals: List of numeric values.

    Returns:
        Median value.
    """
    s = sorted(vals)
    m = len(s) // 2
    return s[m] if len(s) % 2 == 1 else (s[m - 1] + s[m]) / 2.0


# ---------------------------------------------------------------------------
# Step-response extraction
# ---------------------------------------------------------------------------

def _extract_rl_from_step(
    iq_data: List[float],
    vq_data: List[float],
    t_data: List[float],
    step_magnitude: float,
) -> Optional[Dict[str, float]]:
    """Extract R and L from a single step-response.

    Steady-state R is obtained from the median Vq / median Iq over the
    last 30% of the collected samples.  L is estimated via the integral
    method over the first 10 samples:

        L = (integral(Vq) - R * integral(Iq)) / delta_Iq

    When the current rise is too small (|diq| < 0.02) or the resulting
    L falls outside [1e-6, 0.01], L falls back to ``R * REF_TAU``.

    Args:
        iq_data: Q-axis current samples during the step (A).
        vq_data: Q-axis voltage samples during the step (V).
        t_data: Timestamps for each sample (s).
        step_magnitude: The target current for this step (A) -- used
            for reference only, not for calculation.

    Returns:
        Dict with keys ``R``, ``L``, ``tau``, ``step_magnitude``,
        ``iq_ss``, ``vq_ss``, ``n_samples``, or None if extraction fails.
    """
    n = len(iq_data)
    if n < 20:
        return None

    # --- Steady-state R from last 30 % of samples ---
    ss_count = max(1, n // 3)
    vq_ss = _median(vq_data[-ss_count:])
    iq_ss = _median(iq_data[-ss_count:])

    if abs(iq_ss) < 0.05:
        return None  # current didn't rise detectably

    R = vq_ss / iq_ss

    # --- L via integral method over first 10 samples ---
    n_int = min(10, n)
    integral_vq = 0.0
    integral_iq = 0.0
    for i in range(1, n_int):
        dt = t_data[i] - t_data[i - 1]
        integral_vq += (vq_data[i - 1] + vq_data[i]) / 2.0 * dt
        integral_iq += (iq_data[i - 1] + iq_data[i]) / 2.0 * dt

    diq = iq_data[n_int - 1] - iq_data[0]

    if abs(diq) < 0.02:
        L = R * REF_TAU  # fallback -- too little current change
    else:
        L = (integral_vq - R * integral_iq) / diq
        if L < 1e-6 or L > 0.01:
            L = R * REF_TAU  # fallback -- unphysical inductance

    tau = L / R if R != 0.0 else REF_TAU

    return {
        "R": R,
        "L": L,
        "tau": tau,
        "step_magnitude": step_magnitude,
        "iq_ss": iq_ss,
        "vq_ss": vq_ss,
        "n_samples": n,
    }


# ---------------------------------------------------------------------------
# High-level test runner
# ---------------------------------------------------------------------------

def run_step_test(
    iface: Any,
    amplitude: float = 1.0,
    n_samples: int = 30,
    n_steps: int = 3,
) -> Dict[str, Any]:
    """Run a step-response test sequence and extract R/L parameters.

    Procedure:
      1. Flush the serial input buffer and ensure the motor starts from
         zero torque (``T=0.0`` + 50 ms settle time).
      2. Execute *n_steps* alternating-sign steps
         (positive, negative, positive, ...).
      3. For each step, send ``T=<target>``, wait for the MCU status
         flag, then collect *n_samples* telemetry frames.
      4. Extract R/L from each step via :func:`_extract_rl_from_step`.
      5. Send ``T=0.0`` to return current to zero.
      6. Filter step results through the sanity bounds obtained from
         :func:`pi_calc.get_sanity_bounds`.
      7. Compute the median R, L, and tau across the valid (in-bounds)
         step results.

    Args:
        iface: An open :class:`SerialInterface` instance providing
            ``send_cmd``, ``wait_for_flag``, ``read_telemetry``, and
            ``flush_input`` methods.
        amplitude: Torque amplitude for each step in amps.  Default 1.0.
        n_samples: Number of telemetry frames to collect per step.
            Default 30.
        n_steps: Number of steps to perform.  Default 3.

    Returns:
        Dict with keys:

        - ``R`` (float or None) -- median resistance across valid steps.
        - ``L`` (float or None) -- median inductance across valid steps.
        - ``tau`` (float or None) -- median time constant (L / R).
        - ``samples`` (list of dict) -- per-step extraction results from
          :func:`_extract_rl_from_step` (unfiltered).
        - ``valid`` (bool) -- True if at least one step passed sanity
          bounds.
        - ``error`` (str, optional) -- present when ``valid`` is False.
    """
    results: List[Dict[str, Any]] = []

    # Build alternating step targets: +, -, +, ...
    step_targets: List[float] = []
    for i in range(n_steps):
        step_targets.append(amplitude if i % 2 == 0 else -amplitude)

    # --- 1. Start from zero ---
    iface.flush_input()
    iface.send_cmd("T=0.0")
    iface.wait_for_flag(timeout_s=2.0)
    time.sleep(0.05)  # 50 ms settle

    # --- 2. Execute steps ---
    for target in step_targets:
        iq_meas: List[float] = []
        vq_cmd: List[float] = []
        t_stamps: List[float] = []

        iface.send_cmd(f"T={target}")
        iface.wait_for_flag(timeout_s=2.0)

        start = time.monotonic()
        collected = 0
        while collected < n_samples:
            frame = iface.read_telemetry(timeout_s=0.5)
            if frame is not None:
                iq_meas.append(frame["iq_meas"])
                vq_cmd.append(frame["vq_cmd"])
                t_stamps.append(time.monotonic() - start)
                collected += 1

        step_result = _extract_rl_from_step(iq_meas, vq_cmd, t_stamps, target)
        if step_result is not None:
            results.append(step_result)

    # --- 3. Return to zero ---
    iface.send_cmd("T=0.0")
    iface.wait_for_flag(timeout_s=2.0)

    # --- 4. Filter through sanity bounds ---
    r_min, r_max, l_min, l_max = get_sanity_bounds()
    filtered: List[Dict[str, Any]] = []
    for r in results:
        if r_min <= r["R"] <= r_max and l_min <= r["L"] <= l_max:
            filtered.append(r)

    if not filtered:
        return {
            "R": None,
            "L": None,
            "tau": None,
            "samples": [dict(r) for r in results],
            "valid": False,
            "error": "All steps failed sanity bounds",
        }

    # --- 5. Median across valid steps ---
    R_vals = [r["R"] for r in filtered]
    L_vals = [r["L"] for r in filtered]
    tau_vals = [r["tau"] for r in filtered]

    return {
        "R": _median(R_vals),
        "L": _median(L_vals),
        "tau": _median(tau_vals),
        "samples": [dict(r) for r in results],
        "valid": True,
    }
