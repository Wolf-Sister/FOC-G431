"""
Step-response verification with A/B comparison for PI controller auto-tuning.

Provides:
- StepMetrics: Container for step-response metrics (rise time, overshoot, etc.)
- _measure_step: Internal function to run a step test and compute metrics
- run_verification: Public function to compare original vs new gains and decide rollback

Uses ONLY stdlib -- no numpy. All metrics computed manually.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from .pi_calc import _median
from .serial_iface import SerialInterface


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_TELEMETRY_INTERVAL_S: float = 0.01       # 100 Hz -> 10 ms between samples
_MIN_VALID_SAMPLES: int = 10
_DEFAULT_SETTLE_SAMPLES: int = 15
_DEFAULT_TOTAL_SAMPLES: int = 30
_NEAR_ZERO_THRESHOLD: float = 1e-9        # Iq_ss below this is treated as zero


# ---------------------------------------------------------------------------
# StepMetrics
# ---------------------------------------------------------------------------

@dataclass
class StepMetrics:
    """Container for step-response metrics.

    Attributes:
        rise_time_ms: 10 % -> 90 % rise time in milliseconds
            (coarse: 10 ms steps).
        overshoot_pct: Overshoot as a percentage of the steady-state Iq value.
        settling_time_ms: Time of the last sample outside the 5 % band,
            in milliseconds (coarse: 10 ms steps).
        steady_state_error_a: Absolute error between the setpoint and the
            steady-state Iq.
        iq_data: Raw Iq measurement data (useful for debugging / plotting).
        vq_data: Raw Vq command data (useful for debugging / plotting).
    """
    rise_time_ms: float = 0.0
    overshoot_pct: float = 0.0
    settling_time_ms: float = 0.0
    steady_state_error_a: float = 0.0
    iq_data: List[float] = field(default_factory=list)
    vq_data: List[float] = field(default_factory=list)

    def to_dict(self) -> Dict[str, float]:
        """Return a dict of metric values only (no raw data).

        Returns:
            Dict with keys: ``rise_time_ms``, ``overshoot_pct``,
            ``settling_time_ms``, ``steady_state_error_a``.
        """
        return {
            "rise_time_ms": self.rise_time_ms,
            "overshoot_pct": self.overshoot_pct,
            "settling_time_ms": self.settling_time_ms,
            "steady_state_error_a": self.steady_state_error_a,
        }


# ---------------------------------------------------------------------------
# Internal: _measure_step
# ---------------------------------------------------------------------------

def _measure_step(
    iface: SerialInterface,
    setpoint: float,
    settle_samples: int = _DEFAULT_SETTLE_SAMPLES,
    total_samples: int = _DEFAULT_TOTAL_SAMPLES,
) -> StepMetrics:
    """Run a single step test and compute response metrics.

    Sends a torque command ``T=<setpoint>``, waits for the MCU status flag,
    collects *total_samples* telemetry frames, then returns current to zero
    (``T=0.0``).

    Metrics are derived from the collected Iq data:

    - **Steady-state Iq**: median of the last *settle_samples* values.
    - **Rise time**: interval (in 10 ms steps) between the first sample
      reaching 10 % of Iq_ss and the first sample reaching 90 % of Iq_ss.
    - **Overshoot**: ``(max(Iq) - Iq_ss) / Iq_ss * 100``, clamped to >= 0 %.
    - **Settling time**: index of the last sample outside the 5 % band
      (``Iq_ss +/- 5 %``) multiplied by 10 ms.
    - **Steady-state error**: ``|setpoint - Iq_ss|``.

    If fewer than ``_MIN_VALID_SAMPLES`` valid telemetry frames are received,
    returns a ``StepMetrics`` instance with all-zero metrics and empty data
    lists.

    Args:
        iface: Connected :class:`SerialInterface` instance.
        setpoint: Torque setpoint in amps (sent as ``T=<setpoint>``).
        settle_samples: Number of trailing samples used for steady-state
            median estimation (default 15).
        total_samples: Number of telemetry frames to attempt collecting
            (default 30).

    Returns:
        :class:`StepMetrics` with computed metrics.
    """
    # -- Step 1: flush, send setpoint, wait for acknowledgement --------------
    iface.flush_input()
    iface.send_cmd(f"T={setpoint}")
    if not iface.wait_for_flag(timeout_s=2.0):
        # Command was not acknowledged; return current to zero and bail out
        iface.send_cmd("T=0.0")
        return StepMetrics()

    # -- Step 2: collect telemetry frames ------------------------------------
    iq_data: List[float] = []
    vq_data: List[float] = []

    for _ in range(total_samples):
        telemetry = iface.read_telemetry(timeout_s=0.5)
        if telemetry is None:
            continue
        iq_data.append(telemetry["iq_meas"])
        vq_data.append(telemetry["vq_cmd"])

    # -- Step 3: return current to zero --------------------------------------
    iface.send_cmd("T=0.0")

    # -- Step 4: minimum-samples guard ---------------------------------------
    if len(iq_data) < _MIN_VALID_SAMPLES:
        return StepMetrics()

    # -- Step 5: compute steady-state Iq (median of trailing samples) --------
    settle_start = max(0, len(iq_data) - settle_samples)
    settle_iq = iq_data[settle_start:]
    iq_ss = _median(settle_iq)

    # Guard against near-zero steady-state (avoid division by zero)
    if abs(iq_ss) < _NEAR_ZERO_THRESHOLD:
        return StepMetrics(
            steady_state_error_a=abs(setpoint),
            iq_data=iq_data,
            vq_data=vq_data,
        )

    abs_iq_ss = abs(iq_ss)

    # -- Step 6: rise time ---------------------------------------------------
    # Find the index of the first sample reaching 10 % of steady-state.
    idx_10: Optional[int] = None
    for i, val in enumerate(iq_data):
        if val >= 0.1 * abs_iq_ss:
            idx_10 = i
            break

    # Find the index of the first sample reaching 90 % (starting from idx_10).
    idx_90: Optional[int] = None
    if idx_10 is not None:
        for i in range(idx_10, len(iq_data)):
            if iq_data[i] >= 0.9 * abs_iq_ss:
                idx_90 = i
                break

    # Fallback defaults if thresholds were never crossed
    if idx_10 is None:
        idx_10 = 0
    if idx_90 is None:
        idx_90 = len(iq_data) - 1

    rise_time_ms = (idx_90 - idx_10) * _TELEMETRY_INTERVAL_S * 1000.0
    if rise_time_ms < 0.0:
        rise_time_ms = 0.0

    # -- Step 7: overshoot ---------------------------------------------------
    iq_max = max(iq_data)
    overshoot_pct = max(0.0, (iq_max - iq_ss) / abs_iq_ss * 100.0)

    # -- Step 8: settling time -----------------------------------------------
    # Last sample index that falls outside the 5 % band around Iq_ss.
    band_half = 0.05 * abs_iq_ss
    settling_idx = 0
    for i, val in enumerate(iq_data):
        if abs(val - iq_ss) > band_half:
            settling_idx = i
    settling_time_ms = settling_idx * _TELEMETRY_INTERVAL_S * 1000.0

    # -- Step 9: steady-state error ------------------------------------------
    steady_state_error_a = abs(setpoint - iq_ss)

    return StepMetrics(
        rise_time_ms=rise_time_ms,
        overshoot_pct=overshoot_pct,
        settling_time_ms=settling_time_ms,
        steady_state_error_a=steady_state_error_a,
        iq_data=iq_data,
        vq_data=vq_data,
    )


# ---------------------------------------------------------------------------
# Public: run_verification
# ---------------------------------------------------------------------------

def run_verification(
    iface: SerialInterface,
    kp_new: float,
    ki_new: float,
    kp_orig: float,
    ki_orig: float,
    amplitude: float = 1.0,
) -> Dict[str, Any]:
    """A/B compare step response under original vs new PI gains.

    Pipeline:
    1. Set original gains (``P=<kp_orig>,I=<ki_orig>``), wait 50 ms.
    2. Measure step response via :func:`_measure_step` at the given amplitude.
    3. Set new gains (``P=<kp_new>,I=<ki_new>``), wait 50 ms.
    4. Measure step response via :func:`_measure_step` at the same amplitude.
    5. Decision logic:
       - Roll back to original gains **if** new overshoot > 50 % **or**
         new rise time > 2x original rise time (provided original rise time
         is measurable — greater than 0.1 ms).
       - If the new-gain step measurement itself failed (< 10 samples), also
         roll back as a safety measure.
       - ``rollback`` is ``True`` when a rollback was performed.
       - ``passed`` is ``True`` when the new gains are kept (no rollback).
    6. Return a dict with both metrics dicts, rollback status, and pass/fail.

    Args:
        iface: Connected :class:`SerialInterface` instance.
        kp_new: Proposed proportional gain.
        ki_new: Proposed integral gain.
        kp_orig: Original (baseline) proportional gain.
        ki_orig: Original (baseline) integral gain.
        amplitude: Step amplitude in amps (default 1.0).

    Returns:
        Dict with keys:
        - ``"original"``: Original-gain :class:`StepMetrics` via ``to_dict()``.
        - ``"new"``: New-gain :class:`StepMetrics` via ``to_dict()``.
        - ``"rollback"`` (bool): ``True`` if gains were rolled back.
        - ``"passed"`` (bool): ``True`` if new gains were accepted.
    """
    # -- Phase 1: measure with original gains ---------------------------------
    iface.send_cmd(f"P={kp_orig},I={ki_orig}")
    time.sleep(0.05)  # allow 50 ms for the gain change to take effect

    orig_metrics = _measure_step(iface, amplitude)

    # -- Phase 2: measure with new gains --------------------------------------
    iface.send_cmd(f"P={kp_new},I={ki_new}")
    time.sleep(0.05)  # allow 50 ms for the gain change to take effect

    new_metrics = _measure_step(iface, amplitude)

    # -- Phase 3: decision logic ----------------------------------------------
    rollback: bool = False

    if len(new_metrics.iq_data) < _MIN_VALID_SAMPLES:
        # New-gain step measurement failed; roll back for safety.
        rollback = True
    else:
        # Overshoot threshold check
        if new_metrics.overshoot_pct > 50.0:
            rollback = True
        # Rise-time degradation check (only when original rise time is
        # measurable, i.e. > 0.1 ms)
        elif (
            orig_metrics.rise_time_ms > 0.1
            and new_metrics.rise_time_ms > 2.0 * orig_metrics.rise_time_ms
        ):
            rollback = True

    # -- Phase 4: apply rollback if needed ------------------------------------
    if rollback:
        iface.send_cmd(f"P={kp_orig},I={ki_orig}")

    return {
        "original": orig_metrics.to_dict(),
        "new": new_metrics.to_dict(),
        "rollback": rollback,
        "passed": not rollback,
    }
