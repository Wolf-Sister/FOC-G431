"""
PRBS Generator & RLS System Identification for FOC current-loop auto-tuning.

Provides:
- PRBSGenerator: Maximum-length sequence (MLS) generator using n-bit LFSR
- RLSEstimator: Recursive Least Squares for ARX model identification
- _arx_to_rl: Convert ARX discrete params to continuous RL params
- prbs_identify: Orchestrate PRBS injection + RLS identification pipeline

Uses ONLY stdlib -- no numpy. All matrix operations with plain Python lists.
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, List

from .serial_iface import SerialInterface


# ---------------------------------------------------------------------------
# PRBS Generator
# ---------------------------------------------------------------------------

class PRBSGenerator:
    """Maximum-length sequence (MLS) generator using n-bit LFSR.

    Generates a pseudo-random binary sequence by XORing tap bits
    and feeding back.  The LFSR is initialised to all-ones to avoid
    the all-zeros deadlock state.

    For n_bits=7 the MLS period is 2**7 - 1 = 127 samples with
    64 ones and 63 zeros.

    Attributes:
        n_bits: Number of bits in the LFSR (2-12 supported).
        mask: Bitmask for the LFSR register.
        state: Current LFSR state.
    """

    _TAPS: Dict[int, List[int]] = {
        2: [0, 1], 3: [0, 2], 4: [0, 3], 5: [1, 4],
        6: [0, 5], 7: [0, 6], 8: [1, 2, 3, 7],
        9: [3, 8], 10: [2, 9], 11: [1, 10], 12: [0, 3, 5, 11],
    }

    def __init__(self, n_bits: int = 7) -> None:
        if n_bits not in self._TAPS:
            raise ValueError(
                f"n_bits must be in {sorted(self._TAPS.keys())}, got {n_bits}"
            )
        self.n_bits: int = n_bits
        self.mask: int = (1 << n_bits) - 1
        self.state: int = self.mask  # all-ones initialisation
        self._taps: List[int] = self._TAPS[n_bits]
        self._seq_count: int = 0  # count of bits generated

    def next(self) -> int:
        """Return the next PRBS bit (0 or 1)."""
        # XOR all tapped bits
        feedback = 0
        for tap in self._taps:
            feedback ^= (self.state >> tap) & 1

        # Shift left, OR feedback into LSB, mask
        self.state = ((self.state << 1) | feedback) & self.mask

        # Guard against all-zeros deadlock
        if self.state == 0:
            self.state = 1

        self._seq_count += 1
        return feedback

    def sequence(self, length: int) -> List[int]:
        """Generate a list of `length` PRBS bits."""
        return [self.next() for _ in range(length)]


# ---------------------------------------------------------------------------
# RLS Estimator
# ---------------------------------------------------------------------------

class RLSEstimator:
    """Recursive Least Squares estimator for ARX model identification.

    Fits the model::

        Iq[k] = a1 * Iq[k-1] + b1 * Vq[k-1]

    theta = [a1, b1] initialised to zeros.
    P = I * 1e6 (large initial covariance = high uncertainty).

    Args:
        n_params: Number of parameters to estimate (default 2).
        lambda_val: Forgetting factor (0 < lambda <= 1, default 0.98).
    """

    def __init__(self, n_params: int = 2, lambda_val: float = 0.98) -> None:
        if not (0 < lambda_val <= 1.0):
            raise ValueError(f"lambda_val must be in (0, 1], got {lambda_val}")
        self._n_params: int = n_params
        self._lambda: float = lambda_val
        self._theta: List[float] = [0.0] * n_params
        self._P: List[List[float]] = [
            [1e6 if i == j else 0.0 for j in range(n_params)]
            for i in range(n_params)
        ]
        self._iterations: int = 0
        # History of mean absolute parameter change per update
        self._param_changes: List[float] = []

    # -- properties ----------------------------------------------------------

    @property
    def theta(self) -> List[float]:
        """Current parameter estimate [a1, b1]."""
        return list(self._theta)

    @property
    def iterations(self) -> int:
        """Number of RLS updates performed."""
        return self._iterations

    # -- core update ---------------------------------------------------------

    def update(self, y: float, phi: List[float]) -> List[float]:
        """Perform one RLS update step.

        Args:
            y: Measured output (Iq[k]).
            phi: Regressor vector [Iq[k-1], Vq[k-1]].

        Returns:
            Updated theta vector [a1, b1].
        """
        n = self._n_params

        # Prediction error:  e = y - phi^T * theta
        y_pred = _dot(phi, self._theta)
        e = y - y_pred

        # P_phi = P * phi   (2x2 * 2x1 = 2x1)
        P_phi = _mat_vec_mul(self._P, phi)

        # Denominator:  lambda + phi^T * P * phi
        denom = self._lambda + _dot(phi, P_phi)

        # Kalman gain:  K = P * phi / denom
        K = [v / denom for v in P_phi]

        # Save old theta for convergence tracking
        old_theta = list(self._theta)

        # theta = theta + K * e
        for i in range(n):
            self._theta[i] = old_theta[i] + K[i] * e

        # phi^T * P  (row vector, 1x2)
        #    = [ sum_k(phi[k] * P[k][0]),  sum_k(phi[k] * P[k][1]) ]
        phiT_P = [sum(phi[k] * self._P[k][j] for k in range(n)) for j in range(n)]

        # P = (P - K * (phi^T * P)) / lambda
        for i in range(n):
            for j in range(n):
                self._P[i][j] = (self._P[i][j] - K[i] * phiT_P[j]) / self._lambda

        self._iterations += 1

        # Track mean absolute parameter change
        change = sum(abs(self._theta[i] - old_theta[i]) for i in range(n)) / n
        self._param_changes.append(change)

        return list(self._theta)

    # -- convergence ---------------------------------------------------------

    def has_converged(self, window: int = 10, tol: float = 0.01) -> bool:
        """Check if the estimator has converged.

        Returns True if the mean absolute parameter change over the
        last *window* updates is below *tol*.

        Args:
            window: Number of recent updates to consider.
            tol: Convergence tolerance on mean parameter change.

        Returns:
            True if converged, False otherwise.
        """
        if self._iterations < window:
            return False
        recent = self._param_changes[-window:]
        return sum(recent) / len(recent) < tol


# ---------------------------------------------------------------------------
# Internal matrix helpers (stdlib only -- no numpy)
# ---------------------------------------------------------------------------

def _dot(a: List[float], b: List[float]) -> float:
    """Dot product of two equal-length vectors."""
    return sum(a[i] * b[i] for i in range(len(a)))


def _mat_vec_mul(A: List[List[float]], x: List[float]) -> List[float]:
    """Multiply matrix A (m x n) by vector x (n x 1) -> vector (m x 1)."""
    return [_dot(row, x) for row in A]


# ---------------------------------------------------------------------------
# ARX -> RL conversion
# ---------------------------------------------------------------------------

def _arx_to_rl(a1: float, b1: float, Ts: float) -> tuple[float, float]:
    """Convert discrete ARX parameters to continuous RL parameters.

    Uses the ZOH inverse of the discrete RL model::

        R  = (1 - a1) / b1
        L  = -R * Ts / ln(a1)

    Args:
        a1: Autoregressive coefficient (must be in (0, 1)).
        b1: Input coefficient (must be > 0).
        Ts: Sample time in seconds.

    Returns:
        Tuple of (R, L) in ohms and henries.

    Raises:
        ValueError: If parameters are outside valid ranges
            (b1 <= 0, a1 <= 0, or a1 >= 1.0).
    """
    if b1 <= 0:
        raise ValueError(f"b1 must be positive, got {b1}")
    if a1 <= 0 or a1 >= 1.0:
        raise ValueError(
            f"a1 must be strictly between 0 and 1, got {a1}"
        )

    R = (1.0 - a1) / b1
    L = -R * Ts / math.log(a1)
    return (R, L)


# ---------------------------------------------------------------------------
# PRBS Identification Pipeline
# ---------------------------------------------------------------------------

def prbs_identify(
    iface: SerialInterface,
    amplitude: float = 1.0,
    prbs_length: int = 127,
    clock_samples: int = 5,
    Ts: float = 0.01,
) -> Dict[str, Any]:
    """Run PRBS identification: inject PRBS current commands, collect data, fit RLS.

    The pipeline:
    1. Generate *prbs_length* PRBS bits.
    2. For each bit, send ``T={amplitude}`` (bit=1) or ``T={-amplitude}``
       (bit=0) to the MCU, wait for the status flag, then collect
       *clock_samples* telemetry frames.
    3. Send ``T=0.0`` to return current to zero.
    4. Fit an ARX model ``Iq[k] = a1*Iq[k-1] + b1*Vq[k-1]`` via RLS
       (skipping k=0 because the regressor needs k-1 data).
    5. Convert (a1, b1) to (R, L) via ``_arx_to_rl``.

    Args:
        iface: Connected :class:`SerialInterface` instance.
        amplitude: PRBS amplitude in amps (used as +/-).
        prbs_length: Number of PRBS bits to inject (default 127).
        clock_samples: Telemetry frames per PRBS bit (default 5).
        Ts: Sample time in seconds (default 0.01 for 100 Hz telemetry).

    Returns:
        Dict with keys:
        - ``R``, ``L`` (float or None on failure)
        - ``a1``, ``b1`` (float)
        - ``converged`` (bool)
        - ``iterations`` (int)
        - ``iq_data``, ``vq_data`` (list of float)
        - ``error`` (str, present only on failure)
    """
    result: Dict[str, Any] = {
        "R": None,
        "L": None,
        "a1": 0.0,
        "b1": 0.0,
        "converged": False,
        "iterations": 0,
        "iq_data": [],
        "vq_data": [],
    }

    # Generate PRBS sequence
    prbs = PRBSGenerator(n_bits=7)
    bits = prbs.sequence(prbs_length)

    iq_data: List[float] = []
    vq_data: List[float] = []

    def _zero_current() -> None:
        """Best-effort: send T=0 and wait briefly."""
        try:
            iface.send_cmd("T=0.0")
            iface.wait_for_flag(timeout_s=1.0)
        except Exception:
            pass

    try:
        # -- Phase 1: inject PRBS commands and collect telemetry ---------
        for bit in bits:
            cmd_value = amplitude if bit == 1 else -amplitude
            iface.send_cmd(f"T={cmd_value}")

            # Wait for MCU to acknowledge the command
            if not iface.wait_for_flag(timeout_s=2.0):
                _zero_current()
                result["error"] = "Timeout waiting for status flag after command"
                return result

            # Collect clock_samples telemetry frames for this bit
            for _ in range(clock_samples):
                telemetry = iface.read_telemetry(timeout_s=0.5)
                if telemetry is None:
                    continue  # skip missed frames
                iq_data.append(telemetry["iq_meas"])
                vq_data.append(telemetry["vq_cmd"])

        # Return current to zero
        _zero_current()

        # -- Phase 2: sanity checks --------------------------------------
        # Need at least 50 RLS iterations (51 data points, since k=0 skipped)
        if len(iq_data) < 51:
            result["error"] = (
                f"Insufficient data: got {len(iq_data)} points, need at least 51"
            )
            return result

        # Store raw data
        result["iq_data"] = iq_data
        result["vq_data"] = vq_data

        # -- Phase 3: RLS estimation -------------------------------------
        rls = RLSEstimator(n_params=2, lambda_val=0.98)
        for k in range(1, len(iq_data)):
            y = iq_data[k]                     # Iq[k]
            phi = [iq_data[k - 1], vq_data[k - 1]]  # [Iq[k-1], Vq[k-1]]
            rls.update(y, phi)

        a1, b1 = rls.theta
        result["a1"] = a1
        result["b1"] = b1
        result["iterations"] = rls.iterations
        result["converged"] = rls.has_converged()

        # -- Phase 4: convert to R, L ------------------------------------
        R, L = _arx_to_rl(a1, b1, Ts)
        result["R"] = R
        result["L"] = L

        return result

    except ValueError as exc:
        # _arx_to_rl raises ValueError on bad params
        _zero_current()
        result["error"] = str(exc)
        return result
    except Exception as exc:
        _zero_current()
        result["error"] = str(exc)
        return result
