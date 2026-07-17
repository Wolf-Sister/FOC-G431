"""PI controller gain calculation using pole-zero cancellation.

Computes PI gains from motor RL parameters using the pole-zero cancellation
technique: PI zero at plant pole so the closed-loop transfer function becomes
a first-order low-pass filter with bandwidth omega_c.

    Kp = omega_c * L
    Ki = omega_c * R
    G_cl(s) = omega_c / (s + omega_c)
"""

from typing import List, Tuple

# ---------------------------------------------------------------------------
# Reference constants (derived from a reference PI set)
# ---------------------------------------------------------------------------
DEFAULT_OMEGA_C: float = 3000.0  # rad/s ≈ 477 Hz
DEFAULT_KP: float = 1.485
DEFAULT_KI: float = 371.25

REF_R: float = DEFAULT_KI / DEFAULT_OMEGA_C  # ≈ 0.12375 Ω
REF_L: float = DEFAULT_KP / DEFAULT_OMEGA_C  # ≈ 0.000495 H = 495 µH
REF_TAU: float = DEFAULT_KP / DEFAULT_KI  # ≈ 0.004 s = 4 ms


# ---------------------------------------------------------------------------
# Shared utility
# ---------------------------------------------------------------------------

def _median(vals: List[float]) -> float:
    """Return the median of a list of numbers.

    Sorts the list and picks the middle element (odd length) or the
    average of the two middle elements (even length).

    Args:
        vals: List of numeric values.

    Returns:
        Median value.  Returns 0.0 for an empty list.
    """
    if not vals:
        return 0.0
    s = sorted(vals)
    m = len(s) // 2
    return s[m] if len(s) % 2 == 1 else (s[m - 1] + s[m]) / 2.0


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def calculate_pi(
    R: float,
    L: float,
    omega_c: float = DEFAULT_OMEGA_C,
) -> Tuple[float, float]:
    """Compute PI gains from motor resistance and inductance.

    Uses pole-zero cancellation:
        Kp = omega_c * L
        Ki = omega_c * R

    This places the PI zero at the plant pole (-Ki/Kp = -R/L), making the
    closed-loop transfer function a first-order low-pass:

        G_cl(s) = omega_c / (s + omega_c)

    Args:
        R: Motor phase resistance in ohms.
        L: Motor phase inductance in henries.
        omega_c: Target crossover frequency in rad/s (default 3000 rad/s).

    Returns:
        Tuple of (Kp, Ki) — proportional and integral gains.
    """
    kp: float = omega_c * L
    ki: float = omega_c * R
    return (kp, ki)


def implied_rl_from_pi(
    Kp: float,
    Ki: float,
) -> Tuple[float, float]:
    """Back-compute implied R/L from existing PI gains.

    Because the crossover frequency omega_c is unknown when only the gains
    are available, R is normalised to 1.0.  The L/R ratio is Kp/Ki, which
    can be used for sanity checks against the known motor time constant.

    This function is available for **external sanity checks** (e.g. in
    notebooks or calibration dashboards) to validate whether uploaded gains
    match the expected motor time constant.

    Args:
        Kp: Proportional gain.
        Ki: Integral gain.

    Returns:
        Tuple of (R_implied, L_implied).  R is always 1.0; L is Kp/Ki
        (falling back to REF_TAU when Ki is zero).
    """
    if Ki == 0.0:
        return (1.0, REF_TAU)
    return (1.0, Kp / Ki)


def get_sanity_bounds() -> Tuple[float, float, float, float]:
    """Return recommended sanity bounds for motor R and L.

    The bounds are centred on the reference values REF_R and REF_L:

        R_min = REF_R * 0.2
        R_max = REF_R * 5.0
        L_min = REF_L * 0.2
        L_max = REF_L * 5.0

    Returns:
        Tuple of (R_min, R_max, L_min, L_max).
    """
    r_min: float = REF_R * 0.2
    r_max: float = REF_R * 5.0
    l_min: float = REF_L * 0.2
    l_max: float = REF_L * 5.0
    return (r_min, r_max, l_min, l_max)
