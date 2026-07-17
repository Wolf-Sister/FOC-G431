#!/usr/bin/env python3
"""Unit tests for the pi_calc module (stdlib only, no hardware)."""

import sys
import os

# Allow import from the project root (parent of tools/)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from tools.pi_calc import (
    calculate_pi,
    implied_rl_from_pi,
    get_sanity_bounds,
    REF_R,
    REF_L,
)


# ---------------------------------------------------------------------------
# Test 1: calculate_pi
# ---------------------------------------------------------------------------

def test_calculate_pi() -> None:
    """calculate_pi(R=0.125, L=0.0005, omega_c=3000) -> Kp~1.5, Ki~375.0."""
    Kp, Ki = calculate_pi(R=0.125, L=0.0005, omega_c=3000.0)

    # Kp = omega_c * L = 3000 * 0.0005 = 1.5
    # Ki = omega_c * R = 3000 * 0.125 = 375.0
    kp_err = abs(Kp - 1.5) / 1.5
    ki_err = abs(Ki - 375.0) / 375.0

    assert kp_err < 0.01, (
        f"Kp={Kp:.6f} error {kp_err*100:.3f}% >= 1%"
    )
    assert ki_err < 0.01, (
        f"Ki={Ki:.6f} error {ki_err*100:.3f}% >= 1%"
    )

    print(f"  PASS test_calculate_pi: Kp={Kp:.6f} (~1.5, err={kp_err*100:.3f}%), "
          f"Ki={Ki:.6f} (~375.0, err={ki_err*100:.3f}%)")


# ---------------------------------------------------------------------------
# Test 2: implied_rl_from_pi
# ---------------------------------------------------------------------------

def test_implied_rl() -> None:
    """implied_rl_from_pi(Kp=1.485, Ki=371.25) -> tau = L_imp ~ 0.004."""
    R_imp, L_imp = implied_rl_from_pi(Kp=1.485, Ki=371.25)

    # L_imp = Kp / Ki = 1.485 / 371.25 = 0.004
    tau_diff = abs(L_imp - 0.004)

    assert tau_diff < 0.0001, (
        f"L_imp={L_imp:.6f} differs from tau=0.004 by {tau_diff:.6f} "
        f"(limit 0.0001)"
    )

    print(f"  PASS test_implied_rl: L_imp={L_imp:.6f} (~0.004, "
          f"diff={tau_diff:.6f}), R_imp={R_imp:.1f}")


# ---------------------------------------------------------------------------
# Test 3: get_sanity_bounds
# ---------------------------------------------------------------------------

def test_sanity_bounds() -> None:
    """get_sanity_bounds() covers REF_R and REF_L in their ranges."""
    R_min, R_max, L_min, L_max = get_sanity_bounds()

    assert R_min <= REF_R <= R_max, (
        f"REF_R={REF_R} not in [{R_min}, {R_max}]"
    )
    assert L_min <= REF_L <= L_max, (
        f"REF_L={REF_L} not in [{L_min}, {L_max}]"
    )

    # Basic sanity of the bounds themselves
    assert 0 < R_min < R_max, f"Bad R bounds: [{R_min}, {R_max}]"
    assert 0 < L_min < L_max, f"Bad L bounds: [{L_min}, {L_max}]"

    print(f"  PASS test_sanity_bounds: REF_R={REF_R} in [{R_min:.6f}, {R_max:.6f}], "
          f"REF_L={REF_L} in [{L_min:.7f}, {L_max:.7f}]")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print("test_pi_calc:")
    test_calculate_pi()
    test_implied_rl()
    test_sanity_bounds()
    print("All 3 PASS")
