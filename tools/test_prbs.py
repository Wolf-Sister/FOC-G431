#!/usr/bin/env python3
"""Unit tests for the PRBS / RLS system identification module (stdlib only)."""

import sys
import os
import math

# Allow import from the project root (parent of tools/)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from tools.prbs_id import PRBSGenerator, RLSEstimator, _arx_to_rl


# ---------------------------------------------------------------------------
# Test 1: PRBS sequence length and balance
# ---------------------------------------------------------------------------

def test_prbs_sequence_length() -> None:
    """PRBS(7) produces 127 pts, 55-72 ones, deterministic on repeat."""
    # First generation
    prbs1 = PRBSGenerator(n_bits=7)
    seq1 = prbs1.sequence(127)

    assert len(seq1) == 127, (
        f"Expected 127 bits, got {len(seq1)}"
    )

    ones = sum(seq1)
    assert 55 <= ones <= 72, (
        f"Ones count {ones} outside [55, 72] (expected ~64)"
    )

    # Second generation -- must match exactly
    prbs2 = PRBSGenerator(n_bits=7)
    seq2 = prbs2.sequence(127)

    assert seq1 == seq2, "PRBS(7) sequence is not deterministic"

    print(f"  PASS test_prbs_sequence_length: length={len(seq1)}, "
          f"ones={ones} (~64 balanced), deterministic")


# ---------------------------------------------------------------------------
# Test 2: RLS convergence on synthetic RL data
# ---------------------------------------------------------------------------

def test_rls_convergence() -> None:
    """RLS converges with <2 % a1/b1 error and <5 % R/L error."""
    R_true = 0.12
    L_true = 0.0005
    Ts = 0.01
    N = 200
    vq_amp = 2.0

    # True discrete-time ARX parameters (from ZOH discretisation)
    a1_true = math.exp(-R_true * Ts / L_true)
    b1_true = (1.0 - a1_true) / R_true

    # Generate PRBS excitation
    prbs = PRBSGenerator(n_bits=7)
    bits = prbs.sequence(N)
    Vq = [vq_amp if b == 1 else -vq_amp for b in bits]

    # Simulate the ARX plant: Iq[k] = a1 * Iq[k-1] + b1 * Vq[k-1]
    Iq = [0.0] * N
    for k in range(1, N):
        Iq[k] = a1_true * Iq[k-1] + b1_true * Vq[k-1]

    # RLS estimation
    rls = RLSEstimator(n_params=2, lambda_val=0.98)
    for k in range(1, N):
        y = Iq[k]
        phi = [Iq[k - 1], Vq[k - 1]]
        rls.update(y, phi)

    a1_est, b1_est = rls.theta

    # -- Assertions ---------------------------------------------------------

    # 1. RLS must report convergence
    assert rls.has_converged(), "RLS did not converge"

    # 2. Relative errors
    a1_err = abs(a1_est - a1_true) / a1_true
    b1_err = abs(b1_est - b1_true) / b1_true

    assert a1_err < 0.02, (
        f"a1 error {a1_err*100:.2f}% >= 2% (est={a1_est:.6f}, true={a1_true:.6f})"
    )
    assert b1_err < 0.02, (
        f"b1 error {b1_err*100:.2f}% >= 2% (est={b1_est:.6f}, true={b1_true:.6f})"
    )

    # 3. Convert to physical RL and check
    R_est, L_est = _arx_to_rl(a1_est, b1_est, Ts)

    R_err = abs(R_est - R_true) / R_true
    L_err = abs(L_est - L_true) / L_true

    assert R_err < 0.05, (
        f"R error {R_err*100:.2f}% >= 5% (est={R_est:.6f}, true={R_true})"
    )
    assert L_err < 0.05, (
        f"L error {L_err*100:.2f}% >= 5% (est={L_est:.6f}, true={L_true})"
    )

    # -- Summary ------------------------------------------------------------
    print(f"  PASS test_rls_convergence:")
    print(f"       a1={a1_est:.6f} (true={a1_true:.6f}, err={a1_err*100:.2f}%)")
    print(f"       b1={b1_est:.6f} (true={b1_true:.6f}, err={b1_err*100:.2f}%)")
    print(f"       R={R_est:.6f} (true={R_true}, err={R_err*100:.2f}%)")
    print(f"       L={L_est:.6f} (true={L_true}, err={L_err*100:.2f}%)")
    print(f"       converged={rls.has_converged()}, iterations={rls.iterations}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print("test_prbs:")
    test_prbs_sequence_length()
    test_rls_convergence()
    print("All 2 PASS")
