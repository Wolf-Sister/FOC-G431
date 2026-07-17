"""
CLI Entry & Orchestrator for FOC current-loop auto-tuning.

Orchestrates the full tuning pipeline:
  Phase 0: Connect to the MCU via serial, set torque mode
  Phase 1: Step-response test for R/L identification
  Phase 2: PRBS injection + RLS identification (optional)
  Phase 3: PI gain calculation and upload
  Phase 4: A/B verification with rollback on failure

Usage:
  python -m tools.auto_tuner [options]
  python tools/auto_tuner.py [options]
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Any, Dict, Optional, Tuple

if __package__ is None:
    # Running as a script — ensure tools/ is on sys.path as a package
    from pathlib import Path
    _tools_parent = str(Path(__file__).resolve().parent.parent)
    if _tools_parent not in sys.path:
        sys.path.insert(0, _tools_parent)
    from tools.pi_calc import (
        DEFAULT_KP,
        DEFAULT_KI,
        DEFAULT_OMEGA_C,
        REF_L,
        REF_R,
        REF_TAU,
        calculate_pi,
    )
    from tools.prbs_id import prbs_identify
    from tools.serial_iface import SerialInterface
    from tools.step_test import run_step_test
    from tools.validate import run_verification
else:
    # Running as a package module — use relative imports
    from .pi_calc import (
        DEFAULT_KP,
        DEFAULT_KI,
        DEFAULT_OMEGA_C,
        REF_L,
        REF_R,
        REF_TAU,
        calculate_pi,
    )
    from .prbs_id import prbs_identify
    from .serial_iface import SerialInterface
    from .step_test import run_step_test
    from .validate import run_verification


# ---------------------------------------------------------------------------
# Banner / header
# ---------------------------------------------------------------------------

def print_header(port: str) -> None:
    """Print a startup banner with port information."""
    print("=" * 60)
    print("  FOC Current-Loop Auto-Tuner")
    print("  STM32G431 + Python Toolchain")
    print("=" * 60)
    print(f"  Port: {port}")
    print()


# ---------------------------------------------------------------------------
# Phase 0: Connect
# ---------------------------------------------------------------------------

def phase_connect(port: Optional[str]) -> SerialInterface:
    """Connect to the MCU, set torque mode, and read initial state.

    Args:
        port: Serial port name (e.g. "COM3"), or None for auto-detect.

    Returns:
        Connected :class:`SerialInterface` instance.

    Raises:
        SystemExit: If no port is found or connection fails.
    """
    # Auto-detect port if not specified
    if port is None:
        available = SerialInterface.list_ports()
        if not available:
            print("ERROR: No serial ports detected.")
            sys.exit(1)
        port = available[0]
        print(f"Auto-detected port: {port}")

    print_header(port)
    print("[Phase 0] Connecting...")

    iface = SerialInterface(port=port)
    if not iface.connect():
        print(f"ERROR: Failed to connect on {port}. No telemetry received.")
        sys.exit(1)

    print(f"  Connected to {port}")

    # Set torque mode: M=0, D=0.0, T=0.0
    iface.send_cmd("M=0")
    iface.wait_for_flag(timeout_s=2.0)
    iface.send_cmd("D=0.0")
    iface.wait_for_flag(timeout_s=2.0)
    iface.send_cmd("T=0.0")
    iface.wait_for_flag(timeout_s=2.0)

    # Read telemetry to verify mode
    telemetry = iface.read_telemetry(timeout_s=1.0)
    if telemetry is None:
        print("ERROR: No telemetry received after connecting.")
        iface.disconnect()
        sys.exit(1)

    mode = int(telemetry.get("mode", 0))
    mode_name = {0: "TORQUE", 1: "SPEED", 2: "POSITION"}.get(mode, f"UNKNOWN({mode})")

    print(f"  Mode: {mode_name} | Existing PI: Kp={DEFAULT_KP}, Ki={DEFAULT_KI}")
    print()
    return iface


# ---------------------------------------------------------------------------
# Phase 1: Step Test
# ---------------------------------------------------------------------------

def phase_step_test(iface: SerialInterface, args: argparse.Namespace) -> Dict[str, Any]:
    """Run the step-response R/L identification test.

    Falls back to reference R/L values if the test fails or all samples
    are rejected by sanity bounds.

    Args:
        iface: Connected :class:`SerialInterface` instance.
        args: Parsed CLI arguments.

    Returns:
        Result dict from :func:`step_test.run_step_test`.
    """
    print("[Phase 1] Step-Response R/L Identification")
    print(f"  Amplitude: {args.step_amps} A, Samples/step: 30, Steps: 3")

    result = run_step_test(
        iface, amplitude=args.step_amps, n_samples=30, n_steps=3
    )

    # Print per-step details
    samples = result.get("samples", [])
    for i, s in enumerate(samples):
        print(
            f"  Step {i + 1}: R={s['R']:.4f} Ohm, L={s['L']:.6f} H, "
            f"tau={s['tau']:.6f} s, iq_ss={s['iq_ss']:.3f} A"
        )

    if not result.get("valid", False):
        print(f"  WARNING: {result.get('error', 'Step test failed')}")
        print(f"  Falling back to reference values: R={REF_R:.4f} Ohm, L={REF_L:.6f} H")
        result["R"] = REF_R
        result["L"] = REF_L
        result["tau"] = REF_TAU
    else:
        print(
            f"  Median R={result['R']:.4f} Ohm, L={result['L']:.6f} H, "
            f"tau={result['tau']:.6f} s"
        )

    print()
    return result


# ---------------------------------------------------------------------------
# Phase 2: PRBS Identification
# ---------------------------------------------------------------------------

def phase_prbs(
    iface: SerialInterface, args: argparse.Namespace, step_result: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """Run PRBS-based R/L identification and cross-check with step results.

    Args:
        iface: Connected :class:`SerialInterface` instance.
        args: Parsed CLI arguments.
        step_result: Result dict from Phase 1.

    Returns:
        PRBS result dict, or None if skipped or cross-check failed.
    """
    if args.no_prbs:
        print("[Phase 2] PRBS Identification — SKIPPED (--no-prbs)")
        print()
        return None

    print("[Phase 2] PRBS Identification")
    print(
        f"  Amplitude: {args.prbs_amps} A, Length: {args.prbs_length}, "
        f"Clock samples: 5"
    )

    result = prbs_identify(
        iface,
        amplitude=args.prbs_amps,
        prbs_length=args.prbs_length,
        clock_samples=5,
    )

    if "error" in result:
        print(f"  WARNING: PRBS identification failed: {result['error']}")
        print("  Falling back to step-test results.")
        print()
        return None

    print(
        f"  RLS converged: {result['converged']} "
        f"({result['iterations']} iterations)"
    )
    print(f"  a1={result['a1']:.4f}, b1={result['b1']:.6f}")
    print(f"  PRBS R={result['R']:.4f} Ohm, L={result['L']:.6f} H")

    # Cross-check with step result (±30%)
    step_r = step_result.get("R")
    step_l = step_result.get("L")
    if step_r is not None and step_l is not None:
        r_dev = abs(result["R"] - step_r) / abs(step_r) if abs(step_r) > 1e-9 else 999.0
        l_dev = abs(result["L"] - step_l) / abs(step_l) if abs(step_l) > 1e-9 else 999.0
        print(
            f"  Cross-check deviation: R={r_dev * 100:.1f}%, L={l_dev * 100:.1f}%"
        )
        if r_dev > 0.30 or l_dev > 0.30:
            print("  WARNING: PRBS results deviate >30% from step test.")
            print("  Falling back to step-test results.")
            print()
            return None

    print("  PRBS results accepted.")
    print()
    return result


# ---------------------------------------------------------------------------
# Phase 3: PI Calculation
# ---------------------------------------------------------------------------

def phase_calculate(
    step_result: Dict[str, Any],
    prbs_result: Optional[Dict[str, Any]],
    args: argparse.Namespace,
) -> Tuple[float, float, float, float, str]:
    """Compute PI gains from identified R/L parameters.

    Prefers PRBS results over step results.  Sends the computed gains
    to the MCU unless running in dry-run mode.

    Args:
        step_result: Result dict from Phase 1.
        prbs_result: Result dict from Phase 2, or None.
        args: Parsed CLI arguments.

    Returns:
        Tuple of (Kp, Ki, R, L, method).
    """
    print("[Phase 3] PI Gain Calculation")
    print(f"  Bandwidth target: {args.bw} rad/s")

    # Use PRBS if available, else step
    if prbs_result is not None and prbs_result.get("R") is not None:
        R = prbs_result["R"]
        L = prbs_result["L"]
        method = "prbs"
    else:
        R = step_result.get("R", REF_R)
        L = step_result.get("L", REF_L)
        method = "step"

    Kp, Ki = calculate_pi(R, L, omega_c=args.bw)

    print(f"  Method: {method}")
    print(f"  R={R:.4f} Ohm, L={L:.6f} H ({L * 1e6:.1f} uH)")
    print(f"  Computed Kp={Kp:.4f}, Ki={Ki:.2f}")

    if args.dry_run:
        print("  DRY-RUN: Gains NOT sent to MCU.")
    else:
        iface_cmd = f"P={Kp:.6f},I={Ki:.6f}"
        # The iface arg isn't passed to this function — we handle sending in main()
        print(f"  Sending to MCU: P={Kp:.4f},I={Ki:.2f}")

    print()
    return (Kp, Ki, R, L, method)


# ---------------------------------------------------------------------------
# Phase 4: Verification (helper — called from main)
# ---------------------------------------------------------------------------

def _print_verification_table(
    orig_metrics: Dict[str, float],
    new_metrics: Dict[str, float],
    rollback: bool,
    passed: bool,
) -> None:
    """Print a formatted A/B comparison table."""
    print("[Phase 4] Verification")
    print()
    print(f"  {'Metric':<24} | {'Original':>10} | {'New':>10}")
    print(f"  {'-' * 24}-+-{'-' * 10}-+-{'-' * 10}")

    rows = [
        ("Rise Time (ms)", "rise_time_ms"),
        ("Overshoot (%)", "overshoot_pct"),
        ("Settling Time (ms)", "settling_time_ms"),
        ("Steady-State Err (A)", "steady_state_error_a"),
    ]

    for label, key in rows:
        orig_val = orig_metrics.get(key, 0.0)
        new_val = new_metrics.get(key, 0.0)
        print(f"  {label:<24} | {orig_val:>10.2f} | {new_val:>10.2f}")

    print()
    if passed:
        print("  VERDICT: PASS — new gains accepted.")
    else:
        print("  VERDICT: ROLLBACK — reverted to original gains.")
    print()


# ---------------------------------------------------------------------------
# main()
# ---------------------------------------------------------------------------

def main() -> None:
    """CLI entry point for the FOC auto-tuner.

    Parses arguments, runs phases 0-4, and optionally saves JSON output.
    """
    parser = argparse.ArgumentParser(
        description="FOC Current-Loop Auto-Tuner for STM32G431",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Examples:
  python -m tools.auto_tuner
  python -m tools.auto_tuner --port COM3 --bw 5000
  python -m tools.auto_tuner --dry-run
  python -m tools.auto_tuner --no-prbs --save results.json
""",
    )

    parser.add_argument(
        "--port",
        type=str,
        default=None,
        help="Serial port (e.g. COM3). Auto-detected if omitted.",
    )
    parser.add_argument(
        "--bw",
        type=float,
        default=DEFAULT_OMEGA_C,
        help=f"Target current-loop bandwidth in rad/s (default: {DEFAULT_OMEGA_C}).",
    )
    parser.add_argument(
        "--no-prbs",
        action="store_true",
        help="Skip PRBS identification phase.",
    )
    parser.add_argument(
        "--step-amps",
        type=float,
        default=1.0,
        help="Step test amplitude in amps (default: 1.0).",
    )
    parser.add_argument(
        "--prbs-amps",
        type=float,
        default=1.0,
        help="PRBS amplitude in amps (default: 1.0).",
    )
    parser.add_argument(
        "--prbs-length",
        type=int,
        default=127,
        help="PRBS sequence length (default: 127).",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run identification and calculation without modifying the MCU.",
    )
    parser.add_argument(
        "--save",
        type=str,
        default=None,
        help="Save results as JSON to the specified file path.",
    )

    args = parser.parse_args()

    iface: Optional[SerialInterface] = None

    try:
        # ---- Phase 0: Connect ------------------------------------------------
        iface = phase_connect(args.port)

        # ---- Phase 1: Step Test ----------------------------------------------
        step_result = phase_step_test(iface, args)

        # ---- Phase 2: PRBS Identification ------------------------------------
        prbs_result = phase_prbs(iface, args, step_result)

        # ---- Phase 3: PI Calculation -----------------------------------------
        Kp, Ki, R, L, method = phase_calculate(step_result, prbs_result, args)

        # Send gains to MCU (unless dry-run)
        if not args.dry_run:
            iface.send_cmd(f"P={Kp:.6f},I={Ki:.6f}")
            iface.wait_for_flag(timeout_s=2.0)
            time.sleep(0.05)  # 50 ms settle

        # ---- Phase 4: Verification -------------------------------------------
        if not args.dry_run:
            verif_result = run_verification(
                iface,
                kp_new=Kp,
                ki_new=Ki,
                kp_orig=DEFAULT_KP,
                ki_orig=DEFAULT_KI,
                amplitude=args.step_amps,
            )

            _print_verification_table(
                verif_result["original"],
                verif_result["new"],
                verif_result["rollback"],
                verif_result["passed"],
            )
        else:
            print("[Phase 4] Verification — SKIPPED (dry-run)")
            print()

        # ---- Final Output ----------------------------------------------------
        print("=" * 60)
        print("  Tuning Complete")
        print(f"  Method: {method}")
        print(f"  R = {R:.4f} Ohm")
        print(f"  L = {L:.6f} H ({L * 1e6:.1f} uH)")
        print(f"  Kp = {Kp:.4f}")
        print(f"  Ki = {Ki:.2f}")
        print(f"  Bandwidth = {args.bw} rad/s")
        print("=" * 60)

        # ---- JSON Output -----------------------------------------------------
        if args.save:
            output = {
                "R_ohm": R,
                "L_uh": L * 1e6,
                "Kp": Kp,
                "Ki": Ki,
                "bandwidth_rad_s": args.bw,
                "method": method,
            }
            with open(args.save, "w", encoding="utf-8") as f:
                json.dump(output, f, indent=2)
            print(f"\nResults saved to: {args.save}")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
        sys.exit(1)
    except Exception as exc:
        print(f"\nERROR: {exc}")
        sys.exit(1)
    finally:
        if iface is not None:
            iface.disconnect()
            print("\nDisconnected.")


# ---------------------------------------------------------------------------
# Direct execution
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    main()
