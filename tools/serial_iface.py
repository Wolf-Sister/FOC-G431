"""
SerialInterface — UART2 communication with STM32G431 FOC controller.

Implements the VOFA+ text protocol:
  - TX: ASCII commands terminated by \\n  (e.g. "T=1.0\\n", "P=1.5,I=400\\n")
  - RX: Telemetry at 100 Hz in "channels: f0,f1,...,f13\\n" format

Channel map (14 channels):
    0  id_target       D-axis target current (A)
    1  id_meas         D-axis actual current (A)
    2  iq_target       Q-axis target current (A) = set_torque
    3  iq_meas         Q-axis actual current (A)
    4  vd_cmd          D-axis voltage command (V)
    5  vq_cmd          Q-axis voltage command (V)
    6  velocity        Filtered mechanical velocity (rad/s)
    7  status_flag     Step-sync flag (1 = cmd received, 0 = after TX)
    8  speed_sp        Speed setpoint (rad/s)
    9  mode            Control mode (0=torque, 1=speed, 2=position)
    10 position_sp     Position setpoint (rad)
    11 pos_meas        Measured position (rad)
    12 raw_adc_a       ADC1 raw value (phase A current)
    13 raw_adc_c       ADC2 raw value (phase C current)

Dependencies: pyserial (>= 3.5)
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional, Tuple

import serial
import serial.tools.list_ports


class SerialInterface:
    """Full-featured UART2 communication class for the STM32G431 VOFA+ protocol."""

    # Channel index constants
    IDX_ID_TARGET = 0
    IDX_ID_MEAS = 1
    IDX_IQ_TARGET = 2
    IDX_IQ_MEAS = 3
    IDX_VD_CMD = 4
    IDX_VQ_CMD = 5
    IDX_VELOCITY = 6
    IDX_STATUS_FLAG = 7
    IDX_SPEED_SP = 8
    IDX_MODE = 9
    IDX_POSITION_SP = 10
    IDX_POS_MEAS = 11
    IDX_RAW_ADC_A = 12
    IDX_RAW_ADC_C = 13

    # Human-readable keys for the dict returned by read_telemetry()
    _CHANNEL_NAMES: Tuple[str, ...] = (
        "id_target",
        "id_meas",
        "iq_target",
        "iq_meas",
        "vd_cmd",
        "vq_cmd",
        "velocity",
        "status_flag",
        "speed_sp",
        "mode",
        "position_sp",
        "pos_meas",
        "raw_adc_a",
        "raw_adc_c",
    )

    # Expected number of fields in a telemetry frame
    _NUM_CHANNELS = 14

    def __init__(self, port: str, baud: int = 115200) -> None:
        """
        Args:
            port: Serial port name (e.g. "COM3" on Windows, "/dev/ttyUSB0" on Linux).
            baud: Baud rate (default 115200, matches STM32G431 UART2 config).
        """
        self._port: str = port
        self._baud: int = baud
        self._ser: Optional[serial.Serial] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def connect(self, timeout_s: float = 3.0) -> bool:
        """Open the serial port and verify that telemetry is flowing.

        After opening, waits up to *timeout_s* seconds for a valid
        telemetry frame.  Returns True on success, False on failure.

        Args:
            timeout_s: Maximum time (seconds) to wait for the first frame.

        Returns:
            True if connected and receiving telemetry, False otherwise.
        """
        try:
            ser = serial.Serial(
                port=self._port,
                baudrate=self._baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout_s,
            )
        except (serial.SerialException, OSError):
            return False

        self._ser = ser

        # Flush any stale data in the input buffer.
        self.flush_input()

        # Wait for at least one valid telemetry frame.
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            frame = self._read_one_frame(timeout=0.2)
            if frame is not None:
                return True

        self.disconnect()
        return False

    def disconnect(self) -> None:
        """Close the serial port if it is open."""
        if self._ser is not None:
            try:
                self._ser.close()
            except serial.SerialException:
                pass
            finally:
                self._ser = None

    def send_cmd(self, cmd: str) -> None:
        """Send an ASCII command string over UART.

        A trailing newline (``\\n``) is appended automatically; do not
        include it in *cmd*.

        Args:
            cmd: Command string, e.g. ``"T=1.0"``, ``"P=1.5,I=400"``.

        Raises:
            RuntimeError: If the serial port is not open.
        """
        self._ensure_open()
        payload = (cmd + "\n").encode("ascii", errors="replace")
        self._ser.write(payload)  # type: ignore[union-attr]

    def read_telemetry(self, timeout_s: float = 0.5) -> Optional[Dict[str, float]]:
        """Read and parse one telemetry frame.

        Blocks for up to *timeout_s* seconds.  Malformed lines (missing
        the ``"channels:"`` prefix, wrong field count, or unparseable
        floats) are silently skipped.

        Args:
            timeout_s: Maximum time (seconds) to wait for a valid frame.

        Returns:
            A dict mapping named keys to float values, or None if no
            valid frame arrived within the timeout.
        """
        frame = self._read_one_frame(timeout=timeout_s)
        if frame is None:
            return None
        return dict(zip(self._CHANNEL_NAMES, frame))

    def flush_input(self) -> None:
        """Discard all data in the serial input buffer."""
        if self._ser is not None and self._ser.is_open:
            self._ser.reset_input_buffer()

    def wait_for_flag(self, timeout_s: float = 2.0) -> bool:
        """Block until ``status_flag == 1`` is observed in telemetry.

        The MCU sets ``status_flag = 1`` after receiving any command
        and clears it to 0 when the next telemetry frame is sent.
        This method reads frames until a frame with ``status_flag == 1``
        is found.

        Args:
            timeout_s: Maximum time (seconds) to wait.

        Returns:
            True if the flag was seen within the timeout, False otherwise.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            frame = self._read_one_frame(timeout=max(0.01, remaining))
            if frame is not None and len(frame) > self.IDX_STATUS_FLAG:
                if frame[self.IDX_STATUS_FLAG] >= 0.5:
                    return True
        return False

    # ------------------------------------------------------------------
    # Static / class-level helpers
    # ------------------------------------------------------------------

    @staticmethod
    def list_ports() -> List[str]:
        """Return a list of available serial port device names.

        On Windows these will look like ``["COM1", "COM3", ...]``; on
        Linux/macOS they will be ``["/dev/ttyUSB0", ...]``.

        Returns:
            List of port device-name strings.
        """
        return [p.device for p in serial.tools.list_ports.comports()]

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _ensure_open(self) -> None:
        """Raise RuntimeError if the serial port is not open."""
        if self._ser is None or not self._ser.is_open:
            raise RuntimeError("Serial port is not open. Call connect() first.")

    def _read_one_frame(self, timeout: float) -> Optional[List[float]]:
        """Try to read a single telemetry line and parse it.

        Reads until a ``\\n`` is found, or the timeout expires.
        Silently discards lines that do not match the expected format.

        Args:
            timeout: Seconds to wait for data.

        Returns:
            List of 14 floats on success, None on timeout / parse failure.
        """
        if self._ser is None or not self._ser.is_open:
            return None

        original_timeout = self._ser.timeout
        try:
            self._ser.timeout = timeout
            line = self._ser.readline()
        finally:
            self._ser.timeout = original_timeout

        if not line:
            return None  # timeout

        return self._parse_telemetry_line(line)

    def _parse_telemetry_line(self, line: bytes) -> Optional[List[float]]:
        """Parse a raw bytes line into a list of 14 floats.

        Expected format: ``b"channels: f0,f1,...,f13\\r\\n"``

        Args:
            line: Raw bytes from the serial port.

        Returns:
            List of 14 floats on success, None on parse failure.
        """
        try:
            text = line.decode("ascii", errors="replace").strip()
        except (UnicodeDecodeError, ValueError):
            return None

        if not text:
            return None

        # Require the "channels:" prefix
        PREFIX = "channels:"
        if not text.startswith(PREFIX):
            return None

        body = text[len(PREFIX):].strip()
        if not body:
            return None

        parts = [p.strip() for p in body.split(",")]
        if len(parts) != self._NUM_CHANNELS:
            return None

        values: List[float] = []
        for p in parts:
            try:
                values.append(float(p))
            except ValueError:
                return None

        return values
