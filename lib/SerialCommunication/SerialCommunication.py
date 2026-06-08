"""MicroPython helpers for HC-06 Bluetooth serial communication."""

from __future__ import annotations

import time
from dataclasses import dataclass
from machine import UART


@dataclass
class HC06Config:
    """Configuration details for connecting to an HC-06 module."""

    uart_id: int
    baudrate: int = 9600
    tx_pin: int | None = None
    rx_pin: int | None = None
    timeout_ms: int = 1000


class HC06Controller:
    """Controller for sending AT commands to an HC-06 Bluetooth module."""

    def __init__(self, config: HC06Config) -> None:
        self._config = config
        self._uart = self._open_uart()

    def _open_uart(self) -> UART:
        return UART(
            self._config.uart_id,
            baudrate=self._config.baudrate,
            tx=self._config.tx_pin,
            rx=self._config.rx_pin,
            timeout=self._config.timeout_ms,
        )

    def _send_at_command(self, command: str) -> str:
        self._uart.write(command)
        time.sleep(0.2)
        response = self._uart.read()
        if response is None:
            return ""
        return response.decode("utf-8", errors="ignore").strip()

    def set_name(self, name: str) -> str:
        """Set the Bluetooth device name using AT+NAME command."""

        if not name:
            raise ValueError("El nombre no puede estar vacío.")
        return self._send_at_command(f"AT+NAME{name}")

    def set_password(self, pin: str) -> str:
        """Set the pairing PIN using AT+PIN command."""

        if len(pin) != 4 or not pin.isdigit():
            raise ValueError("El PIN debe contener 4 dígitos numéricos.")
        return self._send_at_command(f"AT+PIN{pin}")
