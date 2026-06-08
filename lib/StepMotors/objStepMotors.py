"""Control basico para motores paso a paso en MicroPython."""

from machine import Pin
import time


class objStepMotor:
    """Controla un motor paso a paso mediante pines STEP, DIR y ENABLE."""

    def __init__(
        self,
        step_pin,
        direction_pin,
        enable_pin=None,
        steps_per_revolution=200,
        step_delay_s=0.001,
        enable_active_low=True,
    ):
        self._step = Pin(step_pin, Pin.OUT)
        self._direction = Pin(direction_pin, Pin.OUT)
        self._enable = Pin(enable_pin, Pin.OUT) if enable_pin is not None else None
        self._steps_per_revolution = max(1, int(steps_per_revolution))
        self._step_delay_s = max(0.000001, float(step_delay_s))
        self._enable_active_low = bool(enable_active_low)
        self._position_steps = 0
        self._enabled = False

        self._step.value(0)
        if self._enable is not None:
            self.disable()

    def enable(self):
        """Habilita el driver del motor si existe pin ENABLE."""

        if self._enable is not None:
            self._enable.value(0 if self._enable_active_low else 1)
        self._enabled = True

    def disable(self):
        """Deshabilita el driver del motor si existe pin ENABLE."""

        if self._enable is not None:
            self._enable.value(1 if self._enable_active_low else 0)
        self._enabled = False

    def set_direction(self, clockwise=True):
        """Configura la direccion del giro."""

        self._direction.value(1 if clockwise else 0)

    def set_step_delay(self, step_delay_s):
        """Actualiza la pausa entre pulsos STEP."""

        self._step_delay_s = max(0.000001, float(step_delay_s))

    def step(self, steps=1, clockwise=True, auto_enable=True):
        """Mueve el motor la cantidad indicada de pasos."""

        total_steps = abs(int(steps))
        if total_steps == 0:
            return self._position_steps

        direction = bool(clockwise)
        if int(steps) < 0:
            direction = not direction

        if auto_enable and not self._enabled:
            self.enable()

        self.set_direction(direction)
        position_delta = 1 if direction else -1

        for _ in range(total_steps):
            self._step.value(1)
            time.sleep(self._step_delay_s)
            self._step.value(0)
            time.sleep(self._step_delay_s)
            self._position_steps += position_delta

        return self._position_steps

    def rotate_degrees(self, degrees, clockwise=True, auto_enable=True):
        """Gira el motor usando grados como unidad."""

        steps = round((abs(float(degrees)) / 360.0) * self._steps_per_revolution)
        if float(degrees) < 0:
            clockwise = not clockwise
        return self.step(steps, clockwise=clockwise, auto_enable=auto_enable)

    def reset_position(self, position_steps=0):
        """Define la posicion actual sin mover el motor."""

        self._position_steps = int(position_steps)
        return self._position_steps

    def get_position(self):
        """Devuelve la posicion acumulada en pasos."""

        return self._position_steps
