"""Controlador de movimiento para coordinar multiples ejes StepperAxis.

Modulo MicroPython para Raspberry Pi Pico/RP2040. Este archivo no controla
pines, no configura PIO y no genera pulsos STEP; solamente administra objetos
de eje ya existentes (por ejemplo BeltStepperAxis o ScrewStepperAxis) y dispara
movimientos no bloqueantes para que cada eje avance con su propia StateMachine.
"""

import time


# Estados publicos del controlador.
STATE_IDLE = "IDLE"
STATE_MOVING = "MOVING"
STATE_HOMING = "HOMING"
STATE_ERROR = "ERROR"
STATE_STOPPED = "STOPPED"

# Estados esperados en los ejes. Se usan como texto para no depender de imports.
AXIS_STATE_MOVING = "MOVING"
AXIS_STATE_HOMING = "HOMING"
AXIS_STATE_ERROR = "ERROR"
AXIS_STATE_STOPPED = "STOPPED"


class MotionController:
    """Coordina movimientos simples en paralelo entre varios ejes.

    Cada eje debe exponer la interfaz de objStepperAxis.py. El controlador
    valida los ejes solicitados, inicia todos con start_move_to_mm() y despues,
    si se solicita, espera a que todos terminen. No sincroniza velocidades ni
    genera pulsos; cada eje mantiene su propia configuracion y PIO.
    """

    def __init__(self, axes=None):
        """Crea un controlador con un diccionario opcional de ejes."""

        self.axes = {}
        self.state = STATE_IDLE
        self.error = None

        if axes is not None:
            for axis_name in axes:
                self.add_axis(axis_name, axes[axis_name])

    def _normalize_axis_name(self, axis_name):
        """Convierte el nombre del eje a texto en mayusculas."""

        return str(axis_name).upper()

    def _set_error(self, message):
        """Guarda el error del controlador y cambia a estado ERROR."""

        self.error = str(message)
        self.state = STATE_ERROR
        return False

    def _clear_error(self):
        """Limpia el ultimo error del controlador."""

        self.error = None

    def _axis_state(self, axis):
        """Obtiene el estado de un eje usando get_status() o el atributo state."""

        if hasattr(axis, "get_status"):
            status = axis.get_status()
            if isinstance(status, dict) and "state" in status:
                return status["state"]
        if hasattr(axis, "state"):
            return axis.state
        return None

    def _axis_done(self, axis):
        """Detecta la bandera interna de finalizacion si el eje la expone."""

        if hasattr(axis, "_move_done"):
            return bool(axis._move_done)
        if hasattr(axis, "done"):
            return bool(axis.done)
        return False

    def _axis_is_busy(self, axis):
        """Devuelve True si un eje indica movimiento u homing."""

        if hasattr(axis, "is_busy"):
            return bool(axis.is_busy())

        state = self._axis_state(axis)
        return state == AXIS_STATE_MOVING or state == AXIS_STATE_HOMING

    def _axis_is_homed(self, axis):
        """Devuelve True si un eje ya hizo HOME."""

        if hasattr(axis, "is_homed"):
            return bool(axis.is_homed())

        if hasattr(axis, "is_homed_flag"):
            return bool(axis.is_homed_flag)

        if hasattr(axis, "get_status"):
            status = axis.get_status()
            if isinstance(status, dict) and "homed" in status:
                return bool(status["homed"])

        return False

    def _validate_axis_target(self, axis, target_mm):
        """Valida un destino usando la API publica del eje."""

        if hasattr(axis, "validate_target"):
            return bool(axis.validate_target(target_mm))
        if hasattr(axis, "_validate_target"):
            # Compatibilidad con versiones donde la validacion aun es interna.
            return bool(axis._validate_target(target_mm))
        return self._set_error("El eje no implementa validate_target()")

    def add_axis(self, axis_name, axis_object):
        """Agrega o reemplaza un eje en el controlador."""

        name = self._normalize_axis_name(axis_name)
        self.axes[name] = axis_object
        return True

    def get_axis(self, axis_name):
        """Devuelve el objeto de eje registrado o None si no existe."""

        name = self._normalize_axis_name(axis_name)
        if name in self.axes:
            return self.axes[name]
        return None

    def move_to(self, targets, wait=True):
        """Inicia un movimiento absoluto para varios ejes.

        targets debe ser un diccionario como {"X": 150, "Y": 80}. Todos los
        ejes solicitados se arrancan con start_move_to_mm() antes de esperar.
        """

        if targets is None:
            return self._set_error("targets no puede ser None")

        normalized_targets = {}
        for axis_name in targets:
            name = self._normalize_axis_name(axis_name)
            axis = self.get_axis(name)
            if axis is None:
                return self._set_error("Eje no registrado: %s" % name)
            normalized_targets[name] = targets[axis_name]

        # No permitir iniciar un bloque nuevo mientras cualquier eje registrado
        # siga ocupado; esto evita mezclar trayectorias incompletas.
        for name in self.axes:
            if self._axis_is_busy(self.axes[name]):
                return self._set_error("El eje %s esta ocupado" % name)

        # Solo los ejes pedidos necesitan HOME para este movimiento.
        for name in normalized_targets:
            if not self._axis_is_homed(self.axes[name]):
                return self._set_error("El eje %s requiere HOME" % name)

        # Validar todos los limites antes de arrancar cualquier StateMachine.
        for name in normalized_targets:
            if not self._validate_axis_target(self.axes[name], normalized_targets[name]):
                axis_error = None
                if hasattr(self.axes[name], "error"):
                    axis_error = self.axes[name].error
                if axis_error:
                    return self._set_error("Objetivo invalido en eje %s: %s" % (name, axis_error))
                return self._set_error("Objetivo invalido en eje %s" % name)

        self._clear_error()
        started_axes = []
        self.state = STATE_MOVING

        # Arranque no bloqueante: no se llama move_to_mm() porque bloquearia eje
        # por eje. Cada eje genera sus pulsos con su propia PIO.
        for name in normalized_targets:
            axis = self.axes[name]
            if not axis.start_move_to_mm(normalized_targets[name]):
                for started_axis in started_axes:
                    started_axis.stop()
                self.state = STATE_ERROR
                axis_error = None
                if hasattr(axis, "error"):
                    axis_error = axis.error
                if axis_error:
                    self.error = "No se pudo iniciar eje %s: %s" % (name, axis_error)
                else:
                    self.error = "No se pudo iniciar eje %s" % name
                return False
            started_axes.append(axis)

        if wait:
            return self.wait_until_done()

        return True

    def move_relative(self, distances, wait=True):
        """Convierte desplazamientos relativos a objetivos absolutos y mueve."""

        if distances is None:
            return self._set_error("distances no puede ser None")

        targets = {}
        for axis_name in distances:
            name = self._normalize_axis_name(axis_name)
            axis = self.get_axis(name)
            if axis is None:
                return self._set_error("Eje no registrado: %s" % name)
            targets[name] = axis.get_position() + float(distances[axis_name])

        return self.move_to(targets, wait)

    def wait_until_done(self):
        """Espera hasta que todos los ejes registrados terminen.

        Si un eje ya marco done=True pero su estado visible sigue en MOVING, se
        llama wait_until_done() del eje para que consolide su posicion/estado.
        """

        self.state = STATE_MOVING

        while True:
            any_busy = False

            for name in self.axes:
                axis = self.axes[name]
                state = self._axis_state(axis)

                if state == AXIS_STATE_ERROR or state == AXIS_STATE_STOPPED:
                    self.error = "El eje %s entro en estado %s" % (name, state)
                    self.state = state
                    return False

                if state == AXIS_STATE_MOVING:
                    if self._axis_done(axis) and hasattr(axis, "wait_until_done"):
                        axis.wait_until_done()
                        state = self._axis_state(axis)
                        if state == AXIS_STATE_ERROR or state == AXIS_STATE_STOPPED:
                            self.error = "El eje %s entro en estado %s" % (name, state)
                            self.state = state
                            return False
                    else:
                        any_busy = True

                elif state == AXIS_STATE_HOMING:
                    any_busy = True

            if not any_busy:
                self.state = STATE_IDLE
                self._clear_error()
                return True

            time.sleep_ms(1)

    def home_all(self, sequence=None):
        """Ejecuta HOME secuencialmente en todos los ejes o en la secuencia dada."""

        if sequence is None:
            sequence = []
            for name in self.axes:
                sequence.append(name)

        self._clear_error()
        self.state = STATE_HOMING

        for axis_name in sequence:
            name = self._normalize_axis_name(axis_name)
            axis = self.get_axis(name)
            if axis is None:
                return self._set_error("Eje no registrado en secuencia HOME: %s" % name)
            if not hasattr(axis, "home"):
                return self._set_error("El eje %s no implementa home()" % name)
            if not axis.home():
                axis_error = None
                if hasattr(axis, "error"):
                    axis_error = axis.error
                if axis_error:
                    return self._set_error("Fallo HOME en eje %s: %s" % (name, axis_error))
                return self._set_error("Fallo HOME en eje %s" % name)

        self.state = STATE_IDLE
        self._clear_error()
        return True

    def stop_all(self):
        """Detiene todos los ejes y deja el controlador en STOPPED."""

        for name in self.axes:
            self.axes[name].stop()
        self.state = STATE_STOPPED
        self.error = "Movimiento detenido; se requiere HOME nuevamente en los ejes"
        return True

    def enable_all(self):
        """Habilita todos los drivers mediante cada objeto de eje."""

        for name in self.axes:
            self.axes[name].enable()
        return True

    def disable_all(self):
        """Deshabilita todos los drivers mediante cada objeto de eje."""

        for name in self.axes:
            self.axes[name].disable()
        return True

    def set_axis_speed(self, axis_name, speed_mm_s):
        """Actualiza la velocidad fija de un eje registrado."""

        name = self._normalize_axis_name(axis_name)
        axis = self.get_axis(name)
        if axis is None:
            return self._set_error("Eje no registrado: %s" % name)
        if not axis.set_speed_mm_s(speed_mm_s):
            axis_error = None
            if hasattr(axis, "error"):
                axis_error = axis.error
            if axis_error:
                return self._set_error("No se pudo cambiar velocidad de %s: %s" % (name, axis_error))
            return self._set_error("No se pudo cambiar velocidad de %s" % name)
        return True

    def set_all_speeds(self, speed_mm_s):
        """Actualiza la velocidad fija de todos los ejes registrados."""

        for name in self.axes:
            if not self.set_axis_speed(name, speed_mm_s):
                return False
        return True

    def get_position(self):
        """Devuelve un diccionario con las posiciones actuales en milimetros."""

        positions = {}
        for name in self.axes:
            positions[name] = self.axes[name].get_position()
        return positions

    def all_homed(self):
        """True solo si todos los ejes registrados tienen HOME."""

        for name in self.axes:
            if not self._axis_is_homed(self.axes[name]):
                return False
        return True

    def is_busy(self):
        """True si el controlador o cualquier eje esta ocupado."""

        if self.state == STATE_MOVING or self.state == STATE_HOMING:
            return True
        for name in self.axes:
            if self._axis_is_busy(self.axes[name]):
                return True
        return False

    def get_status(self):
        """Devuelve estado completo del controlador y de cada eje."""

        axes_status = {}
        for name in self.axes:
            axis = self.axes[name]
            if hasattr(axis, "get_status"):
                axes_status[name] = axis.get_status()
            else:
                axes_status[name] = {"state": self._axis_state(axis)}

        return {
            "controller_state": self.state,
            "controller_error": self.error,
            "all_homed": self.all_homed(),
            "positions": self.get_position(),
            "axes": axes_status,
        }
