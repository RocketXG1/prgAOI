"""Controlador avanzado de movimiento sincronizado para multiples ejes.

Modulo MicroPython para Raspberry Pi Pico/RP2040. Este archivo coordina ejes
ya existentes, por ejemplo AdvancedBeltStepperAxis o AdvancedScrewStepperAxis,
pero no controla pines, no configura PIO y no genera pulsos STEP directamente.

La funcion principal del controlador es planificar movimientos absolutos o
relativos de N ejes para que todos lleguen aproximadamente al mismo tiempo. Para
ello calcula el tiempo minimo requerido por cada eje, selecciona el mayor como
tiempo comun y solicita a cada eje que genere su propio perfil con ese tiempo.
"""

import time


# Estados publicos del controlador.
STATE_IDLE = "IDLE"
STATE_MOVING = "MOVING"
STATE_HOMING = "HOMING"
STATE_ERROR = "ERROR"
STATE_STOPPED = "STOPPED"

# Modos de perfil publicos. Se duplican como texto para no importar el modulo
# del eje, ya que ese modulo depende de machine/rp2 en MicroPython.
PROFILE_CONSTANT = "CONSTANT"
PROFILE_TRAPEZOIDAL = "TRAPEZOIDAL"


def _sleep_ms(ms):
    """Pausa compatible con MicroPython y CPython para pruebas simples."""

    if hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    else:
        time.sleep(float(ms) / 1000.0)


class AdvancedMotionController:
    """Coordina movimientos sincronizados entre varios ejes avanzados.

    Cada eje registrado debe implementar la API publica de
    objAdvanceStepperAxis.py: get_position(), is_homed(), is_busy(),
    validate_target(), calculate_minimum_time_s(), start_move_to_mm(), update(),
    stop(), home(), enable(), disable() y get_status().

    El controlador solo planifica, valida, arranca y supervisa. Nunca mueve los
    ejes de forma secuencial durante un movimiento sincronizado y nunca llama a
    move_to_mm(), porque ese metodo bloquea eje por eje.
    """

    def __init__(self, axes=None):
        """Crea un controlador con un diccionario opcional de ejes."""

        self.axes = {}
        self.state = STATE_IDLE
        self.error_message = None
        # Alias de compatibilidad con otros controladores del proyecto.
        self.error = None
        self.last_plan = None

        if axes is not None:
            for axis_name in axes:
                self.add_axis(axis_name, axes[axis_name])

    def _normalize_axis_name(self, axis_name):
        """Convierte el nombre de eje a texto en mayusculas."""

        return str(axis_name).upper()

    def _set_error(self, message):
        """Guarda un error del controlador y cambia el estado a ERROR."""

        self.error_message = str(message)
        self.error = self.error_message
        self.state = STATE_ERROR
        return False

    def _clear_error(self):
        """Limpia el ultimo error del controlador."""

        self.error_message = None
        self.error = None

    def _axis_error_message(self, axis):
        """Obtiene el mensaje de error observable de un eje, si existe."""

        if hasattr(axis, "error_message") and axis.error_message:
            return axis.error_message
        if hasattr(axis, "error") and axis.error:
            return axis.error
        if hasattr(axis, "get_status"):
            status = axis.get_status()
            if isinstance(status, dict):
                if "error_message" in status and status["error_message"]:
                    return status["error_message"]
                if "error" in status and status["error"]:
                    return status["error"]
        return None

    def _axis_state(self, axis):
        """Lee el estado de un eje desde get_status() o desde axis.state."""

        if hasattr(axis, "get_status"):
            status = axis.get_status()
            if isinstance(status, dict) and "state" in status:
                return status["state"]
        if hasattr(axis, "state"):
            return axis.state
        return None

    def _axis_error_state(self, axis):
        """Devuelve la constante ERROR propia del eje o el texto esperado."""

        if hasattr(axis, "STATE_ERROR"):
            return axis.STATE_ERROR
        return STATE_ERROR

    def _axis_stopped_state(self, axis):
        """Devuelve la constante STOPPED propia del eje o el texto esperado."""

        if hasattr(axis, "STATE_STOPPED"):
            return axis.STATE_STOPPED
        return STATE_STOPPED

    def _axis_is_homed(self, axis):
        """Devuelve True si el eje tiene HOME valido."""

        if hasattr(axis, "is_homed"):
            return bool(axis.is_homed())
        if hasattr(axis, "is_homed_flag"):
            return bool(axis.is_homed_flag)
        if hasattr(axis, "get_status"):
            status = axis.get_status()
            if isinstance(status, dict) and "homed" in status:
                return bool(status["homed"])
        return False

    def add_axis(self, axis_name, axis_object):
        """Agrega o reemplaza un eje. El nombre se normaliza a mayusculas."""

        name = self._normalize_axis_name(axis_name)
        self.axes[name] = axis_object
        return True

    def get_axis(self, axis_name):
        """Devuelve el eje registrado o None si no existe."""

        name = self._normalize_axis_name(axis_name)
        if name in self.axes:
            return self.axes[name]
        return None

    def _normalize_targets(self, targets):
        """Normaliza nombres de ejes y convierte los valores a float."""

        if targets is None:
            return None
        normalized = {}
        for axis_name in targets:
            normalized[self._normalize_axis_name(axis_name)] = float(targets[axis_name])
        return normalized

    def _validate_axis_names(self, targets):
        """Verifica que todos los ejes solicitados esten registrados."""

        if targets is None:
            return self._set_error("targets no puede ser None")
        for name in targets:
            if name not in self.axes:
                return self._set_error("Eje no registrado: %s" % name)
        return True

    def _validate_homed(self, targets):
        """Verifica que los ejes solicitados tengan HOME."""

        for name in targets:
            if not self._axis_is_homed(self.axes[name]):
                return self._set_error("El eje %s requiere HOME" % name)
        return True

    def _validate_not_busy(self):
        """Evita iniciar planes si el controlador o algun eje esta ocupado."""

        if self.state == STATE_MOVING:
            return self._set_error("El controlador ya esta en movimiento")
        if self.state == STATE_HOMING:
            return self._set_error("El controlador esta ejecutando HOME")
        for name in self.axes:
            axis = self.axes[name]
            if hasattr(axis, "is_busy") and axis.is_busy():
                return self._set_error("El eje %s esta ocupado" % name)
        return True

    def _validate_limits(self, targets):
        """Valida los limites de todos los objetivos antes de mover."""

        for name in targets:
            axis = self.axes[name]
            if not hasattr(axis, "validate_target"):
                return self._set_error("El eje %s no implementa validate_target()" % name)
            if not axis.validate_target(targets[name]):
                axis_error = self._axis_error_message(axis)
                if axis_error:
                    return self._set_error("Objetivo invalido en eje %s: %s" % (name, axis_error))
                return self._set_error("Objetivo invalido en eje %s" % name)
        return True

    def create_synchronized_profile_plan(self, targets, profile_mode=PROFILE_TRAPEZOIDAL):
        """Crea un plan sincronizado para que todos los ejes lleguen juntos.

        El tiempo comun es el mayor tiempo minimo calculado entre los ejes
        solicitados. Los ejes con trayectos mas cortos reciben ese mismo tiempo
        al iniciar el movimiento, por lo que generaran perfiles mas lentos.
        """

        normalized_targets = self._normalize_targets(targets)
        if not self._validate_axis_names(normalized_targets):
            return None

        mode = str(profile_mode).upper()
        if mode != PROFILE_CONSTANT and mode != PROFILE_TRAPEZOIDAL:
            self._set_error("profile_mode invalido: %s" % profile_mode)
            return None

        plan = {
            "profile_mode": mode,
            "common_time_s": 0.0,
            "axes": {},
        }

        common_time_s = 0.0
        for name in normalized_targets:
            axis = self.axes[name]
            current_mm = float(axis.get_position())
            target_mm = float(normalized_targets[name])
            delta_mm = target_mm - current_mm
            distance_mm = abs(delta_mm)

            if distance_mm <= 0.0:
                minimum_time_s = 0.0
            elif mode == PROFILE_CONSTANT:
                max_speed = float(axis.max_speed_mm_s)
                if max_speed <= 0.0:
                    self._set_error("max_speed_mm_s invalido en eje %s" % name)
                    return None
                minimum_time_s = distance_mm / max_speed
            else:
                if not hasattr(axis, "calculate_minimum_time_s"):
                    self._set_error("El eje %s no implementa calculate_minimum_time_s()" % name)
                    return None
                minimum_time_s = float(axis.calculate_minimum_time_s(distance_mm))

            if minimum_time_s > common_time_s:
                common_time_s = minimum_time_s

            plan["axes"][name] = {
                "axis": axis,
                "current_mm": current_mm,
                "target_mm": target_mm,
                "delta_mm": delta_mm,
                "distance_mm": distance_mm,
                "minimum_time_s": minimum_time_s,
                "will_move": distance_mm > 0.0,
            }

        plan["common_time_s"] = common_time_s
        self.last_plan = plan
        return plan

    def move_to(self, targets, profile_mode=PROFILE_TRAPEZOIDAL, wait=True):
        """Ejecuta un movimiento absoluto sincronizado.

        targets debe ser un diccionario como {"X": 150, "Y": 80}. El
        movimiento es absoluto respecto al cero/HOME: pedir X=70 mueve el eje a
        70 mm, no suma 70 mm a la posicion actual.
        """

        normalized_targets = self._normalize_targets(targets)

        if not self._validate_not_busy():
            return False
        if not self._validate_axis_names(normalized_targets):
            return False
        if not self._validate_homed(normalized_targets):
            return False
        if not self._validate_limits(normalized_targets):
            return False

        plan = self.create_synchronized_profile_plan(normalized_targets, profile_mode)
        if plan is None:
            return False

        common_time_s = float(plan["common_time_s"])
        if common_time_s <= 0.0:
            self.state = STATE_IDLE
            self._clear_error()
            return True

        self._clear_error()
        self.state = STATE_MOVING
        started_axes = []

        # Arranque no bloqueante: todos los ejes reciben el mismo total_time_s.
        for name in plan["axes"]:
            axis_plan = plan["axes"][name]
            if axis_plan["will_move"]:
                axis = axis_plan["axis"]
                ok = axis.start_move_to_mm(
                    axis_plan["target_mm"],
                    profile_mode=plan["profile_mode"],
                    total_time_s=common_time_s,
                )
                if not ok:
                    for started_axis in started_axes:
                        started_axis.stop()
                    axis_error = self._axis_error_message(axis)
                    if axis_error:
                        return self._set_error("No se pudo iniciar eje %s: %s" % (name, axis_error))
                    return self._set_error("No se pudo iniciar eje %s" % name)
                started_axes.append(axis)

        if wait:
            return self.wait_until_done()
        return True

    def move_relative(self, distances, profile_mode=PROFILE_TRAPEZOIDAL, wait=True):
        """Convierte desplazamientos relativos a destinos absolutos y mueve."""

        normalized_distances = self._normalize_targets(distances)
        if not self._validate_axis_names(normalized_distances):
            return False

        targets = {}
        for name in normalized_distances:
            axis = self.axes[name]
            targets[name] = float(axis.get_position()) + float(normalized_distances[name])

        return self.move_to(targets, profile_mode=profile_mode, wait=wait)

    def wait_until_done(self):
        """Supervisa ejes hasta que el movimiento sincronizado termine."""

        while self.state == STATE_MOVING:
            any_busy = False

            for name in self.axes:
                axis = self.axes[name]

                if hasattr(axis, "is_busy") and axis.is_busy():
                    any_busy = True
                    if hasattr(axis, "update"):
                        axis.update()

                axis_state = self._axis_state(axis)
                if axis_state == self._axis_error_state(axis):
                    axis_error = self._axis_error_message(axis)
                    if axis_error:
                        self.error_message = "El eje %s entro en ERROR: %s" % (name, axis_error)
                    else:
                        self.error_message = "El eje %s entro en ERROR" % name
                    self.error = self.error_message
                    self.state = STATE_ERROR
                    return False
                if axis_state == self._axis_stopped_state(axis):
                    self.error_message = "El eje %s entro en STOPPED" % name
                    self.error = self.error_message
                    self.state = STATE_STOPPED
                    return False

            if not any_busy:
                self.state = STATE_IDLE
                self._clear_error()
                return True

            _sleep_ms(1)

        return self.state == STATE_IDLE

    def home_all(self, sequence=None):
        """Ejecuta HOME en todos los ejes de forma secuencial por seguridad."""

        if not self._validate_not_busy():
            return False

        if sequence is None:
            sequence = []
            for name in self.axes:
                sequence.append(name)

        normalized_sequence = []
        for axis_name in sequence:
            normalized_sequence.append(self._normalize_axis_name(axis_name))

        self._clear_error()
        self.state = STATE_HOMING

        for name in normalized_sequence:
            axis = self.get_axis(name)
            if axis is None:
                return self._set_error("Eje no registrado en secuencia HOME: %s" % name)
            if not hasattr(axis, "home"):
                return self._set_error("El eje %s no implementa home()" % name)
            if not axis.home():
                axis_error = self._axis_error_message(axis)
                if axis_error:
                    return self._set_error("Fallo HOME en eje %s: %s" % (name, axis_error))
                return self._set_error("Fallo HOME en eje %s" % name)

        self.state = STATE_IDLE
        self._clear_error()
        return True

    def stop_all(self):
        """Detiene todos los ejes y marca paro general."""

        for name in self.axes:
            axis = self.axes[name]
            if hasattr(axis, "stop"):
                axis.stop()
        self.state = STATE_STOPPED
        self.error_message = "Paro general activado"
        self.error = self.error_message
        return True

    def enable_all(self):
        """Habilita todos los drivers mediante cada objeto de eje."""

        for name in self.axes:
            if hasattr(self.axes[name], "enable"):
                self.axes[name].enable()
        return True

    def disable_all(self):
        """Deshabilita todos los drivers mediante cada objeto de eje."""

        for name in self.axes:
            if hasattr(self.axes[name], "disable"):
                self.axes[name].disable()
        return True

    def get_position(self):
        """Devuelve las posiciones actuales de todos los ejes registrados."""

        positions = {}
        for name in self.axes:
            positions[name] = self.axes[name].get_position()
        return positions

    def all_homed(self):
        """True si todos los ejes registrados tienen HOME."""

        for name in self.axes:
            if not self._axis_is_homed(self.axes[name]):
                return False
        return True

    def is_busy(self):
        """True si el controlador o cualquier eje esta ocupado."""

        if self.state == STATE_MOVING or self.state == STATE_HOMING:
            return True
        for name in self.axes:
            axis = self.axes[name]
            if hasattr(axis, "is_busy") and axis.is_busy():
                return True
        return False

    def get_last_plan(self):
        """Devuelve el ultimo plan sincronizado calculado."""

        return self.last_plan

    def get_status(self):
        """Devuelve estado completo del controlador y de los ejes."""

        axes_status = {}
        for name in self.axes:
            axis = self.axes[name]
            if hasattr(axis, "get_status"):
                axes_status[name] = axis.get_status()
            else:
                axes_status[name] = {"state": self._axis_state(axis)}

        return {
            "controller_state": self.state,
            "controller_error": self.error_message,
            "all_homed": self.all_homed(),
            "positions": self.get_position(),
            "last_plan": self.last_plan,
            "axes": axes_status,
        }
