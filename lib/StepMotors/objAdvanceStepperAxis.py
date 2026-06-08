"""Eje lineal avanzado con motor a pasos, PIO y perfiles de movimiento.

Modulo para Raspberry Pi Pico/RP2040 con MicroPython. Cada instancia controla
un solo eje lineal mediante driver externo STEP/DIR/ENABLE (TB6600, TMC,
DRV8825, A4988 o similar). Soporta movimientos absolutos y relativos con
perfil de velocidad constante o perfil trapezoidal/triangular segmentado.

Este archivo solo representa un eje individual: no implementa TCP/IP, Excel,
interfaz grafica ni coordinacion multi-eje.
"""

from machine import Pin
import rp2
import time
import math


# Estados publicos del eje.
STATE_NOT_HOMED = "NOT_HOMED"
STATE_IDLE = "IDLE"
STATE_MOVING = "MOVING"
STATE_HOMING = "HOMING"
STATE_ERROR = "ERROR"
STATE_STOPPED = "STOPPED"

# Modos de perfil publicos.
PROFILE_CONSTANT = "CONSTANT"
PROFILE_TRAPEZOIDAL = "TRAPEZOIDAL"

# El programa PIO usa 4 ciclos por cada pulso STEP:
# set alto, nop, set bajo, jmp condicional.
PIO_CYCLES_PER_STEP = 4


@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def step_pulse_pio():
    """Genera N pulsos STEP recibidos por FIFO y dispara IRQ al terminar.

    Se envia ``steps - 1`` al FIFO. Con ``jmp(x_dec, ...)`` el salto usa el
    valor previo de X: X=0 produce 1 pulso y X=N-1 produce N pulsos.
    """

    pull(block)
    mov(x, osr)
    label("pulse_loop")
    set(pins, 1)
    nop()
    set(pins, 0)
    jmp(x_dec, "pulse_loop")
    irq(rel(0))


class AdvancedStepperAxisBase:
    """Clase base para un eje lineal avanzado con STEP/DIR/ENABLE/HOME.

    Las clases hijas calculan ``steps_per_mm`` segun la transmision mecanica.
    Esta clase administra estado, limites, HOME, PIO y perfiles segmentados.
    """

    def __init__(
        self,
        name,
        step_pin,
        dir_pin,
        enable_pin,
        sm_id,
        motor_steps_per_rev=200,
        microsteps=1,
        max_speed_mm_s=100.0,
        acceleration_mm_s2=100.0,
        deceleration_mm_s2=100.0,
        min_speed_mm_s=1.0,
        profile_segment_time_s=0.02,
        min_position_mm=0.0,
        max_position_mm=100.0,
        home_pin=None,
        home_direction=-1,
        home_speed_mm_s=5.0,
        home_backoff_mm=2.0,
        invert_direction=False,
        enable_active_low=True,
        home_active_low=True,
    ):
        self.name = str(name)
        self.step_pin_number = int(step_pin)
        self.dir_pin_number = int(dir_pin)
        self.enable_pin_number = int(enable_pin)
        self.sm_id = int(sm_id)

        self.motor_steps_per_rev = int(motor_steps_per_rev)
        self.microsteps = int(microsteps)
        self.max_speed_mm_s = float(max_speed_mm_s)
        self.acceleration_mm_s2 = float(acceleration_mm_s2)
        self.deceleration_mm_s2 = float(deceleration_mm_s2)
        self.min_speed_mm_s = float(min_speed_mm_s)
        self.profile_segment_time_s = float(profile_segment_time_s)
        self.min_position_mm = float(min_position_mm)
        self.max_position_mm = float(max_position_mm)
        self.home_direction = -1 if int(home_direction) < 0 else 1
        self.home_speed_mm_s = float(home_speed_mm_s)
        self.home_backoff_mm = abs(float(home_backoff_mm))
        self.invert_direction = bool(invert_direction)
        self.enable_active_low = bool(enable_active_low)
        self.home_active_low = bool(home_active_low)

        self._validate_constructor_values()
        self.steps_per_mm = float(self.calculate_steps_per_mm())
        if self.steps_per_mm <= 0:
            raise ValueError("steps_per_mm debe ser mayor que cero")

        self.step_pin = Pin(self.step_pin_number, Pin.OUT)
        self.dir_pin = Pin(self.dir_pin_number, Pin.OUT)
        self.enable_pin = Pin(self.enable_pin_number, Pin.OUT)
        if home_pin is None:
            self.home_pin_number = None
            self.home_pin = None
        else:
            self.home_pin_number = int(home_pin)
            self.home_pin = Pin(self.home_pin_number, Pin.IN, Pin.PULL_UP)

        self.current_position_mm = 0.0
        self.current_position_steps = 0
        self.target_position_mm = None
        self.target_position_steps = None
        self.error = None
        self.state = STATE_NOT_HOMED
        self.is_homed_flag = False
        self._enabled = False

        self._profile_segments = []
        self._current_segment_index = -1
        self._current_segment_steps = 0
        self._completed_profile_steps = 0
        self._move_total_steps = 0
        self._move_direction_sign = 1
        self._move_start_position_steps = 0
        self._move_start_position_mm = 0.0
        self._move_done = True
        self._segment_done = True
        self._active_profile_mode = None
        self._requested_total_time_s = None
        self._planned_total_time_s = 0.0

        self.step_pin.value(0)
        self.disable()
        self.sm = None
        self.init_pio()

    def _validate_constructor_values(self):
        if self.motor_steps_per_rev <= 0:
            raise ValueError("motor_steps_per_rev debe ser mayor que cero")
        if self.microsteps <= 0:
            raise ValueError("microsteps debe ser mayor que cero")
        if self.max_speed_mm_s <= 0:
            raise ValueError("max_speed_mm_s debe ser mayor que cero")
        if self.acceleration_mm_s2 <= 0:
            raise ValueError("acceleration_mm_s2 debe ser mayor que cero")
        if self.deceleration_mm_s2 <= 0:
            raise ValueError("deceleration_mm_s2 debe ser mayor que cero")
        if self.min_speed_mm_s <= 0 or self.min_speed_mm_s > self.max_speed_mm_s:
            raise ValueError("min_speed_mm_s debe ser > 0 y <= max_speed_mm_s")
        if self.profile_segment_time_s <= 0:
            raise ValueError("profile_segment_time_s debe ser mayor que cero")
        if self.min_position_mm >= self.max_position_mm:
            raise ValueError("min_position_mm debe ser menor que max_position_mm")
        if self.home_speed_mm_s <= 0 or self.home_speed_mm_s > self.max_speed_mm_s:
            raise ValueError("home_speed_mm_s debe ser > 0 y <= max_speed_mm_s")

    def calculate_steps_per_mm(self):
        """Calcula steps/mm. Debe implementarse en las clases hijas."""

        raise NotImplementedError

    def init_pio(self):
        """Inicializa la StateMachine PIO para pulsos STEP."""

        self.sm = rp2.StateMachine(
            self.sm_id,
            step_pulse_pio,
            freq=self._speed_to_pio_freq(self.min_speed_mm_s),
            set_base=self.step_pin,
        )
        self.sm.irq(self._on_pio_done)
        self.sm.active(0)
        return self.sm

    def _on_pio_done(self, state_machine):
        """IRQ PIO: marca el segmento actual como terminado."""

        self._segment_done = True

    def _set_error(self, message):
        self.error = str(message)
        self.state = STATE_ERROR
        self._move_done = True
        self._segment_done = True
        return False

    def _clear_error(self):
        self.error = None

    def _mm_to_steps(self, mm_value):
        return int(round(float(mm_value) * self.steps_per_mm))

    def _steps_to_mm(self, steps):
        return float(steps) / self.steps_per_mm

    def _speed_to_pio_freq(self, speed_mm_s):
        steps_per_second = max(abs(float(speed_mm_s)), self.min_speed_mm_s) * self.steps_per_mm
        freq = int(round(steps_per_second * PIO_CYCLES_PER_STEP))
        if freq < PIO_CYCLES_PER_STEP:
            freq = PIO_CYCLES_PER_STEP
        return freq

    def _write_direction(self, direction_sign):
        logical_positive = int(direction_sign) >= 0
        pin_value = 1 if logical_positive else 0
        if self.invert_direction:
            pin_value = 0 if pin_value else 1
        self.dir_pin.value(pin_value)

    def _is_home_active(self):
        if self.home_pin is None:
            return False
        raw_value = self.home_pin.value()
        if self.home_active_low:
            return raw_value == 0
        return raw_value == 1

    def is_home_active(self):
        """Devuelve True si el sensor HOME esta activo."""

        return self._is_home_active()

    def enable(self):
        """Habilita el driver externo."""

        self.enable_pin.value(0 if self.enable_active_low else 1)
        self._enabled = True

    def disable(self):
        """Deshabilita el driver externo."""

        self.enable_pin.value(1 if self.enable_active_low else 0)
        self._enabled = False

    def validate_target(self, target_mm):
        """Valida que el destino absoluto este dentro de los limites fisicos."""

        target = float(target_mm)
        if target < self.min_position_mm or target > self.max_position_mm:
            return self._set_error(
                "Destino fuera de limites: %.3f mm no esta entre %.3f y %.3f mm"
                % (target, self.min_position_mm, self.max_position_mm)
            )
        return True

    def get_position(self):
        """Devuelve la posicion absoluta actual en mm."""

        return self.current_position_mm

    def is_homed(self):
        """Devuelve True si el eje tiene referencia HOME valida."""

        return bool(self.is_homed_flag)

    def is_busy(self):
        """Devuelve True si el eje esta moviendose o haciendo HOME."""

        return self.state == STATE_MOVING or self.state == STATE_HOMING

    def get_status(self):
        """Devuelve un diccionario completo con el estado observable del eje."""

        return {
            "name": self.name,
            "state": self.state,
            "error": self.error,
            "position_mm": self.current_position_mm,
            "position_steps": self.current_position_steps,
            "target_position_mm": self.target_position_mm,
            "target_position_steps": self.target_position_steps,
            "homed": self.is_homed_flag,
            "busy": self.is_busy(),
            "enabled": self._enabled,
            "steps_per_mm": self.steps_per_mm,
            "max_speed_mm_s": self.max_speed_mm_s,
            "acceleration_mm_s2": self.acceleration_mm_s2,
            "deceleration_mm_s2": self.deceleration_mm_s2,
            "min_speed_mm_s": self.min_speed_mm_s,
            "profile_segment_time_s": self.profile_segment_time_s,
            "min_position_mm": self.min_position_mm,
            "max_position_mm": self.max_position_mm,
            "active_profile_mode": self._active_profile_mode,
            "requested_total_time_s": self._requested_total_time_s,
            "planned_total_time_s": self._planned_total_time_s,
            "segment_index": self._current_segment_index,
            "segment_count": len(self._profile_segments),
            "current_segment_steps": self._current_segment_steps,
            "completed_profile_steps": self._completed_profile_steps,
            "move_total_steps": self._move_total_steps,
            "pio_cycles_per_step": PIO_CYCLES_PER_STEP,
        }

    def calculate_minimum_time_s(self, distance_mm):
        """Calcula el tiempo minimo para una distancia con a, d y vmax.

        Diferencia perfil trapezoidal y triangular. La distancia puede recibirse
        con signo; para el calculo solo importa su magnitud.
        """

        distance = abs(float(distance_mm))
        if distance <= 0:
            return 0.0

        vmax = self.max_speed_mm_s
        accel = self.acceleration_mm_s2
        decel = self.deceleration_mm_s2
        accel_distance = (vmax * vmax) / (2.0 * accel)
        decel_distance = (vmax * vmax) / (2.0 * decel)

        if distance >= accel_distance + decel_distance:
            cruise_distance = distance - accel_distance - decel_distance
            return (vmax / accel) + (cruise_distance / vmax) + (vmax / decel)

        # Perfil triangular: distancia = vp^2/(2a) + vp^2/(2d)
        peak_speed = math.sqrt((2.0 * distance * accel * decel) / (accel + decel))
        return (peak_speed / accel) + (peak_speed / decel)

    def _profile_peak_speed_for_time(self, distance_mm, total_time_s):
        """Obtiene velocidad pico/crucero para cubrir distancia en total_time_s."""

        distance = abs(float(distance_mm))
        total_time = float(total_time_s)
        if distance <= 0 or total_time <= 0:
            return 0.0

        min_time = self.calculate_minimum_time_s(distance)
        if total_time <= min_time:
            total_time = min_time

        accel = self.acceleration_mm_s2
        decel = self.deceleration_mm_s2
        c = (1.0 / (2.0 * accel)) + (1.0 / (2.0 * decel))
        discriminant = (total_time * total_time) - (4.0 * c * distance)

        if discriminant < 0.0:
            # Caso triangular minimo con pequeno error de redondeo numerico.
            return math.sqrt((2.0 * distance * accel * decel) / (accel + decel))

        root = (total_time - math.sqrt(discriminant)) / (2.0 * c)
        if root > self.max_speed_mm_s:
            root = self.max_speed_mm_s
        if root < self.min_speed_mm_s and distance > 0:
            root = self.min_speed_mm_s
        return root

    def _speed_at_time(self, elapsed_s, peak_speed, accel_time, cruise_time):
        if elapsed_s <= accel_time:
            return peak_speed if accel_time <= 0 else peak_speed * (elapsed_s / accel_time)
        if elapsed_s <= accel_time + cruise_time:
            return peak_speed
        decel_elapsed = elapsed_s - accel_time - cruise_time
        decel_time = peak_speed / self.deceleration_mm_s2
        if decel_time <= 0:
            return self.min_speed_mm_s
        speed = peak_speed * (1.0 - (decel_elapsed / decel_time))
        return max(speed, 0.0)


    def _distance_at_time(self, elapsed_s, peak_speed, accel_time, cruise_time):
        """Distancia recorrida desde t=0 segun el perfil continuo."""

        t = max(0.0, float(elapsed_s))
        accel = self.acceleration_mm_s2
        decel = self.deceleration_mm_s2
        accel_distance = 0.5 * accel * accel_time * accel_time
        cruise_distance = peak_speed * cruise_time

        if t <= accel_time:
            return 0.5 * accel * t * t
        if t <= accel_time + cruise_time:
            return accel_distance + peak_speed * (t - accel_time)

        decel_elapsed = t - accel_time - cruise_time
        decel_time = peak_speed / decel
        if decel_elapsed >= decel_time:
            return accel_distance + cruise_distance + (0.5 * decel * decel_time * decel_time)
        return accel_distance + cruise_distance + (peak_speed * decel_elapsed) - (0.5 * decel * decel_elapsed * decel_elapsed)

    def _append_segment(self, segments, steps, speed_mm_s):
        steps = int(steps)
        if steps <= 0:
            return
        speed = abs(float(speed_mm_s))
        if speed < self.min_speed_mm_s:
            speed = self.min_speed_mm_s
        if speed > self.max_speed_mm_s:
            speed = self.max_speed_mm_s
        if segments and abs(segments[-1]["speed_mm_s"] - speed) < 0.000001:
            segments[-1]["steps"] += steps
        else:
            segments.append({"steps": steps, "speed_mm_s": speed})

    def create_trapezoidal_profile(self, distance_mm, total_time_s=None):
        """Crea segmentos para un perfil trapezoidal o triangular.

        Si ``total_time_s`` se proporciona y es mayor al minimo, se reduce la
        velocidad pico/crucero para finalizar aproximadamente en ese tiempo. La
        suma final de ``steps`` siempre coincide con la distancia cuantizada en
        pasos.
        """

        distance = abs(float(distance_mm))
        total_steps = self._mm_to_steps(distance)
        if total_steps <= 0:
            return []

        quantized_distance = self._steps_to_mm(total_steps)
        min_time = self.calculate_minimum_time_s(quantized_distance)
        if total_time_s is None or float(total_time_s) <= min_time:
            planned_time = min_time
        else:
            planned_time = float(total_time_s)

        peak_speed = self._profile_peak_speed_for_time(quantized_distance, planned_time)
        if peak_speed <= 0:
            peak_speed = self.min_speed_mm_s

        accel_time = peak_speed / self.acceleration_mm_s2
        decel_time = peak_speed / self.deceleration_mm_s2
        accel_distance = (peak_speed * peak_speed) / (2.0 * self.acceleration_mm_s2)
        decel_distance = (peak_speed * peak_speed) / (2.0 * self.deceleration_mm_s2)
        cruise_distance = quantized_distance - accel_distance - decel_distance
        if cruise_distance < 0.0:
            cruise_distance = 0.0
        cruise_time = cruise_distance / peak_speed if peak_speed > 0 else 0.0
        planned_time = accel_time + cruise_time + decel_time

        segments = []
        previous_steps = 0
        segment_time = self.profile_segment_time_s
        segment_count = int(math.ceil(planned_time / segment_time))
        if segment_count < 1:
            segment_count = 1

        for index in range(segment_count):
            t0 = min(index * segment_time, planned_time)
            t1 = min((index + 1) * segment_time, planned_time)
            if t1 <= t0:
                continue
            # Usar cinematica real para repartir pasos: pocos pasos al inicio/fin
            # y mas pasos durante crucero. La velocidad del segmento se toma en
            # el tiempo medio como aproximacion constante para ese bloque PIO.
            tm = (t0 + t1) / 2.0
            speed = self._speed_at_time(tm, peak_speed, accel_time, cruise_time)
            distance_at_t1 = self._distance_at_time(t1, peak_speed, accel_time, cruise_time)
            target_steps = int(round((distance_at_t1 / quantized_distance) * total_steps))
            if index == segment_count - 1:
                target_steps = total_steps
            segment_steps = target_steps - previous_steps
            previous_steps = target_steps
            self._append_segment(segments, segment_steps, speed)

        remaining_steps = total_steps - sum(segment["steps"] for segment in segments)
        if remaining_steps > 0:
            final_speed = segments[-1]["speed_mm_s"] if segments else self.min_speed_mm_s
            self._append_segment(segments, remaining_steps, final_speed)
        elif remaining_steps < 0 and segments:
            # Corregir cualquier exceso por redondeo desde el ultimo segmento hacia atras.
            excess = -remaining_steps
            for index in range(len(segments) - 1, -1, -1):
                removable = min(excess, segments[index]["steps"])
                segments[index]["steps"] -= removable
                excess -= removable
                if excess <= 0:
                    break
            segments = [segment for segment in segments if segment["steps"] > 0]

        return segments

    def create_constant_profile(self, distance_mm, speed_mm_s=None):
        """Crea un perfil de velocidad constante de un solo segmento."""

        total_steps = self._mm_to_steps(abs(float(distance_mm)))
        if total_steps <= 0:
            return []
        if speed_mm_s is None:
            speed = self.max_speed_mm_s
        else:
            speed = float(speed_mm_s)
        if speed <= 0:
            speed = self.min_speed_mm_s
        if speed > self.max_speed_mm_s:
            speed = self.max_speed_mm_s
        if speed < self.min_speed_mm_s:
            speed = self.min_speed_mm_s
        return [{"steps": total_steps, "speed_mm_s": speed}]

    def _start_segment(self, segment):
        steps = int(segment["steps"])
        if steps <= 0:
            return False
        speed = float(segment["speed_mm_s"])
        self._current_segment_steps = steps
        self._segment_done = False
        self.sm.active(0)
        self.sm.init(step_pulse_pio, freq=self._speed_to_pio_freq(speed), set_base=self.step_pin)
        self.sm.irq(self._on_pio_done)
        self.sm.active(1)
        self.sm.put(steps - 1)
        return True

    def _finish_move(self):
        self.sm.active(0)
        self.current_position_steps = self.target_position_steps
        self.current_position_mm = self.target_position_mm
        self._move_done = True
        self._segment_done = True
        self._current_segment_index = len(self._profile_segments)
        self._current_segment_steps = 0
        self.state = STATE_IDLE if self.is_homed_flag else STATE_NOT_HOMED

    def start_move_to_mm(self, target_mm, profile_mode=PROFILE_TRAPEZOIDAL, total_time_s=None):
        """Inicia un movimiento absoluto no bloqueante hacia ``target_mm``."""

        if self.state == STATE_MOVING or self.state == STATE_HOMING:
            return self._set_error("El eje ya esta en movimiento")
        if not self.is_homed_flag:
            return self._set_error("El eje requiere home antes de mover")
        if not self.validate_target(target_mm):
            return False

        target_steps = self._mm_to_steps(target_mm)
        delta_steps = target_steps - self.current_position_steps
        if delta_steps == 0:
            self.target_position_steps = target_steps
            self.target_position_mm = self._steps_to_mm(target_steps)
            self.current_position_steps = target_steps
            self.current_position_mm = self.target_position_mm
            self.state = STATE_IDLE
            self._move_done = True
            self._clear_error()
            return True

        distance_mm = self._steps_to_mm(abs(delta_steps))
        mode = str(profile_mode).upper()
        if mode == PROFILE_CONSTANT:
            if total_time_s is not None and float(total_time_s) > 0:
                speed = distance_mm / float(total_time_s)
            else:
                speed = None
            segments = self.create_constant_profile(distance_mm, speed)
        elif mode == PROFILE_TRAPEZOIDAL:
            segments = self.create_trapezoidal_profile(distance_mm, total_time_s)
        else:
            return self._set_error("profile_mode invalido: %s" % profile_mode)

        if not segments:
            return self._set_error("No se pudo crear perfil de movimiento")

        self._clear_error()
        self.target_position_steps = target_steps
        self.target_position_mm = self._steps_to_mm(target_steps)
        self._move_start_position_steps = self.current_position_steps
        self._move_start_position_mm = self.current_position_mm
        self._move_total_steps = abs(delta_steps)
        self._move_direction_sign = 1 if delta_steps > 0 else -1
        self._profile_segments = segments
        self._current_segment_index = 0
        self._completed_profile_steps = 0
        self._move_done = False
        self._segment_done = True
        self._active_profile_mode = mode
        self._requested_total_time_s = total_time_s
        self._planned_total_time_s = sum(
            self._steps_to_mm(segment["steps"]) / segment["speed_mm_s"] for segment in segments
        )
        self.state = STATE_MOVING

        self.enable()
        self._write_direction(self._move_direction_sign)
        return self._start_segment(self._profile_segments[0])

    def move_to_mm(self, target_mm, profile_mode=PROFILE_TRAPEZOIDAL, total_time_s=None):
        """Movimiento absoluto bloqueante respecto al cero/home."""

        if not self.start_move_to_mm(target_mm, profile_mode, total_time_s):
            return False
        self.wait_until_done()
        return self.state == STATE_IDLE

    def start_move_relative_mm(self, distance_mm, profile_mode=PROFILE_TRAPEZOIDAL, total_time_s=None):
        """Inicia un movimiento relativo no bloqueante."""

        return self.start_move_to_mm(
            self.current_position_mm + float(distance_mm), profile_mode, total_time_s
        )

    def move_relative_mm(self, distance_mm, profile_mode=PROFILE_TRAPEZOIDAL, total_time_s=None):
        """Movimiento relativo bloqueante respecto a la posicion actual."""

        if not self.start_move_relative_mm(distance_mm, profile_mode, total_time_s):
            return False
        self.wait_until_done()
        return self.state == STATE_IDLE

    def update(self):
        """Avanza segmentos cuando PIO indica fin del segmento actual.

        Este metodo no bloquea; debe llamarlo el controlador avanzado en un
        bucle. La posicion logica se consolida al terminar todo el perfil.
        """

        if self.state != STATE_MOVING:
            return False
        if not self._segment_done:
            return True

        self._completed_profile_steps += self._current_segment_steps
        next_index = self._current_segment_index + 1
        if next_index >= len(self._profile_segments):
            self._finish_move()
            return False

        self._current_segment_index = next_index
        self._start_segment(self._profile_segments[self._current_segment_index])
        return True

    def wait_until_done(self):
        """Espera hasta que el eje termine, llamando update() cada 1 ms."""

        while self.is_busy():
            self.update()
            time.sleep_ms(1)
        return self.state

    def _run_steps_blocking(self, steps, direction_sign, speed_mm_s, stop_on_home=False):
        """Ejecuta pasos bloqueantes para HOME, opcionalmente cortando por sensor."""

        total_steps = abs(int(steps))
        if total_steps <= 0:
            return True
        self.enable()
        self._write_direction(direction_sign)

        # HOME necesita poder parar apenas active el sensor; por eso se emiten
        # bloques cortos y se consulta el pin entre bloques.
        remaining = total_steps
        block_steps = max(1, int(self.steps_per_mm / 2.0))
        while remaining > 0:
            if stop_on_home and self._is_home_active():
                return True
            step_count = min(block_steps, remaining)
            self._move_done = False
            self._segment_done = False
            self.sm.active(0)
            self.sm.init(step_pulse_pio, freq=self._speed_to_pio_freq(speed_mm_s), set_base=self.step_pin)
            self.sm.irq(self._mark_internal_move_done)
            self.sm.active(1)
            self.sm.put(step_count - 1)
            while not self._move_done:
                time.sleep_ms(1)
            remaining -= step_count
        self.sm.irq(self._on_pio_done)
        return not stop_on_home or self._is_home_active()

    def _mark_internal_move_done(self, state_machine=None):
        self._move_done = True
        self._segment_done = True

    def home(self):
        """Rutina de HOME con sensor, backoff y segundo acercamiento lento."""

        if self.home_pin is None:
            return self._set_error("No hay home_pin configurado")
        if self.state == STATE_MOVING or self.state == STATE_HOMING:
            return self._set_error("El eje ya esta en movimiento")

        self._clear_error()
        self.state = STATE_HOMING
        self.is_homed_flag = False
        self.enable()

        travel_mm = self.max_position_mm - self.min_position_mm
        search_steps = self._mm_to_steps(travel_mm + self.home_backoff_mm + 5.0)
        found = self._run_steps_blocking(
            search_steps, self.home_direction, self.home_speed_mm_s, stop_on_home=True
        )
        if not found:
            self.stop()
            self.state = STATE_ERROR
            self.error = "No se encontro sensor HOME"
            return False

        backoff_steps = self._mm_to_steps(self.home_backoff_mm)
        if backoff_steps > 0:
            self._run_steps_blocking(backoff_steps, -self.home_direction, self.home_speed_mm_s)

        slow_speed = max(self.min_speed_mm_s, self.home_speed_mm_s / 3.0)
        approach_steps = self._mm_to_steps(self.home_backoff_mm * 2.0 + 5.0)
        found = self._run_steps_blocking(approach_steps, self.home_direction, slow_speed, stop_on_home=True)
        if not found:
            self.stop()
            self.state = STATE_ERROR
            self.error = "No se encontro HOME en segundo acercamiento"
            return False

        self.sm.active(0)
        self.current_position_steps = 0
        self.current_position_mm = 0.0
        self.target_position_steps = 0
        self.target_position_mm = 0.0
        self.is_homed_flag = True
        self.state = STATE_IDLE
        self._move_done = True
        self._segment_done = True
        return True

    def stop(self):
        """Detiene el movimiento y marca HOME como no confiable."""

        if self.sm is not None:
            self.sm.active(0)
        self.disable()
        self._profile_segments = []
        self._current_segment_index = -1
        self._current_segment_steps = 0
        self._move_done = True
        self._segment_done = True
        self.is_homed_flag = False
        self.state = STATE_STOPPED
        return True


class AdvancedBeltStepperAxis(AdvancedStepperAxisBase):
    """Eje lineal con polea y banda dentada."""

    def __init__(self, *args, pulley_teeth=20, belt_pitch_mm=2.0, **kwargs):
        self.pulley_teeth = int(pulley_teeth)
        self.belt_pitch_mm = float(belt_pitch_mm)
        if self.pulley_teeth <= 0:
            raise ValueError("pulley_teeth debe ser mayor que cero")
        if self.belt_pitch_mm <= 0:
            raise ValueError("belt_pitch_mm debe ser mayor que cero")
        super().__init__(*args, **kwargs)

    def calculate_steps_per_mm(self):
        return (self.motor_steps_per_rev * self.microsteps) / (
            self.pulley_teeth * self.belt_pitch_mm
        )

    def get_status(self):
        status = super().get_status()
        status["axis_type"] = "BELT"
        status["pulley_teeth"] = self.pulley_teeth
        status["belt_pitch_mm"] = self.belt_pitch_mm
        return status


class AdvancedScrewStepperAxis(AdvancedStepperAxisBase):
    """Eje lineal con tornillo/husillo."""

    def __init__(self, *args, screw_lead_mm=8.0, **kwargs):
        self.screw_lead_mm = float(screw_lead_mm)
        if self.screw_lead_mm <= 0:
            raise ValueError("screw_lead_mm debe ser mayor que cero")
        super().__init__(*args, **kwargs)

    def calculate_steps_per_mm(self):
        return (self.motor_steps_per_rev * self.microsteps) / self.screw_lead_mm

    def get_status(self):
        status = super().get_status()
        status["axis_type"] = "SCREW"
        status["screw_lead_mm"] = self.screw_lead_mm
        return status
