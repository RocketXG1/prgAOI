"""Control orientado a objetos de un eje lineal con motor a pasos y PIO.

Modulo pensado para Raspberry Pi Pico/RP2040 ejecutando MicroPython.  Cada
instancia controla un solo eje lineal mediante un driver externo de motor a
pasos (TB6600, TMC2209, DRV8825 o similar) con senales STEP/PUL, DIR y ENABLE.

La generacion de pulsos STEP se delega a una StateMachine PIO distinta por eje.
Esta version trabaja con velocidad fija: no implementa aceleracion, perfil
trapezoidal ni movimiento coordinado entre varios ejes.
"""

from machine import Pin
import rp2
import time


# Estados publicos del eje.
STATE_NOT_HOMED = "NOT_HOMED"
STATE_IDLE = "IDLE"
STATE_MOVING = "MOVING"
STATE_HOMING = "HOMING"
STATE_ERROR = "ERROR"
STATE_STOPPED = "STOPPED"


# El programa PIO ejecuta estas instrucciones por cada pulso STEP:
#   set alto, nop, set bajo, jmp condicional.
# Por tanto la frecuencia de la StateMachine debe ser aproximadamente:
#   pasos_por_segundo * PIO_CYCLES_PER_STEP
PIO_CYCLES_PER_STEP = 4


@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def _step_pulse_program():
    """Genera N pulsos STEP recibidos por FIFO.

    Se debe enviar ``steps - 1`` al FIFO. Con ``jmp(x_dec, ...)`` el salto se
    evalua usando el valor anterior de X, de modo que X=0 produce exactamente un
    pulso y X=N-1 produce N pulsos.
    """

    pull(block)
    mov(x, osr)
    label("pulse_loop")
    set(pins, 1)
    nop()
    set(pins, 0)
    jmp(x_dec, "pulse_loop")
    irq(rel(0))


class StepperAxisBase:
    """Clase base para controlar un eje lineal con STEP/DIR/ENABLE y PIO.

    Las clases hijas solo calculan ``steps_per_mm`` segun la transmision
    mecanica. Esta clase contiene validacion, posicion, homing, movimientos
    absolutos/relativos y administracion de la StateMachine PIO.
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
        speed_mm_s=10.0,
        max_speed_mm_s=100.0,
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
        self.speed_mm_s = float(speed_mm_s)
        self.max_speed_mm_s = float(max_speed_mm_s)
        self.min_position_mm = float(min_position_mm)
        self.max_position_mm = float(max_position_mm)
        self.home_direction = -1 if int(home_direction) < 0 else 1
        self.home_speed_mm_s = float(home_speed_mm_s)
        self.home_backoff_mm = abs(float(home_backoff_mm))
        self.invert_direction = bool(invert_direction)
        self.enable_active_low = bool(enable_active_low)
        self.home_active_low = bool(home_active_low)

        self.steps_per_mm = float(self._calculate_steps_per_mm())
        if self.steps_per_mm <= 0:
            raise ValueError("steps_per_mm debe ser mayor que cero")
        if self.motor_steps_per_rev <= 0:
            raise ValueError("motor_steps_per_rev debe ser mayor que cero")
        if self.microsteps <= 0:
            raise ValueError("microsteps debe ser mayor que cero")
        if self.max_speed_mm_s <= 0:
            raise ValueError("max_speed_mm_s debe ser mayor que cero")
        if self.speed_mm_s <= 0 or self.speed_mm_s > self.max_speed_mm_s:
            raise ValueError("speed_mm_s debe ser > 0 y <= max_speed_mm_s")
        if self.home_speed_mm_s <= 0 or self.home_speed_mm_s > self.max_speed_mm_s:
            raise ValueError("home_speed_mm_s debe ser > 0 y <= max_speed_mm_s")
        if self.min_position_mm >= self.max_position_mm:
            raise ValueError("min_position_mm debe ser menor que max_position_mm")

        self.step_pin = Pin(self.step_pin_number, Pin.OUT)
        self.dir_pin = Pin(self.dir_pin_number, Pin.OUT)
        self.enable_pin = Pin(self.enable_pin_number, Pin.OUT)
        self.home_pin = None
        if home_pin is not None:
            self.home_pin_number = int(home_pin)
            self.home_pin = Pin(self.home_pin_number, Pin.IN, Pin.PULL_UP)
        else:
            self.home_pin_number = None

        self.current_position_mm = 0.0
        self.current_position_steps = 0
        self.target_position_mm = None
        self.target_position_steps = None
        self._move_start_position_mm = 0.0
        self._move_delta_steps = 0
        self._move_direction_sign = 1
        self._move_done = True
        self._enabled = False
        self.error = None
        self.is_homed_flag = False
        self.state = STATE_NOT_HOMED

        self.step_pin.value(0)
        self.disable()

        self.sm = rp2.StateMachine(
            self.sm_id,
            _step_pulse_program,
            freq=self._speed_to_pio_freq(self.speed_mm_s),
            set_base=self.step_pin,
        )
        self.sm.irq(self._on_pio_done)
        self.sm.active(0)

    def _calculate_steps_per_mm(self):
        """Debe ser implementado por las clases hijas."""

        raise NotImplementedError

    def _on_pio_done(self, state_machine):
        """Callback PIO: marca terminado y consolida la posicion final."""

        self._move_done = True
        if self.state == STATE_MOVING:
            self.current_position_steps = self.target_position_steps
            self.current_position_mm = self.target_position_mm
            self.state = STATE_IDLE if self.is_homed_flag else STATE_NOT_HOMED

    def _set_error(self, message):
        """Registra error y deja el eje en estado ERROR."""

        self.error = str(message)
        self.state = STATE_ERROR
        self._move_done = True
        return False

    def _clear_error(self):
        self.error = None

    def _speed_to_pio_freq(self, speed_mm_s):
        """Convierte velocidad lineal en frecuencia de StateMachine PIO."""

        steps_per_second = abs(float(speed_mm_s)) * self.steps_per_mm
        freq = int(steps_per_second * PIO_CYCLES_PER_STEP)
        if freq < PIO_CYCLES_PER_STEP:
            freq = PIO_CYCLES_PER_STEP
        return freq

    def _mm_to_steps(self, mm_value):
        """Convierte milimetros a pasos redondeando al paso mas cercano."""

        return int(round(float(mm_value) * self.steps_per_mm))

    def _steps_to_mm(self, steps):
        """Convierte pasos a milimetros."""

        return float(steps) / self.steps_per_mm

    def _validate_target(self, target_mm):
        """Valida que el destino este dentro de limites del eje."""

        target = float(target_mm)
        if target < self.min_position_mm or target > self.max_position_mm:
            return self._set_error(
                "Destino fuera de limites: %.3f mm no esta entre %.3f y %.3f mm"
                % (target, self.min_position_mm, self.max_position_mm)
            )
        return True

    def _write_direction(self, direction_sign):
        """Escribe el pin DIR aplicando inversion logica si fue solicitada."""

        logical_positive = int(direction_sign) >= 0
        pin_value = 1 if logical_positive else 0
        if self.invert_direction:
            pin_value = 0 if pin_value else 1
        self.dir_pin.value(pin_value)

    def enable(self):
        """Habilita el driver externo."""

        self.enable_pin.value(0 if self.enable_active_low else 1)
        self._enabled = True

    def disable(self):
        """Deshabilita el driver externo."""

        self.enable_pin.value(1 if self.enable_active_low else 0)
        self._enabled = False

    def is_home_active(self):
        """Devuelve True si el sensor HOME esta activo."""

        if self.home_pin is None:
            return False
        raw_value = self.home_pin.value()
        if self.home_active_low:
            return raw_value == 0
        return raw_value == 1

    def get_position(self):
        """Devuelve la posicion actual del eje en milimetros."""

        return self.current_position_mm

    def get_status(self):
        """Devuelve un diccionario con el estado observable del eje."""

        return {
            "name": self.name,
            "state": self.state,
            "error": self.error,
            "position_mm": self.current_position_mm,
            "position_steps": self.current_position_steps,
            "target_position_mm": self.target_position_mm,
            "target_position_steps": self.target_position_steps,
            "homed": self.is_homed_flag,
            "enabled": self._enabled,
            "steps_per_mm": self.steps_per_mm,
            "speed_mm_s": self.speed_mm_s,
            "max_speed_mm_s": self.max_speed_mm_s,
            "min_position_mm": self.min_position_mm,
            "max_position_mm": self.max_position_mm,
            "moving": self.state == STATE_MOVING,
            "pio_freq": self._speed_to_pio_freq(self.speed_mm_s),
            "pio_cycles_per_step": PIO_CYCLES_PER_STEP,
        }

    def set_speed_mm_s(self, speed_mm_s):
        """Actualiza la velocidad fija normal del eje."""

        speed = float(speed_mm_s)
        if speed <= 0 or speed > self.max_speed_mm_s:
            self._set_error("Velocidad fuera de rango")
            return False
        if self.state == STATE_MOVING or self.state == STATE_HOMING:
            self._set_error("No se puede cambiar velocidad durante movimiento")
            return False
        self.speed_mm_s = speed
        return True

    def start_move_to_mm(self, target_mm):
        """Inicia un movimiento absoluto no bloqueante hacia ``target_mm``."""

        if self.state == STATE_MOVING or self.state == STATE_HOMING:
            return self._set_error("El eje ya esta en movimiento")
        if not self.is_homed_flag:
            return self._set_error("El eje requiere home antes de mover")
        if not self._validate_target(target_mm):
            return False

        target_steps = self._mm_to_steps(target_mm)
        delta_steps = target_steps - self.current_position_steps
        if delta_steps == 0:
            self.target_position_mm = self._steps_to_mm(target_steps)
            self.target_position_steps = target_steps
            self.current_position_mm = self.target_position_mm
            self.current_position_steps = target_steps
            self.state = STATE_IDLE
            self._move_done = True
            self._clear_error()
            return True

        self._clear_error()
        self._move_start_position_mm = self.current_position_mm
        self._move_delta_steps = abs(delta_steps)
        self._move_direction_sign = 1 if delta_steps > 0 else -1
        self.target_position_steps = target_steps
        self.target_position_mm = self._steps_to_mm(target_steps)
        self._move_done = False
        self.state = STATE_MOVING

        self.enable()
        self._write_direction(self._move_direction_sign)
        self.sm.active(0)
        self.sm.init(
            _step_pulse_program,
            freq=self._speed_to_pio_freq(self.speed_mm_s),
            set_base=self.step_pin,
        )
        self.sm.irq(self._on_pio_done)
        self.sm.active(1)
        self.sm.put(self._move_delta_steps - 1)
        return True

    def move_to_mm(self, target_mm):
        """Movimiento absoluto bloqueante respecto al cero/home."""

        if not self.start_move_to_mm(target_mm):
            return False
        self.wait_until_done()
        return self.state == STATE_IDLE

    def start_move_relative_mm(self, distance_mm):
        """Inicia un movimiento relativo no bloqueante."""

        return self.start_move_to_mm(self.current_position_mm + float(distance_mm))

    def move_relative_mm(self, distance_mm):
        """Movimiento relativo bloqueante respecto a la posicion actual."""

        if not self.start_move_relative_mm(distance_mm):
            return False
        self.wait_until_done()
        return self.state == STATE_IDLE

    def wait_until_done(self):
        """Espera hasta que termine el movimiento PIO actual."""

        while self.state == STATE_MOVING and not self._move_done:
            time.sleep_ms(1)
        return self.state

    def _run_steps_blocking(self, steps, direction_sign, speed_mm_s):
        """Ejecuta pasos sin modificar posicion logica; usado para homing."""

        total_steps = abs(int(steps))
        if total_steps <= 0:
            return True
        self._move_done = False
        self.enable()
        self._write_direction(direction_sign)
        self.sm.active(0)
        self.sm.init(
            _step_pulse_program,
            freq=self._speed_to_pio_freq(speed_mm_s),
            set_base=self.step_pin,
        )
        self.sm.irq(lambda sm: self._mark_internal_move_done())
        self.sm.active(1)
        self.sm.put(total_steps - 1)
        while not self._move_done:
            time.sleep_ms(1)
        self.sm.irq(self._on_pio_done)
        return True

    def _mark_internal_move_done(self):
        self._move_done = True

    def home(self):
        """Busca HOME, retrocede y vuelve a buscar lentamente.

        El sensor se busca en ``home_direction``. Despues de la primera
        activacion, el eje se retira ``home_backoff_mm`` en sentido contrario y
        realiza una segunda aproximacion mas lenta para mejorar repetibilidad.
        Al finalizar, la posicion logica queda en 0 mm.
        """

        if self.home_pin is None:
            return self._set_error("No hay home_pin configurado")
        if self.state == STATE_MOVING:
            return self._set_error("No se puede hacer home durante movimiento")

        self._clear_error()
        self.state = STATE_HOMING
        self.is_homed_flag = False
        self.enable()

        # Si ya esta tocando el sensor, primero se retira para liberarlo.
        backoff_steps = self._mm_to_steps(self.home_backoff_mm)
        if self.is_home_active() and backoff_steps > 0:
            self._run_steps_blocking(backoff_steps, -self.home_direction, self.home_speed_mm_s)

        # Busqueda rapida: como esta version no tiene interrupcion por pin HOME
        # dentro del PIO, se avanza en pequenos bloques PIO y se consulta HOME
        # entre bloques. No se generan pulsos con sleep_us.
        travel_mm = self.max_position_mm - self.min_position_mm + self.home_backoff_mm
        max_search_steps = self._mm_to_steps(travel_mm)
        chunk_steps = max(1, self._mm_to_steps(0.25))
        searched_steps = 0
        while not self.is_home_active() and searched_steps < max_search_steps:
            steps_now = min(chunk_steps, max_search_steps - searched_steps)
            self._run_steps_blocking(steps_now, self.home_direction, self.home_speed_mm_s)
            searched_steps += steps_now

        if not self.is_home_active():
            self.is_homed_flag = False
            return self._set_error("HOME no detectado dentro del recorrido permitido")

        # Retirada del sensor.
        if backoff_steps > 0:
            self._run_steps_blocking(backoff_steps, -self.home_direction, self.home_speed_mm_s)

        # Segunda aproximacion lenta en bloques mas pequenos.
        slow_speed = self.home_speed_mm_s / 2.0
        if slow_speed <= 0:
            slow_speed = self.home_speed_mm_s
        slow_chunk_steps = max(1, self._mm_to_steps(0.05))
        searched_steps = 0
        while not self.is_home_active() and searched_steps < backoff_steps + chunk_steps:
            steps_now = min(slow_chunk_steps, backoff_steps + chunk_steps - searched_steps)
            self._run_steps_blocking(steps_now, self.home_direction, slow_speed)
            searched_steps += steps_now

        if not self.is_home_active():
            self.is_homed_flag = False
            return self._set_error("HOME no detectado en segunda aproximacion")

        self.current_position_mm = 0.0
        self.current_position_steps = 0
        self.target_position_mm = 0.0
        self.target_position_steps = 0
        self.is_homed_flag = True
        self.state = STATE_IDLE
        self._move_done = True
        return True

    def stop(self):
        """Parada local: detiene PIO, deshabilita eje e invalida el home."""

        self.sm.active(0)
        self.disable()
        self._move_done = True
        self.is_homed_flag = False
        self.state = STATE_STOPPED
        self.error = "Eje detenido; se requiere home nuevamente"
        return True


class BeltStepperAxis(StepperAxisBase):
    """Eje lineal con polea y banda."""

    def __init__(self, pulley_teeth, belt_pitch_mm, *args, **kwargs):
        self.pulley_teeth = int(pulley_teeth)
        self.belt_pitch_mm = float(belt_pitch_mm)
        if self.pulley_teeth <= 0:
            raise ValueError("pulley_teeth debe ser mayor que cero")
        if self.belt_pitch_mm <= 0:
            raise ValueError("belt_pitch_mm debe ser mayor que cero")
        StepperAxisBase.__init__(self, *args, **kwargs)

    def _calculate_steps_per_mm(self):
        return (
            float(self.motor_steps_per_rev) * float(self.microsteps)
        ) / (float(self.pulley_teeth) * float(self.belt_pitch_mm))


class ScrewStepperAxis(StepperAxisBase):
    """Eje lineal con tornillo/husillo."""

    def __init__(self, screw_lead_mm, *args, **kwargs):
        self.screw_lead_mm = float(screw_lead_mm)
        if self.screw_lead_mm <= 0:
            raise ValueError("screw_lead_mm debe ser mayor que cero")
        StepperAxisBase.__init__(self, *args, **kwargs)

    def _calculate_steps_per_mm(self):
        return (
            float(self.motor_steps_per_rev) * float(self.microsteps)
        ) / float(self.screw_lead_mm)
