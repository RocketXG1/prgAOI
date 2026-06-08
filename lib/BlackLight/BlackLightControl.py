from machine import Pin, PWM
import time


class BlackLightControl:
    def __init__(self, pin, frequency=1000):
        self._pwm = PWM(Pin(pin))
        self._pwm.freq(frequency)
        self._percent = 0
        self.set_percent(0)

    def set_percent(self, percent):
        percent = max(0, min(100, int(percent)))
        self._percent = percent
        duty = int((percent / 100) * 65535)
        self._pwm.duty_u16(duty)

    def ramp_percent(self, step_percent, total_time_s, start_percent=0, end_percent=100):
        step_percent = abs(int(step_percent))
        if step_percent < 1:
            step_percent = 1
        start_percent = max(0, min(100, int(start_percent)))
        end_percent = max(0, min(100, int(end_percent)))

        if start_percent == end_percent:
            self.set_percent(end_percent)
            return

        direction = 1 if end_percent > start_percent else -1
        step_percent *= direction

        steps = int((end_percent - start_percent) / step_percent)
        if steps == 0:
            self.set_percent(end_percent)
            return

        delay_s = total_time_s / abs(steps)
        current = start_percent
        for _ in range(abs(steps)):
            self.set_percent(current)
            time.sleep(delay_s)
            current += step_percent

        self.set_percent(end_percent)

    def deinit(self):
        self._pwm.deinit()
