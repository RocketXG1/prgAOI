import array, time
from machine import Pin
import rp2

# MicroPython builds may not expose ProcessLookupError.
# Define a compatibility fallback to avoid NameError in exception paths.
try:
    ProcessLookupError
except NameError:
    class ProcessLookupError(Exception):
        pass

# PIO state machine for RGB. Pulls 24 bits (rgb -> 3 * 8bit) automatically
@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)
def ws2812():
    T1 = 2
    T2 = 5
    T3 = 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")          .side(1)    [T2 - 1]
    label("do_zero")
    nop().side(0)                       [T2 - 1]
    wrap()

# PIO state machine for RGBW. Pulls 32 bits (rgbw -> 4 * 8bit) automatically
@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=32)
def sk6812():
    T1 = 2
    T2 = 5
    T3 = 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")          .side(1)    [T2 - 1]
    label("do_zero")
    nop()                   .side(0)    [T2 - 1]
    wrap()


# Delay here is the reset time. You need a pause to reset the LED strip back to the initial LED
# however, if you have quite a bit of processing to do before the next time you update the strip
# you could put in delay=0 (or a lower delay)
#
# Class supports different order of individual colors (GRB, RGB, WRGB, GWRB ...). In order to achieve
# this, we need to flip the indexes: in 'RGBW', 'R' is on index 0, but we need to shift it left by 3 * 8bits,
# so in it's inverse, 'WBGR', it has exactly right index. Since micropython doesn't have [::-1] and recursive rev()
# isn't too efficient we simply do that by XORing (operator ^) each index with 3 (0b11) to make this flip.
# When dealing with just 'RGB' (3 letter string), this means same but reduced by 1 after XOR!.
# Example: in 'GRBW' we want final form of 0bGGRRBBWW, meaning G with index 0 needs to be shifted 3 * 8bit ->
# 'G' on index 0: 0b00 ^ 0b11 -> 0b11 (3), just as we wanted.
# Same hold for every other index (and - 1 at the end for 3 letter strings).

class Neopixel:
    def __init__(self, num_leds, state_machine, pin, mode="RGB", delay=0.0001):
        self.pixels = array.array("I", [0 for _ in range(num_leds)])
        self.mode = set(mode)   # set for better performance
        if 'W' in self.mode:
            # RGBW uses different PIO state machine configuration
            self.sm = rp2.StateMachine(state_machine, sk6812, freq=8000000, sideset_base=Pin(pin))
            # dictionary of values required to shift bit into position (check class desc.)
            self.shift = {'R': (mode.index('R') ^ 3) * 8, 'G': (mode.index('G') ^ 3) * 8,
                          'B': (mode.index('B') ^ 3) * 8, 'W': (mode.index('W') ^ 3) * 8}
        else:
            self.sm = rp2.StateMachine(state_machine, ws2812, freq=8000000, sideset_base=Pin(pin))
            self.shift = {'R': ((mode.index('R') ^ 3) - 1) * 8, 'G': ((mode.index('G') ^ 3) - 1) * 8,
                          'B': ((mode.index('B') ^ 3) - 1) * 8, 'W': 0}
        self.sm.active(1)
        self.num_leds = num_leds
        self.delay = delay
        self.brightnessvalue = 255

    # Set the overal value to adjust brightness when updating leds
    def brightness(self, brightness=None):
        if brightness == None:
            return self.brightnessvalue
        else:
            if brightness < 1:
                brightness = 1
        if brightness > 255:
            brightness = 255
        self.brightnessvalue = brightness

    # Create a gradient with two RGB colors between "pixel1" and "pixel2" (inclusive)
    # Function accepts two (r, g, b) / (r, g, b, w) tuples
    def set_pixel_line_gradient(self, pixel1, pixel2, left_rgb_w, right_rgb_w):
        if pixel2 - pixel1 == 0:
            return
        right_pixel = max(pixel1, pixel2)
        left_pixel = min(pixel1, pixel2)

        for i in range(right_pixel - left_pixel + 1):
            fraction = i / (right_pixel - left_pixel)
            red = round((right_rgb_w[0] - left_rgb_w[0]) * fraction + left_rgb_w[0])
            green = round((right_rgb_w[1] - left_rgb_w[1]) * fraction + left_rgb_w[1])
            blue = round((right_rgb_w[2] - left_rgb_w[2]) * fraction + left_rgb_w[2])
            # if it's (r, g, b, w)
            if len(left_rgb_w) == 4 and 'W' in self.mode:
                white = round((right_rgb_w[3] - left_rgb_w[3]) * fraction + left_rgb_w[3])
                self.set_pixel(left_pixel + i, (red, green, blue, white))
            else:
                self.set_pixel(left_pixel + i, (red, green, blue))

    # Set an array of pixels starting from "pixel1" to "pixel2" (inclusive) to the desired color.
    # Function accepts (r, g, b) / (r, g, b, w) tuple
    def set_pixel_line(self, pixel1, pixel2, rgb_w):
        for i in range(pixel1, pixel2 + 1):
            self.set_pixel(i, rgb_w)

    # Set a range of pixels from "pixel_start" to "pixel_end" (inclusive) to a specific color.
    # It supports both directions (start > end) and keeps values inside the strip limits.
    # Function accepts (r, g, b) / (r, g, b, w) tuple
    def set_pixel_range(self, pixel_start, pixel_end, rgb_w):
        if self.num_leds <= 0:
            return

        start = max(0, min(pixel_start, pixel_end))
        end = min(self.num_leds - 1, max(pixel_start, pixel_end))

        for i in range(start, end + 1):
            self.set_pixel(i, rgb_w)

    # Animate a gradual color change from an initial position to a final position.
    # It can run in both directions (start -> end or end -> start) and distributes
    # the total animation time over all pixel updates.
    # Function accepts (r, g, b) / (r, g, b, w) tuple
    def animate_color_range(self, pixel_start, pixel_end, rgb_w, total_time, reverse=False):
        if self.num_leds <= 0:
            return
        if total_time is None or total_time < 0:
            total_time = 0

        start = max(0, min(pixel_start, pixel_end))
        end = min(self.num_leds - 1, max(pixel_start, pixel_end))
        if start > end:
            return

        steps = end - start + 1
        if steps <= 0:
            return

        delay_per_step = total_time / steps if steps > 0 else 0
        effective_sleep = max(0, delay_per_step - self.delay)

        if reverse:
            iterator = range(end, start - 1, -1)
        else:
            iterator = range(start, end + 1)

        for i in iterator:
            self.set_pixel(i, rgb_w)
            self.show()
            if effective_sleep > 0:
                time.sleep(effective_sleep)

    # Set red, green and blue value of pixel on position <pixel_num>
    # Function accepts (r, g, b) / (r, g, b, w) tuple
    def set_pixel(self, pixel_num, rgb_w):
        if pixel_num < 0 or pixel_num >= self.num_leds:
            return
        if len(rgb_w) < 3:
            return

        pos = self.shift
        scale = self.brightnessvalue / 255

        red = min(255, max(0, round(rgb_w[0] * scale)))
        green = min(255, max(0, round(rgb_w[1] * scale)))
        blue = min(255, max(0, round(rgb_w[2] * scale)))
        white = 0
        # if it's (r, g, b, w)
        if len(rgb_w) == 4 and 'W' in self.mode:
            white = min(255, max(0, round(rgb_w[3] * scale)))

        self.pixels[pixel_num] = white << pos['W'] | blue << pos['B'] | red << pos['R'] | green << pos['G']

    # Converts HSV color to rgb tuple and returns it
    # Function accepts integer values for <hue>, <saturation> and <value>
    # The logic is almost the same as in Adafruit NeoPixel library:
    # https://github.com/adafruit/Adafruit_NeoPixel so all the credits for that
    # go directly to them (license: https://github.com/adafruit/Adafruit_NeoPixel/blob/master/COPYING)
    def colorHSV(self, hue, sat, val):
        if hue >= 65536:
            hue %= 65536

        hue = (hue * 1530 + 32768) // 65536
        if hue < 510:
            b = 0
            if hue < 255:
                r = 255
                g = hue
            else:
                r = 510 - hue
                g = 255
        elif hue < 1020:
            r = 0
            if hue < 765:
                g = 255
                b = hue - 510
            else:
                g = 1020 - hue
                b = 255
        elif hue < 1530:
            g = 0
            if hue < 1275:
                r = hue - 1020
                b = 255
            else:
                r = 255
                b = 1530 - hue
        else:
            r = 255
            g = 0
            b = 0

        v1 = 1 + val
        s1 = 1 + sat
        s2 = 255 - sat

        r = ((((r * s1) >> 8) + s2) * v1) >> 8
        g = ((((g * s1) >> 8) + s2) * v1) >> 8
        b = ((((b * s1) >> 8) + s2) * v1) >> 8

        return r, g, b


    # Rotate <num_of_pixels> pixels to the left
    def rotate_left(self, num_of_pixels):
        if num_of_pixels == None:
            num_of_pixels = 1
        self.pixels = self.pixels[num_of_pixels:] + self.pixels[:num_of_pixels]

    # Rotate <num_of_pixels> pixels to the right
    def rotate_right(self, num_of_pixels):
        if num_of_pixels == None:
            num_of_pixels = 1
        num_of_pixels = -1 * num_of_pixels
        self.pixels = self.pixels[num_of_pixels:] + self.pixels[:num_of_pixels]

    # Update pixels
    def show(self):
        # If mode is RGB, we cut 8 bits of, otherwise we keep all 32
        cut = 8
        if 'W' in self.mode:
            cut = 0
        for i in range(self.num_leds):
            self.sm.put(self.pixels[i], cut)
        time.sleep(self.delay)

    # Set all pixels to given rgb values
    # Function accepts (r, g, b) / (r, g, b, w)
    def fill(self, rgb_w):
        for i in range(self.num_leds):
            self.set_pixel(i, rgb_w)
        time.sleep(self.delay)

    def circular_bounce_fill(self, color=(255, 0, 0), Startdelay=0.05,FinishDelay=0.05, brightness=50):
        numpix = self.num_leds
        self.brightness(brightness)
        off = (0, 0, 0, 0) if 'W' in self.mode else (0, 0, 0)
        self.fill(off)
        self.show()

        left = 0
        right = numpix - 1
        previous = None
        while left <= right:
            self.set_pixel(left, color)
            self.set_pixel(right, color)
            if previous is not None:
                prev_left, prev_right = previous
                self.set_pixel(prev_left, off)
                self.set_pixel(prev_right, off)
            self.show()
            time.sleep(Startdelay)
            previous = (left, right)
            left += 1
            right -= 1

        left = max(left - 1, 0)
        right = min(right + 1, numpix - 1)
        while left > 0 or right < numpix - 1:
            if left > 0:
                left -= 1
                self.set_pixel(left, color)
            if right < numpix - 1:
                right += 1
                self.set_pixel(right, color)
            self.show()
            time.sleep(FinishDelay)


# Cascaded progressive animation by sections.
    # Each section advances one LED per tick. When section i reaches its
    # trigger position, section i+1 starts (while previous keeps advancing).
    def SecuencialCascade(self, data_pin, total_leds, sections_count, leds_per_section,
                         section_colors, step_ms=40, trigger_position=40,
                         trail=True, repeat=False, state_machine=0, brightness=255):
        # Validate and normalize numeric inputs.
        try:
            total_leds = int(total_leds)
            sections_count = int(sections_count)
            step_ms = max(1, int(step_ms))
        except:
            return

        if total_leds <= 0 or sections_count <= 0 or not section_colors:
            return

        def _build_section_sizes(leds_cfg, count):
            try:
                if isinstance(leds_cfg, int):
                    return [max(0, int(leds_cfg)) for _ in range(count)]
                sizes = [max(0, int(v)) for v in list(leds_cfg)[:count]]
                if len(sizes) < count:
                    return None
                return sizes
            except:
                return None

        def _build_sections(total, count, sizes, colors):
            sections_local = []
            cursor_local = 0
            for idx in range(count):
                if cursor_local >= total:
                    break
                size = min(sizes[idx], total - cursor_local)
                if size <= 0:
                    continue
                sections_local.append({
                    "start": cursor_local,
                    "size": size,
                    "color": colors[idx % len(colors)],
                    "pos": -1,
                    "active": False,
                    "done": False,
                    "last_abs": None,
                })
                cursor_local += size
            return sections_local

        def _build_trigger_points(trigger_cfg, transitions):
            try:
                if isinstance(trigger_cfg, (list, tuple)):
                    return [max(0, int(trigger_cfg[i])) if i < len(trigger_cfg) else 0
                            for i in range(transitions)]
                tp = max(0, int(trigger_cfg))
                return [tp for _ in range(transitions)]
            except:
                return None

        def _reset_cycle(sections_local, off_color):
            self.fill(off_color)
            for sec in sections_local:
                sec["pos"] = -1
                sec["active"] = False
                sec["done"] = False
                sec["last_abs"] = None
            sections_local[0]["active"] = True
            self.show()

        def _reverse_off(sections_local, off_color, step_delay_ms):
            tails = [sec["size"] - 1 for sec in sections_local]
            while True:
                any_update = False
                for idx, sec in enumerate(sections_local):
                    if tails[idx] < 0:
                        continue
                    abs_i = sec["start"] + tails[idx]
                    self.set_pixel(abs_i, off_color)
                    tails[idx] -= 1
                    any_update = True
                self.show()
                if not any_update:
                    return
                wait_until = _ticks_add_int(_ticks_ms_int(), step_delay_ms)
                while True:
                    now_tick = _ticks_ms_int()
                    if now_tick is None:
                        return
                    if _ticks_due(now_tick, wait_until):
                        break

        section_sizes = _build_section_sizes(leds_per_section, sections_count)
        if not section_sizes:
            return

        sections = _build_sections(total_leds, sections_count, section_sizes, section_colors)
        if not sections:
            return

        mode = "RGBW" if 'W' in self.mode else "RGB"
        self.__init__(total_leds, state_machine, data_pin, mode=mode, delay=self.delay)
        try:
            self.brightness(int(brightness))
        except:
            self.brightness(255)

        transitions = len(sections) - 1
        trigger_points = _build_trigger_points(trigger_position, transitions)
        if trigger_points is None:
            return

        off = (0, 0, 0, 0) if 'W' in self.mode else (0, 0, 0)
        _reset_cycle(sections, off)

        def _ticks_ms_int():
            try:
                return int(time.ticks_ms())
            except:
                return None

        def _ticks_add_int(base, delta):
            try:
                return int(time.ticks_add(int(base), int(delta)))
            except:
                return int(base) + int(delta)

        def _ticks_due(now_tick, next_tick_ref):
            try:
                return time.ticks_diff(int(now_tick), int(next_tick_ref)) >= 0
            except:
                return int(now_tick) >= int(next_tick_ref)

        next_tick = _ticks_ms_int()
        if next_tick is None:
            return

        while True:
            now = _ticks_ms_int()
            if now is None:
                return
            if not _ticks_due(now, next_tick):
                continue
            next_tick = _ticks_add_int(next_tick, step_ms)

            updated = False
            for idx, sec in enumerate(sections):
                if (not sec["active"]) or sec["done"]:
                    continue

                next_pos = sec["pos"] + 1
                if next_pos >= sec["size"]:
                    sec["active"] = False
                    sec["done"] = True
                    # Fallback chaining: if trigger was never reached, start next at completion.
                    if idx < transitions:
                        nxt = sections[idx + 1]
                        if (not nxt["active"]) and (not nxt["done"]):
                            nxt["active"] = True
                    continue

                if (not trail) and (sec["last_abs"] is not None):
                    self.set_pixel(sec["last_abs"], off)

                abs_i = sec["start"] + next_pos
                self.set_pixel(abs_i, sec["color"])
                sec["last_abs"] = abs_i
                sec["pos"] = next_pos
                updated = True

                # Trigger next section once current reaches its relative trigger position.
                if idx < transitions and sec["pos"] >= trigger_points[idx]:
                    nxt = sections[idx + 1]
                    if (not nxt["active"]) and (not nxt["done"]):
                        nxt["active"] = True

            # Single global write per tick.
            self.show()

            if not updated:
                # When all sections finished, run reverse shutdown (tail -> head).
                _reverse_off(sections, off, step_ms)
                if not repeat:
                    return
                _reset_cycle(sections, off)
                next_tick = _ticks_ms_int()
                if next_tick is None:
                    return



    # Two-phase section color gradient with global brightness ramp.
    def _GradientTransition(self, section_colors_now, section_colors_mid, section_colors_final,
                            leds_per_section, section_count, total_leds,
                            step_ms=16, start_brightness=40, max_brightness=255,
                            repeat=False, phase_steps=120):
        if total_leds <= 0 or section_count <= 0:
            return
        if not section_colors_now or not section_colors_mid or not section_colors_final:
            return

        if isinstance(leds_per_section, int):
            section_sizes = [max(0, int(leds_per_section)) for _ in range(section_count)]
        else:
            section_sizes = [max(0, int(v)) for v in list(leds_per_section)[:section_count]]
            if len(section_sizes) < section_count:
                return

        start_brightness = max(1, min(255, int(start_brightness)))

        sections = []
        cursor = 0
        for i in range(section_count):
            if cursor >= total_leds:
                break
            size = min(section_sizes[i], total_leds - cursor)
            if size > 0:
                sections.append((cursor, size))
            cursor += max(0, section_sizes[i])

        if not sections:
            return

        def lerp(c1, c2, t):
            channels = min(len(c1), len(c2))
            return tuple(int(round(c1[k] + (c2[k] - c1[k]) * t)) for k in range(channels))

        phase_steps = max(2, int(phase_steps))

        def paint_phase(from_colors, to_colors, next_tick_ref):
            for step in range(phase_steps + 1):
                while True:
                    now = time.ticks_ms()
                    if time.ticks_diff(now, next_tick_ref[0]) >= 0:
                        next_tick_ref[0] = time.ticks_add(next_tick_ref[0], max(1, int(step_ms)))
                        break
                self.brightness(start_brightness)
                t = step / phase_steps
                for idx, (start, size) in enumerate(sections):
                    c_from = from_colors[idx % len(from_colors)]
                    c_to = to_colors[idx % len(to_colors)]
                    c = lerp(c_from, c_to, t)
                    self.set_pixel_range(start, start + size - 1, c)
                self.show()

        while True:
            next_tick = [time.ticks_ms()]

            # Forward: now -> mid -> final
            paint_phase(section_colors_now, section_colors_mid, next_tick)
            paint_phase(section_colors_mid, section_colors_final, next_tick)

            # Return: final -> mid -> now
            paint_phase(section_colors_final, section_colors_mid, next_tick)
            paint_phase(section_colors_mid, section_colors_now, next_tick)

            if not repeat:
                return
    