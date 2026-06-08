try:
    import os
except ImportError:
    os = None


VALID_DAYS = (
    "Lunes",
    "Martes",
    "Miércoles",
    "Jueves",
    "Viernes",
    "Sábado",
    "Domingo",
)

VALID_DOW_CHARS = ("0", "1")

class RecipeTimerEvent:
    """Evento compacto para MicroPython."""

    __slots__ = ("event_id", "dow", "time_text", "out1", "out2", "out3")

    def __init__(self, event_id, dow, time_text, out1, out2, out3):
        self.event_id = validate_event_id(event_id)
        self.dow = validate_dow(dow)
        self.time_text = normalize_time(time_text)
        self.out1 = validate_output(out1, "out1")
        self.out2 = validate_output(out2, "out2")
        self.out3 = validate_output(out3, "out3")

    def same_schedule(self, dow, time_text):
        normalized_dow = validate_dow(dow)
        return (
            self.time_text == time_text
            and has_shared_days(self.dow, normalized_dow)
        )

    def write_to_file(self, file):
        file.write("[event")
        file.write(str(self.event_id))
        file.write("]\n")
        file.write("dow=")
        file.write(self.dow)
        file.write("\n")
        file.write("hora=")
        file.write(self.time_text)
        file.write("\n")
        file.write("out1=")
        file.write(str(self.out1))
        file.write("\n")
        file.write("out2=")
        file.write(str(self.out2))
        file.write("\n")
        file.write("out3=")
        file.write(str(self.out3))
        file.write("\n")


def validate_event_id(event_id):
    value = int(event_id)
    if value < 1 or value > 50:
        raise ValueError("El identificador del evento debe estar entre 1 y 50.")
    return value



def validate_dow(dow):
    day_text = str(dow).strip()
    if day_text in VALID_DAYS:
        return day_name_to_mask(day_text)
    if "," in day_text:
        return day_names_to_mask(day_text)
    return validate_dow_mask(day_text)


def day_name_to_mask(day_name):
    index = VALID_DAYS.index(day_name)
    chars = ["0", "0", "0", "0", "0", "0", "0"]
    chars[index] = "1"
    return "".join(chars)


def day_names_to_mask(day_names_text):
    chars = ["0", "0", "0", "0", "0", "0", "0"]
    names = day_names_text.split(",")
    for name in names:
        clean_name = name.strip()
        if clean_name not in VALID_DAYS:
            raise ValueError(
                "Dia invalido '%s'. Use nombres validos: %s."
                % (clean_name, ",".join(VALID_DAYS))
            )
        chars[VALID_DAYS.index(clean_name)] = "1"
    return validate_dow_mask("".join(chars))


def validate_dow_mask(dow_mask):
    if len(dow_mask) != 7:
        raise ValueError("El DoW debe tener 7 posiciones (formato 0000000).")

    enabled_days = 0
    for char in dow_mask:
        if char not in VALID_DOW_CHARS:
            raise ValueError("El DoW solo permite 0 o 1.")
        if char == "1":
            enabled_days += 1

    if enabled_days == 0:
        raise ValueError("El DoW no puede tener todos los dias en 0.")
    if enabled_days > 7:
        raise ValueError("El DoW debe tener entre 1 y 7 dias activos.")

    return dow_mask


def has_shared_days(dow_a, dow_b):
    index = 0
    while index < 7:
        if dow_a[index] == "1" and dow_b[index] == "1":
            return True
        index += 1
    return False


def weekday_to_mask(weekday):
    if weekday < 0 or weekday > 6:
        raise ValueError("El valor de dia de la semana del RTC no es valido.")
    chars = ["0", "0", "0", "0", "0", "0", "0"]
    chars[weekday] = "1"
    return "".join(chars)


def format_dow_for_display(dow):
    days = []
    index = 0
    while index < 7:
        if dow[index] == "1":
            days.append(VALID_DAYS[index])
        index += 1
    return ",".join(days)



def normalize_time(time_text):
    text = str(time_text).strip()
    parts = text.split(":")
    if len(parts) != 3:
        raise ValueError("La hora debe tener el formato HH:MM:SS.")

    hours = int(parts[0])
    minutes = int(parts[1])
    seconds = int(parts[2])

    if hours < 0 or hours > 23:
        raise ValueError("La hora debe estar entre 00 y 23.")
    if minutes < 0 or minutes > 59:
        raise ValueError("Los minutos deben estar entre 00 y 59.")
    if seconds < 0 or seconds > 59:
        raise ValueError("Los segundos deben estar entre 00 y 59.")

    return "%02d:%02d:%02d" % (hours, minutes, seconds)



def validate_output(value, label):
    output = int(value)
    if output != 0 and output != 1:
        raise ValueError("%s debe ser 0 o 1." % label)
    return output


class RecipeTimerStore:
    """Almacen simple de eventos optimizado para Pico/MicroPython."""

    def __init__(self, filename="recipe_timer.ini", base_path="/RecipeTime"):
        filename = str(filename).strip()
        if not filename:
            raise ValueError("El nombre del archivo no puede estar vacio.")

        self.base_path = base_path.rstrip("/") or "/RecipeTime"
        self.filename = filename
        self.path = self.base_path + "/" + self.filename
        self.events = {}
        self.load()

    def _ensure_base_path(self):
        if os is None:
            return
        try:
            os.mkdir(self.base_path)
        except OSError:
            pass

    def load(self):
        self.events = {}
        current_id = 0
        current_data = None

        try:
            file = open(self.path, "r")
        except OSError:
            return self.events

        try:
            for raw_line in file:
                line = raw_line.strip()
                if not line:
                    continue

                if line.startswith("[event") and line.endswith("]"):
                    if current_data is not None:
                        self._store_loaded_event(current_id, current_data)
                    current_id = self._parse_section_id(line)
                    current_data = {}
                    continue

                if current_data is None:
                    continue

                pos = line.find("=")
                if pos <= 0:
                    continue
                key = line[:pos].strip()
                value = line[pos + 1 :].strip()
                current_data[key] = value

            if current_data is not None:
                self._store_loaded_event(current_id, current_data)
        finally:
            file.close()

        return self.events

    def _parse_section_id(self, line):
        return int(line[6:-1])

    def _store_loaded_event(self, event_id, data):
        try:
            event = RecipeTimerEvent(
                event_id,
                data.get("dow", ""),
                data.get("hora", ""),
                data.get("out1", 0),
                data.get("out2", 0),
                data.get("out3", 0),
            )
        except ValueError:
            return
        self.events[event.event_id] = event

    def save(self):
        self._ensure_base_path()
        file = open(self.path, "w")
        try:
            event_ids = list(self.events.keys())
            event_ids.sort()
            total = len(event_ids)
            index = 0
            while index < total:
                event = self.events[event_ids[index]]
                event.write_to_file(file)
                if index != total - 1:
                    file.write("\n")
                index += 1
        finally:
            file.close()
        return self.path

    def list_events(self):
        event_ids = list(self.events.keys())
        event_ids.sort()
        items = []
        for event_id in event_ids:
            items.append(self.events[event_id])
        return items

    def get_event(self, event_id):
        return self.events.get(int(event_id))

    def find_duplicate_schedule(self, dow, time_text, ignore_event_id=None):
        normalized_dow = validate_dow(dow)
        normalized_time = normalize_time(time_text)
        for event_id in self.events:
            if ignore_event_id is not None and event_id == ignore_event_id:
                continue
            event = self.events[event_id]
            if event.same_schedule(normalized_dow, normalized_time):
                return event
        return None

    def add_event(self, event_id, dow, time_text, out1, out2, out3, overwrite=False):
        new_event = RecipeTimerEvent(event_id, dow, time_text, out1, out2, out3)

        duplicate = self.find_duplicate_schedule(
            new_event.dow, new_event.time_text, new_event.event_id
        )
        if duplicate is not None:
            raise ValueError(
                "Ya existe un evento en [%s] a las %s (evento %s)."
                % (
                    format_dow_for_display(duplicate.dow),
                    duplicate.time_text,
                    duplicate.event_id,
                )
            )

        if (new_event.event_id in self.events) and (not overwrite):
            raise ValueError(
                "El evento %s ya existe. Use overwrite=True para sobrescribir o aborte la operacion."
                % new_event.event_id
            )

        self.events[new_event.event_id] = new_event
        self.save()
        return new_event

    def update_event(self, event_id, dow, time_text, out1, out2, out3):
        return self.add_event(event_id, dow, time_text, out1, out2, out3, True)

    def delete_event(self, event_id):
        event_id = int(event_id)
        if event_id not in self.events:
            return None
        deleted = self.events[event_id]
        del self.events[event_id]
        self.save()
        return deleted

    def clear(self):
        self.events = {}
        self.save()


class RecipeTimerController:
    """Control de coincidencia entre RTC y eventos."""

    __slots__ = ("store", "rtc", "output_handler", "_last_trigger_dow", "_last_trigger_time")

    def __init__(self, store, rtc, output_handler=None):
        self.store = store
        self.rtc = rtc
        if output_handler is None:
            output_handler = default_output_handler
        self.output_handler = output_handler
        self._last_trigger_dow = None
        self._last_trigger_time = None

    def get_current_schedule(self):
        rtc_tuple = self.rtc.datetime()
        weekday = rtc_tuple[3]
        return weekday_to_mask(weekday), "%02d:%02d:%02d" % (
            rtc_tuple[4],
            rtc_tuple[5],
            rtc_tuple[6],
        )

    def find_matching_event(self, dow=None, time_text=None):
        if dow is None or time_text is None:
            dow, time_text = self.get_current_schedule()
        return self.store.find_duplicate_schedule(dow, time_text)

    def run_pending(self):
        dow, time_text = self.get_current_schedule()
        if self._last_trigger_dow == dow and self._last_trigger_time == time_text:
            return None

        event = self.store.find_duplicate_schedule(dow, time_text)
        self._last_trigger_dow = dow
        self._last_trigger_time = time_text

        if event is None:
            return None

        self.output_handler(event.out1, event.out2, event.out3)
        return event


class PinOutputHandler:
    """Aplicador opcional para tres pines digitales de salida."""

    __slots__ = ("pin1", "pin2", "pin3")

    def __init__(self, pin1, pin2, pin3):
        self.pin1 = pin1
        self.pin2 = pin2
        self.pin3 = pin3

    def __call__(self, out1, out2, out3):
        self.pin1.value(out1)
        self.pin2.value(out2)
        self.pin3.value(out3)
        return out1, out2, out3



def default_output_handler(out1, out2, out3):
    return out1, out2, out3



def create_recipe_timer_store(filename="recipe_timer.ini", base_path="/RecipeTime"):
    return RecipeTimerStore(filename, base_path)
