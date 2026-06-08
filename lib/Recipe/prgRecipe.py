class RecipeKeyValueStore:
    """Manage key-value files stored inside the Recipe directory."""

    def __init__(self, filename, base_path="/Recipe"):
        filename = filename.strip()
        if not filename:
            raise ValueError("El nombre del archivo no puede estar vacío.")
        self.base_path = base_path.rstrip("/") or "/Recipe"
        self.filename = filename
        self.path = "{}/{}".format(self.base_path, self.filename)

    def _load(self):
        config = {}
        try:
            with open(self.path, "r") as file:
                for line in file:
                    stripped = line.strip()
                    if not stripped or "=" not in stripped:
                        continue
                    key, value = stripped.split("=", 1)
                    key = key.strip()
                    value = value.strip()
                    if key:
                        config[key] = value
        except OSError:
            return {}
        return config

    def _save(self, config):
        with open(self.path, "w") as file:
            for key, value in config.items():
                file.write("{}={}\n".format(key, value))

    def create(self, initial_data=None):
        data = {}
        if initial_data:
            for key, value in initial_data.items():
                key_text = str(key).strip()
                if key_text:
                    data[key_text] = str(value)
        self._save(data)
        return self.path

    def read_all(self):
        return self._load()

    def read_value(self, key, default=None):
        return self._load().get(key, default)

    def set_value(self, key, value):
        key = str(key).strip()
        if not key:
            raise ValueError("La clave no puede estar vacía.")

        config = self._load()
        config[key] = str(value)
        self._save(config)
        return config[key]

    def update_values(self, values):
        config = self._load()
        for key, value in values.items():
            key_text = str(key).strip()
            if not key_text:
                continue
            config[key_text] = str(value)
        self._save(config)
        return config

    def delete_value(self, key):
        config = self._load()
        if key in config:
            deleted_value = config.pop(key)
            self._save(config)
            return deleted_value
        return None

    def delete_file(self):
        try:
            import os

            os.remove(self.path)
            return True
        except (ImportError, OSError):
            return False


# Compatibilidad con la API previa.
def read_config_kv(path="/Recipe/recipe.txt"):
    base_path, filename = path.rsplit("/", 1)
    return RecipeKeyValueStore(filename, base_path=base_path).read_all()


# Compatibilidad con la API previa.
def write_config_kv(config, path="/Recipe/recipe.txt"):
    base_path, filename = path.rsplit("/", 1)
    RecipeKeyValueStore(filename, base_path=base_path).create(config)
