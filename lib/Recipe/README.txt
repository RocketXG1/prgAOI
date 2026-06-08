Indice
1. Proposito del codigo
2. Uso basico
3. Ejemplo pequeno de uso
4. Funciones y metodos disponibles

1. Proposito del codigo
Este modulo en MicroPython permite guardar configuraciones simples en archivos de texto con formato clave=valor dentro de la carpeta /Recipe. La clase principal recibe solo el nombre del archivo y construye la ruta completa automaticamente.

2. Uso basico
- Copia prgRecipe.py en el Raspberry Pi Pico (o en tu proyecto).
- Asegura que exista la carpeta /Recipe en la raiz del Pico.
- Crea una instancia indicando unicamente el nombre del archivo.
- Usa los metodos de la clase para crear, leer, modificar y eliminar claves.

3. Ejemplo pequeno de uso
from prgRecipe import RecipeKeyValueStore

recipe = RecipeKeyValueStore("recipe.txt")
recipe.create({"modo": "automatico", "brillo": "25"})
recipe.set_value("color", "verde")
recipe.update_values({"brillo": "50", "velocidad": "120"})
print(recipe.read_all())
print(recipe.read_value("modo"))
recipe.delete_value("color")

4. Funciones y metodos disponibles
- RecipeKeyValueStore(nombre_archivo, base_path="/Recipe")
  Crea el manejador del archivo clave=valor.
- create(initial_data=None)
  Crea o sobrescribe el archivo con un diccionario inicial.
- read_all()
  Retorna todas las claves y valores del archivo.
- read_value(key, default=None)
  Retorna el valor de una clave especifica.
- set_value(key, value)
  Agrega o modifica una sola clave.
- update_values(values)
  Agrega o modifica varias claves desde un diccionario.
- delete_value(key)
  Elimina una clave del archivo.
- delete_file()
  Elimina el archivo completo.
- read_config_kv(path="/Recipe/recipe.txt")
  Funcion de compatibilidad para leer un archivo existente.
- write_config_kv(config, path="/Recipe/recipe.txt")
  Funcion de compatibilidad para escribir un archivo existente.
