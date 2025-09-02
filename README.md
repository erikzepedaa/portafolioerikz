# Plantilla de Documentación con MkDocs + Material

Formato de página web para documentar **proyectos** y **actividades de clase** usando **Markdown**.  
Esta plantilla genera un sitio con **MkDocs** y el tema **Material**.

---

## Qué van a lograr
- Ver la documentación **en su computadora** (servidor local con autorecarga).
- Editar archivos Markdown en `docs/`.
- Publicar el sitio **automáticamente** en GitHub Pages al hacer push (si usan GitHub con esta plantilla).

---

## Requisitos (instalar primero)

### 1) Instalar **Python 3.10+**
- **Windows/macOS/Linux:** descargar desde https://www.python.org/downloads/
- En **Windows**, marcar la casilla **“Add Python to PATH”** durante la instalación.

Para comprobar desde terminal:
```bash
python --version
# o según tu sistema
python3 --version
```

### 2) Instalar Git

- Descargar desde https://git-scm.com/downloads (Windows/macOS/Linux).

Para comprobar desde terminal:
```bash
git --version
```

Instalar: Visual Studio Code https://code.visualstudio.com/

### 3) Crear tu repositorio desde esta plantilla

1. Abre Visual Studio Code.
2. En la barra de menú, selecciona "Terminal"-> "Nuevo Terminal". Aparecerá una nueva pestaña de terminal en la parte inferior.
3. En un navegador de internet abre esta plantilla en GitHub y haz clic en Use this template → Create a new repository.
4. Ponle nombre a tu repo (por ejemplo, mi-docs) y crea el repositorio.
5. En tu computadora regresa a la terminal de Visual Studio Code, y elige en que carpeta clonar tu nuevo repo, usando los comandos `cd` por ejemplo:
```bash
cd ruta/a/tu/carpeta
```
6. Clona tu repositorio con el comando siguiente, reemplazando `<tu-usuario>` y `<tu-repo>` con el encontrado en la pagina, cuando le das click a "Code"->"https" y copias la ruta

```bash
git clone https://github.com/<tu-usuario>/<tu-repo>.git
cd <tu-repo>
```

!!! Importante: No olvides configurar tu nombre de usuario y correo electrónico en Git. Esto es necesario para que tus commits se registren correctamente.

```bash
git config --global user.name "Tu Nombre"
git config --global user.email "tu.email@example.com"
```

### 4) Instalar dependencias (una sola vez por repo)

Recomendado: usar entorno virtual para aislar paquetes. (OPCIONAL)

WINDOWS

```bash
pip install mkdocs-material
```

macOS / Linux

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install mkdocs-material
```

### 5) Ejecutar sitio

Desde la carpeta del repo en la terminal:

```bash
mkdocs serve
```

Abre el navegador en http://127.0.0.1:8000/
Cada cambio que hagas en archivos dentro de `docs/` se verá al instante cuando guardes(autorecarga).

---

## FAQS

### ¿Dónde edito?

Todas las páginas están en la carpeta `docs/`.
La página inicial es `docs/index.md`.
Crea más páginas (`.md`) y no uses acentos/espacios en nombres de archivo.

Controla el menú lateral agregando una sección `nav:` en `mkdocs.yml`.

### Publicación automática

Este proyecto incluye un flujo de GitHub Actions que publica el sitio automáticamente al hacer push a la rama principal (o cuando el flujo está configurado para ejecutarse).
Pasos:

1. Configura el deploy de tu pagina, en tu repositorio de GitHub, entra a la pestaña "Settings" y busca la sección "Pages".
2. En `source` selecciona Deploy from a branch, y en Branch seleccion `gh-pages` y `/root`
3. Sube tus Cambios
```bash
git add .
git commit -m "Actualizo documentación"
git push origin main
```
4. Ve a la pestaña Actions de tu repo en GitHub y verifica que el flujo de “build/deploy” se ejecute correctamente.
5. La página quedará disponible como Project Site en una URL del tipo:
```php-template
https://<tu-usuario>.github.io/<tu-repo>/
```

Si no aparece:

- Revisa Settings → Pages y selecciona GitHub Actions como método de publicación.
- Asegúrate de que el flujo en .github/workflows/ exista y finalice en “success”.


--- 

## Estructura Minima de proyecto
```bash
.
├─ docs/
│  └─ index.md        # tu portada
├─ mkdocs.yml         # configuración de MkDocs
├─ requirements.txt   # (opcional) dependencias
└─ .github/
   └─ workflows/      # flujo para publicar automáticamente
```
