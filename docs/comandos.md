# Encabezados

``` codigo
# Título H1
## Título H2
### Título H3
```

# Título H1
## Título H2
### Título H3

---

# Énfasis y código en línea

``` codigo
**negritas**, *cursivas*, ~~tachado~~, `código en línea`
```

**negritas**, *cursivas*, ~~tachado~~, `código en línea`

---

# Citas (blockquote)

``` codigo
> Esta es una cita destacada.
> Puede tener múltiples líneas.
```

> Esta es una cita destacada.
> Puede tener múltiples líneas.

---

# Enlaces

``` codigo
[Enlace directo](https://www.iberopuebla.mx/)

[Texto del enlace de referencia][doc-ref]

[doc-ref]: https://www.iberopuebla.mx//docs "Título opcional"
```

[Enlace directo](https://www.iberopuebla.mx/)

[Texto del enlace de referencia][doc-ref]

[doc-ref]: https://www.iberopuebla.mx//docs "Título opcional"

---

# Listas: viñetas, numeradas y de tareas

``` codigo

- Item A
    * Subitem A.1
    * Subitem A.2
- Item B
    - Subitem B.1
    - Subitem B.2

1.  Paso 1
    1.  Paso 1.1
    2.  Paso 1.2
        1.  Paso 1.2.1
        2.  Paso 1.2.2
        
- [x] Hecho
- [ ] Pendiente

```

- Item A
    * Subitem A.1
    * Subitem A.2
- Item B
    - Subitem B.1
    - Subitem B.2

---

1.  Paso 1
    1.  Paso 1.1
    2.  Paso 1.2
        1.  Paso 1.2.1
        2.  Paso 1.2.2
        
- [x] Hecho
- [ ] Pendiente

---

# Tablas

``` codigo
| Componente | Cant. | Nota        |
|-----------:|:-----:|-------------|
| Sensor X   | 2     | I2C         |
| MCU Y      | 1     | WiFi/BLE    |
```

| Componente | Cant. | Nota        |
|-----------:|:-----:|-------------|
| Sensor X   | 2     | I2C         |
| MCU Y      | 1     | WiFi/BLE    |

---

# Imágenes

``` codigo
![Diagrama del sistema](recursos/imgs/ibero.jpeg)

<!-- Control de tamaño usando HTML (cuando se requiera) -->
<img src="../recursos/imgs/ibero.jpeg" alt="Diagrama del sistema" width="420">
```

![Diagrama del sistema](recursos/imgs/ibero.jpeg)

<img src="../recursos/imgs/ibero.jpeg" alt="Diagrama del sistema" width="420">

---

# PDFs (enlace y embebido)

``` codigo
[Descargar especificación (PDF)](recursos/archivos/Calendario.pdf)

<!-- Embed (requiere navegador compatible) -->
<object data="recursos/archivos/Calendario.pdf" type="application/pdf" width="100%" height="600">
  <p>No se pudo mostrar el PDF. <a href="../recursos/archivos/Calendario.pdf">Descargar</a></p>
</object>
```

[Descargar especificación (PDF)](recursos/archivos/Calendario.pdf)

<object data="../recursos/archivos/Calendario.pdf" type="application/pdf" width="100%" height="600">
  <p>No se pudo mostrar el PDF. <a href="../recursos/archivos/Calendario.pdf">Descargar</a></p>
</object>

---

# Admonitions (Material)

``` codigo
!!! note "Nota"
    Esto es una nota informativa.

!!! tip "Sugerencia"
    Un consejo breve para el usuario.

!!! warning "Advertencia"
    Precauciones o riesgos a considerar.

??? info "Más información (colapsable)"
    Contenido adicional que se puede expandir.
```

!!! note "Nota"
    Esto es una nota informativa.

!!! tip "Sugerencia"
    Un consejo breve para el usuario.

!!! warning "Advertencia"
    Precauciones o riesgos a considerar.

??? info "Más información (colapsable)"
    Contenido adicional que se puede expandir.

---

# Código con resaltado

``` codigo
```python
def medir(canal: int) -> dict:
    # Simulación de lectura
    return {"canal": canal, "valor": 523, "unidad": "mV"}

print(medir(1))
```
```

```python
def medir(canal: int) -> dict:
    # Simulación de lectura
    return {"canal": canal, "valor": 523, "unidad": "mV"}

print(medir(1))
```

---

# Separador horizontal

``` codigo
---
```

---

---

# Listas anidadas con código y notas

``` codigo
- **Módulo A**
  - Función: `procesar()`
  - Entrada:
    - `signal` (float)
    - `freq` (Hz)
  - Salida:
    - JSON con `valor`, `unidad`
  - !!! note
        Documenta rangos válidos y casos borde.
```

- **Módulo A**
  - Función: `procesar()`
  - Entrada:
    - `signal` (float)
    - `freq` (Hz)
  - Salida:
    - JSON con `valor`, `unidad`
  - !!! note
        Documenta rangos válidos y casos borde.

---

# Bloques de cita con código (pseudo-logs)

``` codigo
> **Log:**
> ```
> [12:00:00] Init OK
> [12:00:01] Conectando a I2C...
> [12:00:02] Lectura: 523 mV
> ```
```

> **Log:**
> ```
> [12:00:00] Init OK
> [12:00:01] Conectando a I2C...
> [12:00:02] Lectura: 523 mV
> ```
