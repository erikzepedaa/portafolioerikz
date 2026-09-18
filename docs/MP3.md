# Guía Rápida: Primeros pasos con KiCad

Esta es una introducción práctica al diseño de placas de circuito impreso (PCB) con **KiCad**. Recorreremos el camino completo: desde el esquema lógico del circuito hasta la tarjeta física, pasando por la instalación de la librería de Fab Lab (`fablib`).

---

## Tabla de Contenido

- [1. Introducción y Entorno de KiCad](#1-introducción-y-entorno-de-kicad)
- [2. Descarga y Configuración de la Librería Fab Lab](#2-descarga-y-configuración-de-la-librería-fab-lib)
- [3. El Editor de Esquemas (Schematic Editor)](#3-el-editor-de-esquemas-schematic-editor)
- [4. El Editor de Placas (PCB Editor)](#4-el-editor-de-placas-pcb-editor)
- [5. Verificación (DRC) y Exportación](#5-verificación-drc-y-exportación)

---

## 1. Introducción y Entorno de KiCad

Todo el trabajo de un proyecto —esquemáticos, placas y archivos generados— se administra desde el gestor de proyectos de KiCad, que funciona como punto central de organización.

### Vista General del Proyecto

Así se ve el gestor principal trabajando sobre el proyecto **HELLOWORLD**:

<img src="../recursos/imgs/kicad_pantalla_completa_general.png" alt="Vista general de KiCad a pantalla completa" width="800">

---

## 2. Descarga y Configuración de la Librería Fab Lab

El primer paso es instalar el software oficial y sumar los componentes de fabricación digital que se usarán más adelante.

> 🔗 [Sitio oficial de descarga de KiCad](https://www.kicad.org/download/)

### Proceso de Descarga

1. **Selección de Plataforma:** indica el sistema operativo que usas (Windows, macOS, Linux o Docker).

<img src="../recursos/imgs/kicad_seleccion_so.png" alt="Pantalla de selección de sistema operativo" width="600">

2. **Espejos de Descarga:** elige un servidor cercano a tu región para que la descarga sea más rápida y estable.

<img src="../recursos/imgs/kicad_espejos_descarga.png" alt="Panel de espejos de descarga" width="600">

### Instalación de `fablib`

1. Ve al **Administrador de complementos y contenidos** dentro de KiCad.
2. En la pestaña de bibliotecas, localiza `FABLIB`, descárgala e instálala.
3. Cierra y vuelve a abrir la aplicación para que los cambios queden aplicados.

<img src="../recursos/imgs/fablib_instalacion.png" alt="Administrador de bibliotecas con fablib" width="600">

---

## 3. El Editor de Esquemas (Schematic Editor)

Aquí se define la lógica eléctrica del circuito: qué componentes lo forman y cómo se conectan entre sí mediante símbolos.

### Diagrama Esquemático Completo

Vista general de un circuito ya armado, con sus bloques de entrada, de salida y las etiquetas de red correspondientes:

<img src="../recursos/imgs/esquematico_pantalla_completa.png" alt="Diagrama esquemático completo" width="800">

### Flujo de Trabajo Esencial

- **Insertar componentes (`A`):** abre el buscador de símbolos y colócalos en el lienzo según los necesites.

<img src="../recursos/imgs/esquematico_agregar_componente.png" alt="Diálogo de selección de componentes" width="600">

- **Conexiones y Etiquetas (`W` / `L`):** conecta nodos cercanos con cables directos, o recurre a etiquetas de red cuando quieras evitar que las líneas se crucen.

<img src="../recursos/imgs/esquematico_etiquetas.png" alt="Esquema con etiquetas de red" width="600">

- **Documentación visual:** apóyate en rectángulos y bloques de texto para dejar el esquema ordenado y fácil de leer.

<img src="../recursos/imgs/esquematico_apoyo_visual.png" alt="Cuadros organizadores y texto descriptivo" width="600">

- **Validación (ERC):** corre el chequeo de reglas eléctricas para confirmar que ningún pin quedó sin conectar.

<img src="../recursos/imgs/esquematico_erc.png" alt="Ventana de ejecución del ERC" width="600">

---

## 4. El Editor de Placas (PCB Editor)

En esta etapa se traduce el esquema en la geometría real de la tarjeta: la forma de la placa y el trazado físico de las pistas.

### Vista del PCB Editor

Resultado final de una placa con contorno personalizado, pistas ruteadas en la cara de cobre superior (`F.Cu`) y zonas de relleno ya definidas:

<img src="../recursos/imgs/pcb_pantalla_completa.png" alt="Editor de placas PCB en pantalla completa" width="800">

### Pasos de Creación

1. **Sincronización (`F8`):** pasa los componentes definidos en el esquema hacia la placa.

<img src="../recursos/imgs/pcb_actualizar.png" alt="Botón de actualización de la PCB" width="600">
<img src="../recursos/imgs/pcb_actualizar1.png" alt="Ventana de actualización de la PCB" width="600">

2. **Capas de Trabajo:** el trabajo se concentra sobre todo en `F.Cu` (cobre superior) y `Edge.Cuts` (el contorno de la placa).

<img src="../recursos/imgs/pcb_capas.png" alt="Panel de gestión de capas" width="600">

3. **Ruteo y Pistas:**
   - Fija el grosor de cada elemento, por ejemplo **0.4 mm** para las pistas y **0.8 mm** para el contorno.
   - Usa la herramienta Ruta (`X`) para trazarlas, procurando no dejar ángulos de 90°.

<img src="../recursos/imgs/pcb_ancho_pistas.png" alt="Ancho de pistas" width="600">
<img src="../recursos/imgs/pcb_ruteo.png" alt="Pistas ruteadas" width="600">

> **Tip para cruces:** cuando dos pistas necesiten cruzarse, coloca una resistencia de `0 ohms` como puente en el esquemático y luego actualiza la placa para reflejar el cambio.
>
> <img src="../recursos/imgs/pcb_puente_resistencia.png" alt="Puente con resistencia" width="600">

4. **Detalles y Perforaciones:**
   - Añade rótulos de texto (`T`) y perforaciones ordenadas en matriz (`Ctrl + T`), y define el contorno final en `Edge.Cuts`.
   - Genera zonas de relleno (`Ctrl + Shift + Z`) y refresca los planos con la tecla `B`.

<img src="../recursos/imgs/pcb_texto.png" alt="Texto en placa" width="600">
<img src="../recursos/imgs/pcb_perforaciones_matriz.png" alt="Perforaciones y matriz" width="600">
<img src="../recursos/imgs/pcb_contorno_zonas.png" alt="Contorno y zonas" width="600">
<img src="../recursos/imgs/pcb_zona_rellena_panel.png" alt="Configuración de zonas rellenas" width="600">

---

## 5. Verificación (DRC) y Exportación

### Reglas de Diseño y DRC

- Define los márgenes de aislamiento en **Reglas de Diseño → Requerimientos**.
- Corre el **Verificador de Reglas de Diseño (DRC)** para asegurarte de que la placa quede sin errores.

<img src="../recursos/imgs/pcb_reglas_drc_config.png" alt="Configuración de reglas de diseño" width="600">
<img src="../recursos/imgs/pcb_drc.png" alt="Ventana del DRC sin errores" width="600">

### Exportación de Archivos

1. Entra a **Archivo → Salidas de fabricación** para generar Gerbers o archivos vectoriales en **SVG**.
2. Marca las capas que necesitas, ajusta el tamaño de página y produce los archivos finales listos para manufactura.

<img src="../recursos/imgs/pcb_salidas_de_fabricacion.png" alt="Menú de salidas de fabricación" width="600">
<img src="../recursos/imgs/Captura de pantalla 2026-09-12 232851.png" alt="Configuración de archivos Gerber y SVG" width="600">
