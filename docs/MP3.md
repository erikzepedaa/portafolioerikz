# Guía Rápida: Primeros pasos con KiCad

Bienvenido a esta introducción práctica al diseño de placas de circuito impreso (PCB) utilizando **KiCad**. Aquí verás el flujo esencial para llevar un circuito desde su esquema lógico hasta la tarjeta física, incluyendo la correcta incorporación de la librería Fab Lab (`fablib`)[cite: 1].

---

## Tabla de Contenido
- [1. Introducción y Entorno de KiCad](#1-introducción-y-entorno-de-kicad)
- [2. Descarga y Configuración de la Librería Fab Lab](#2-descarga-y-configuración-de-la-librería-fab-lab)
- [3. El Editor de Esquemas (Schematic Editor)](#3-el-editor-de-esquemas-schematic-editor)
- [4. El Editor de Placas (PCB Editor)](#4-el-editor-de-placas-pcb-editor)
- [5. Verificación (DRC) y Exportación](#5-verificación-drc-y-exportación)

---

## 1. Introducción y Entorno de KiCad

El gestor de proyectos de KiCad centraliza y administra todos los archivos de trabajo (esquemáticos, archivos de diseño de placas y salidas de fabricación)[cite: 1].

### Vista General del Proyecto

Vista del gestor principal con el proyecto *HELLOWORLD*:

<img src="../recursos/imgs/kicad_pantalla_completa_general.png" alt="Vista general de KiCad a pantalla completa" width="800">

---

## 2. Descarga y Configuración de la Librería Fab Lab

Para iniciar, asegúrate de contar con conexión a internet, descarga el software oficial y añade los componentes especializados de fabricación digital[cite: 1].

> 🔗 [Sitio oficial de descarga de KiCad](https://www.kicad.org/download/)

### Proceso de Descarga

1. **Selección de Plataforma:** Elige tu sistema operativo correspondiente (Windows, macOS, Linux o Docker)[cite: 1].

<img src="../recursos/imgs/kicad_seleccion_so.png" alt="Pantalla de selección de sistema operativo" width="600">

2. **Espejos de Descarga:** Selecciona un servidor regional cercano para una descarga rápida y segura[cite: 1].

<img src="../recursos/imgs/kicad_espejos_descarga.png" alt="Panel de espejos de descarga" width="600">

### Instalación de `fablib`

1. Dirígete a la pestaña superior **Herramientas** y abre el **Administrador de complementos y contenidos**[cite: 1].
2. Busca, descarga e instala `FABLIB` desde la pestaña de bibliotecas, haz clic en **Aplicar cambios pendientes** y presiona **Cerrar**[cite: 1].
3. **Paso crítico:** Cierra completamente la aplicación y vuelve a abrirla para que las librerías se indexen correctamente en el sistema[cite: 1].

<img src="../recursos/imgs/fablib_instalacion.png" alt="Administrador de bibliotecas con fablib" width="600">

---

## 3. El Editor de Esquemas (Schematic Editor)

Espacio dedicado a definir la lógica del circuito mediante símbolos normalizados y conexiones eléctricas.

### Diagrama Esquemático Completo

Estructura general del circuito con bloques de entradas, salidas, fuentes de alimentación y etiquetas de red[cite: 1]:

<img src="../recursos/imgs/esquematico_pantalla_completa.png" alt="Diagrama esquemático completo" width="800">

### Flujo de Trabajo Esencial

- **Configuración inicial:** Ajusta de preferencia la cuadrícula de trabajo en milímetros (`mm`) para mantener referencias métricas claras[cite: 1].
- **Insertar componentes (`A`):** Busca y añade los elementos necesarios al lienzo (filtrando por la librería `fablib` para componentes como resistencias SMD 1206, LEDs, pulsadores o pines)[cite: 1]. Usa la tecla **`R`** para rotarlos[cite: 1].

<img src="../recursos/imgs/esquematico_agregar_componente.png" alt="Diálogo de selección de componentes" width="600">

- **Conexiones y Etiquetas (`W` / `L`):** Une nodos cercanos con cables directos o utiliza etiquetas de red (como `VDD`, `GND`, `S1`) para evitar cruces innecesarios de líneas[cite: 1].

<img src="../recursos/imgs/esquematico_etiquetas.png" alt="Esquema con etiquetas de red" width="600">

- **Documentación visual:** Usa bloques rectangulares organizadores y notas de texto descriptivo para mantener el esquema limpio y legible[cite: 1].

<img src="../recursos/imgs/esquematico_apoyo_visual.png" alt="Cuadros organizadores y texto descriptivo" width="600">

- **Validación (ERC):** Ejecuta la Verificación de Reglas Eléctricas (ERC) para comprobar que no existan pines sueltos o errores de diseño lógico[cite: 1].

<img src="../recursos/imgs/esquematico_erc.png" alt="Ventana de ejecución del ERC" width="600">

---

## 4. El Editor de Placas (PCB Editor)

Fase clave para traducir el diagrama esquemático en la geometría real de la tarjeta física, definiendo su contorno y las pistas de cobre.

### Vista del PCB Editor

Diseño final de la tarjeta con silueta personalizada, ruteo en cobre frontal (`F.Cu`) y verificación espacial mediante el visor 3D (`Ver > Visor 3D`)[cite: 1]:

<img src="../recursos/imgs/pcb_pantalla_completa.png" alt="Editor de placas PCB en pantalla completa" width="800">

### Pasos de Creación

1. **Sincronización (`F8`):** Haz clic en **Actualizar la placa desde el esquemático** para transferir todos los componentes al entorno físico. Separa y desagrega los elementos en la zona de trabajo[cite: 1].

<img src="../recursos/imgs/pcb_actualizar.png" alt="Botón de actualización de la PCB" width="600">
<img src="../recursos/imgs/pcb_actualizar1.png" alt="Ventana de actualización de la PCB" width="600">

2. **Capas de Trabajo:** Enfoca el diseño principalmente sobre `F.Cu` (cobre superior) para las pistas y `Edge.Cuts` para definir el perímetro de corte[cite: 1].

<img src="../recursos/imgs/pcb_capas.png" alt="Panel de gestión de capas" width="600">

3. **Ruteo y Pistas:**  
   - Define grosores predefinidos en **Editar tamaños predefinidos**: usa por ejemplo **`0.4 mm`** para las pistas de señal y alimentación, y **`0.8 mm`** para contornos[cite: 1].
   - Traza utilizando la herramienta Ruta (`X`), procurando no dejar ángulos rectos cerrados (90°)[cite: 1].

<img src="../recursos/imgs/pcb_ancho_pistas.png" alt="Ancho de pistas" width="600">
<img src="../recursos/imgs/pcb_ruteo.png" alt="Pistas ruteadas" width="600">

> **Tip para cruces:** Si dos pistas se intersecan inevitablemente, puedes añadir una resistencia de `0 ohms` como puente lógico en el esquema y actualizar la placa[cite: 1].
> 
> <img src="../recursos/imgs/pcb_puente_resistencia.png" alt="Puente con resistencia" width="600">

4. **Detalles, Contorno y Zonas:**
   - Dibuja el contorno de la tarjeta en la capa `Edge.Cuts` (puedes crear figuras personalizadas) y ajusta su grosor exacto a **`0.8 mm`** para un fresado o corte óptimo[cite: 1].
   - Agrega textos identificativos (`T`) y perforaciones precisas mediante matrices (`Ctrl + T`)[cite: 1].
   - Añade zonas rellenas (`Ctrl + Shift + Z`) y actualiza los planos presionando la tecla **`B`**[cite: 1].

<img src="../recursos/imgs/pcb_texto.png" alt="Texto en placa" width="600">
<img src="../recursos/imgs/pcb_perforaciones_matriz.png" alt="Perforaciones y matriz" width="600">
<img src="../recursos/imgs/pcb_contorno_zonas.png" alt="Contorno y zonas" width="600">
<img src="../recursos/imgs/pcb_zona_rellena_panel.png" alt="Configuración de zonas rellenas" width="600">

---

## 5. Verificación (DRC) y Exportación

### Reglas de Diseño y DRC

- Configura los parámetros de aislamiento y clearance requeridos en las reglas de diseño[cite: 1].
- Ejecuta el **Verificador de Reglas de Diseño (DRC)** para certificar que el circuito esté completamente libre de errores de separación o pistas desconectadas[cite: 1].

<img src="../recursos/imgs/pcb_reglas_drc_config.png" alt="Configuración de reglas de diseño" width="600">
<img src="../recursos/imgs/pcb_drc.png" alt="Ventana del DRC sin errores" width="600">

### Exportación de Archivos

1. Ve a **Archivo > Salidas de fabricación** para generar los archivos Gerber o vectores en formato **SVG**[cite: 1].
2. Selecciona las capas requeridas, ajusta la escala o el tamaño de página y genera los trazos finales listos para manufactura o control numérico[cite: 1].

<img src="../recursos/imgs/pcb_salidas_de_fabricacion.png" alt="Menú de salidas de fabricación" width="600">
<img src="../recursos/imgs/Captura de pantalla 2026-09-12 232851.png" alt="Configuración de archivos Gerber y SVG" width="600">
