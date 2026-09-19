<h1>Flujo de Trabajo: Diseño y Manufactura CNC (KiCad + Mods)</h1>

<table width="100%">
  <tr>
    <td align="center" valign="top" width="50%">
      <img src="../recursos/imgs/Sebas.jpg" alt="Sebastian Gomez Rodriguez" width="150" height="150" style="width: 150px !important; height: 150px !important; max-width: 150px !important; object-fit: cover; border-radius: 12px; display: block; margin: 0 auto 10px auto;" />
      <b>Sebastian Gomez Rodriguez</b>
      <br />
      204486
    </td>
    <td align="center" valign="top" width="50%">
      <img src="../recursos/imgs/erik.jpg" alt="Erik Andre Zepeda Tapia" width="150" height="150" style="width: 150px !important; height: 150px !important; max-width: 150px !important; object-fit: cover; border-radius: 12px; display: block; margin: 0 auto 10px auto;" />
      <b>Erik Andre Zepeda Tapia</b>
      <br />
      204440
    </td>
  </tr>
</table>

<h2>Gestión del Entorno y Librerías</h2>
<p>Esta documentación define el estándar operativo para la elaboración, exportación y fabricación de placas de circuito impreso (PCB) mediante fresado CNC local, garantizando la compatibilidad con el repositorio especializado de componentes para prototipado.</p>

<h3>Arquitectura e Instalación</h3>
<p>El gestor de proyectos opera como el núcleo centralizador para la administración de jerarquías, bases de datos de huellas y archivos de salida de manufactura. Descargue el ejecutable desde el portal oficial de KiCad y seleccione un servidor de distribución geográficamente cercano.</p>

<img src="../recursos/imgs/kicad_pantalla_completa_general.png" alt="Vista general del gestor de proyectos" class="img-fluida">

<div class="img-grid">
    <img src="../recursos/imgs/kicad_seleccion_so.png" alt="Selección de SO" class="img-fluida">
    <img src="../recursos/imgs/kicad_espejos_descarga.png" alt="Espejos de descarga" class="img-fluida">
</div>

<h3>Despliegue de Librería FabLab</h3>
<p>La integración de repositorios locales es indispensable para asegurar la correspondencia entre el diseño digital y el inventario físico del laboratorio.</p>
<ol>
    <li>Navegue al menú <b>Herramientas</b> y ejecute el <b>Administrador de complementos y contenidos</b>.</li>
    <li>Localice el paquete <b>FABLIB</b> en la pestaña de bibliotecas y proceda con la instalación.</li>
    <li>Seleccione <b>Aplicar cambios pendientes</b>.</li>
    <li>Reinicie la instancia de KiCad obligatoriamente para indexar los nuevos símbolos y huellas en el sistema.</li>
</ol>

<img src="../recursos/imgs/fablib_instalacion.png" alt="Instalación de Fablib" class="img-fluida">

<h2>Captura Esquemática</h2>
<p>El entorno esquemático define la topología lógica del circuito y establece las relaciones funcionales mediante símbolos normalizados.</p>

<h3>Topología y Espacio de Trabajo</h3>
<p>Configure la retícula de trabajo estrictamente en milímetros (mm) para mantener la escala normalizada. Estructure el plano mediante bloques funcionales (alimentación, interfaces de control, elementos activos) apoyándose en herramientas de texto y delimitadores visuales.</p>

<img src="../recursos/imgs/esquematico_pantalla_completa.png" alt="Esquemático completo" class="img-fluida">
<img src="../recursos/imgs/esquematico_apoyo_visual.png" alt="Bloques organizadores" class="img-fluida">

<h3>Instanciación y Conectividad</h3>
<ul>
    <li><b>Asignación de Símbolos (A):</b> Invoque el selector de componentes y filtre exclusivamente por el directorio <code>fablib</code>. Utilice componentes estandarizados como resistencias SMD 1206 o microcontroladores específicos (SeeedStudio XIAO RP2040).</li>
    <li><b>Orientación (R):</b> Ajuste la posición espacial de cada símbolo para optimizar el flujo visual.</li>
    <li><b>Enrutamiento Lógico (W / L):</b> Conecte nodos cercanos con trazos directos. Para señales extensas, implemente etiquetas de red globales (VDD, GND, TX, RX) para evitar la saturación del plano.</li>
</ul>

<div class="img-grid">
    <img src="../recursos/imgs/esquematico_agregar_componente.png" alt="Agregar componente" class="img-fluida">
    <img src="../recursos/imgs/esquematico_etiquetas.png" alt="Etiquetas de red" class="img-fluida">
</div>

<h3>Validación de Reglas (ERC)</h3>
<p>Ejecute el Verificador de Reglas Eléctricas (ERC) para certificar la estabilidad de la red. Si el sistema arroja advertencias sobre pines de alimentación no controlados, asigne etiquetas <code>PWR_FLAG</code>. El reporte debe resultar libre de conflictos antes de avanzar.</p>

<img src="../recursos/imgs/esquematico_erc.png" alt="Reporte ERC" class="img-fluida">

<h2>Diseño Físico (PCB Layout)</h2>
<p>Esta fase traduce la topología lógica en una topometría física, estableciendo dimensiones de tarjeta, restricciones espaciales y trazos de cobre.</p>

<h3>Sincronización de Datos</h3>
<p>Ejecute el comando <b>Actualizar la placa desde el esquemático (F8)</b>. Este proceso importa la lista de redes (netlist) y los encapsulados físicos al área de trabajo.</p>

<img src="../recursos/imgs/pcb_pantalla_completa.png" alt="Vista de PCB Editor" class="img-fluida">

<div class="img-grid">
    <img src="../recursos/imgs/pcb_actualizar.png" alt="Actualizar PCB Botón" class="img-fluida">
    <img src="../recursos/imgs/pcb_actualizar1.png" alt="Ventana actualizar PCB" class="img-fluida">
</div>

<h3>Gestión de Capas y Ruteo</h3>
<ul>
    <li><b>Restricciones de Pistas:</b> En <i>Editar tamaños predefinidos</i>, fije un grosor de <b>0.4 mm</b> para pistas de señal/datos y <b>0.8 mm</b> para líneas de potencia.</li>
    <li><b>Trazado:</b> Utilice la herramienta de enrutamiento (X) ejecutando cambios de dirección a <b>45 grados</b>. Evite ángulos rectos para optimizar el paso de la fresa CNC.</li>
    <li><b>Puentes de Salto:</b> Ante la imposibilidad de rutear sin cruces, retorne al esquemático, integre una resistencia puente de 0 ohms, y sincronice los cambios.</li>
</ul>

<div class="img-grid">
    <img src="../recursos/imgs/pcb_ancho_pistas.png" alt="Anchos predefinidos" class="img-fluida">
    <img src="../recursos/imgs/pcb_ruteo.png" alt="Ruteo a 45 grados" class="img-fluida">
</div>
<img src="../recursos/imgs/pcb_puente_resistencia.png" alt="Puente 0 ohms" class="img-fluida">

<h3>Geometría y Planos de Cobre</h3>
<ul>
    <li><b>Contorno de Placa:</b> Cambie a la capa <code>Edge.Cuts</code> y defina una geometría completamente cerrada. Asigne un grosor de trazo de <b>2.0 mm</b>.</li>
    <li><b>Planos de Tierra:</b> Genere zonas de relleno de cobre (Ctrl + Shift + Z) asignadas a GND y actualice los polígonos (B) para maximizar la disipación térmica.</li>
    <li><b>Mecánica y Serigrafía:</b> Añada referencias de texto (T) y genere matrices de perforación paramétricas (Ctrl + T).</li>
</ul>

<div class="img-grid">
    <img src="../recursos/imgs/pcb_contorno_zonas.png" alt="Contorno Edge Cuts" class="img-fluida">
    <img src="../recursos/imgs/pcb_zona_rellena_panel.png" alt="Zonas de relleno" class="img-fluida">
    <img src="../recursos/imgs/pcb_texto.png" alt="Serigrafía" class="img-fluida">
    <img src="../recursos/imgs/pcb_perforaciones_matriz.png" alt="Matriz de perforación" class="img-fluida">
</div>

<h2>Salidas de Fabricación</h2>
<h3>Inspección Final (DRC)</h3>
<p>Ejecute el Verificador de Reglas de Diseño (DRC) para asegurar que no existen cortocircuitos, colisiones mecánicas, ni violaciones a las tolerancias de la máquina CNC.</p>

<div class="img-grid">
    <img src="../recursos/imgs/pcb_reglas_drc_config.png" alt="Configuración DRC" class="img-fluida">
    <img src="../recursos/imgs/pcb_drc.png" alt="Reporte DRC" class="img-fluida">
</div>

<h3>Exportación Vectorial (SVG)</h3>
<p>Navegue a <b>Archivo > Exportar > SVG...</b> para compilar los paquetes de manufactura aplicando los siguientes parámetros:</p>
<ol>
    <li><b>Modo de impresión:</b> Seleccione <b>Blanco y negro</b>.</li>
    <li><b>Lienzo:</b> Active la opción <b>Área de la placa únicamente</b> (Ajustar página a la placa) para evitar descalibraciones del punto de origen.</li>
    <li><b>Capas:</b> Exporte de forma independiente los archivos <code>pistas.svg</code> (F.Cu), <code>contorno.svg</code> (Edge.Cuts) y <code>perforaciones.svg</code>.</li>
</ol>

<div class="img-grid">
    <img src="../recursos/imgs/pcb_salidas_de_fabricacion.png" alt="Opciones de salida" class="img-fluida">
    <img src="../recursos/imgs/Captura de pantalla 2026-09-12 232851.png" alt="Parámetros SVG" class="img-fluida">
</div>

<h2>Procesamiento CNC (SRM-20)</h2>
<h3>Configuración de Plataforma CAM</h3>
<p>Inicie la plataforma web Mods Community. Cargue el entorno de trabajo navegando a <code>Programs > Open Program > Roland SRM-20 milling machine > mill 2D PCB</code>.</p>

<h3>Secuencia de Maquinado</h3>
<p>Ajuste los parámetros según la siguiente tabla e introduzca los archivos SVG en la máquina en este orden estricto:</p>

<table>
    <thead>
        <tr>
            <th>Operación</th>
            <th>Archivo</th>
            <th>Herramienta</th>
            <th>Velocidad</th>
            <th>Parámetros Específicos</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><b>1. Perforaciones</b></td>
            <td><code>perforaciones.svg</code></td>
            <td>Broca 0.8 mm</td>
            <td>0.3 – 0.4 mm/s</td>
            <td><b>Profundidad pasada:</b> 0.254 mm. <b>Total:</b> 1.7 mm. Se ejecuta primero para aprovechar la rigidez de la placa.</td>
        </tr>
        <tr>
            <td><b>2. Trazado</b></td>
            <td><code>pistas.svg</code></td>
            <td>V-Bit 0.4 mm</td>
            <td>4.0 mm/s</td>
            <td><b>Activar Invert:</b> Lo negro representa el área a remover. <b>Offset:</b> 4 pasadas.</td>
        </tr>
        <tr>
            <td><b>3. Contorno</b></td>
            <td><code>contorno.svg</code></td>
            <td>Fresa 2.0 mm</td>
            <td>1.5 – 4.0 mm/s</td>
            <td><b>Profundidad total:</b> 1.7 mm. <b>Offset:</b> 1 pasada. Separa la tarjeta del panel.</td>
        </tr>
    </tbody>
</table>