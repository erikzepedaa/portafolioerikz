# Comunicación Bluetooth entre ESP32 y Celular
## Proposito
Establecer una comunicación inalámbrica entre un ESP32 y un teléfono celular mediante Bluetooth, para enviar mensajes desde la app de Arduino y visualizar el texto recibido en el monitor serial de la computadora.
## Meta de la practica
Establecer una comunicación inalámbrica entre el ESP32 y un celular mediante Bluetooth, para recibir mensajes escritos desde una app móvil y mostrarlos en el monitor serial de la computadora.
## Organización del Equipo
El equipo se dividió en dos áreas principales: desarrollo del codigo de programacion y desarrollo electrónico.

## Materiales Utilizados
- **ESP32**
- **Protoboard**
- **Cables de conexión (jumpers)**
- **Cable USB para cargar el programa**
- **Aplicación Arduino Bluetooth Controller (en celular Android)**
- **Computadora con Arduino IDE**
  
## Tecnologías Utilizadas
- **Lenguajes:** `Python`
- **Hardware:** `ESP32`, `Arduino`
- **Software:** `ARDUINO IDE` 

## Sistema Electrónico
El ESP32 se conectó a la computadora mediante USB. No se utilizaron componentes externos como sensores o actuadores, ya que el enfoque fue exclusivamente en la comunicación serial por Bluetooth. El ESP32 se configuró como servidor Bluetooth con un nombre personalizado ("ESP32AÑ") para que el celular pudiera detectarlo y conectarse.

## Programación
El código desarrollado en esta práctica permite establecer una comunicación inalámbrica entre un ESP32 y un teléfono celular mediante Bluetooth. El objetivo es que los mensajes escritos desde la app del celular se reciban en el ESP32 y se muestren en el monitor serial de la computadora.
Primero, se incluye la librería BluetoothSerial.h, que es necesaria para habilitar la comunicación Bluetooth en el ESP32. Luego, se crea un objeto llamado SerialBT, el cual se encargará de manejar la conexión Bluetooth.
En la función setup(), se inicializa la comunicación serial con la computadora usando Serial.begin(115200), lo que permite visualizar los datos en el monitor serial del Arduino IDE. Después, se inicia el Bluetooth con SerialBT.begin("ESP32AÑ"), donde "ESP32AÑ" es el nombre que aparecerá en el celular al buscar dispositivos Bluetooth. Finalmente, se imprime un mensaje indicando que el Bluetooth está listo para recibir conexiones.
En la función loop(), el programa verifica constantemente si hay datos disponibles desde el celular con SerialBT.available(). Si se detecta un mensaje, se lee con SerialBT.readString() y se muestra en el monitor serial usando Serial.println(). Esto permite ver en tiempo real lo que se escribe en la app del celular directamente en la pantalla de la computadora. Se incluye un pequeño retraso de 1 segundo (delay(1000)) para evitar que el ciclo se ejecute demasiado rápido.
Este código demuestra cómo se puede usar el ESP32 como receptor Bluetooth para recibir texto desde un dispositivo móvil y visualizarlo en la computadora, lo cual es útil para proyectos de comunicación, monitoreo o control remoto.

## Resultados y Observaciones
La conexión Bluetooth fue exitosa y estable.
Los mensajes escritos desde el celular se reflejaban correctamente en el monitor serial.
El ESP32 respondió de forma inmediata a los datos recibidos.
No se presentaron errores de transmisión ni desconexiones durante la prueba.

## Evidencias
[Ver en YouTube](https://youtube.com/shorts/FjRERy2lNNs?si=2Luq59n9TTcmMw0b)

## Reflexiones Finales
Esta práctica permitió entender cómo se establece una comunicación serial inalámbrica entre un microcontrolador y un dispositivo móvil.
Se reforzó el uso de librerías específicas como BluetoothSerial y la importancia de configurar correctamente la velocidad de transmisión.
Fue útil para visualizar cómo los datos enviados desde una app pueden ser procesados en tiempo real por el ESP32 y mostrados en la computadora.
---
## Codigo

```bash
#include "BluetoothSerial.h"   // Librería para comunicación Bluetooth

BluetoothSerial SerialBT;      // Objeto para manejar el Bluetooth

void setup() {
  Serial.begin(115200);        // Inicia comunicación serial con la PC
  SerialBT.begin("ESP32AÑ");   // Nombre del dispositivo Bluetooth
  Serial.println("Bluetooth listo. Esperando conexión...");
}

void loop() {
  if (SerialBT.available()) {                // Si hay datos disponibles por Bluetooth
    String mensaje = SerialBT.readString();  // Leer el mensaje recibido
    Serial.println("Recibido: " + mensaje);  // Mostrarlo en el monitor serial
  }
  delay(1000);  // Espera 1 segundo antes de revisar otra vez
}
```
