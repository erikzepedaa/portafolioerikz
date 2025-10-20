# Encendido de LED con Pulsador usando Arduino
## Proposito
El objetivo principal de esta práctica fue comprender el funcionamiento básico de entradas y salidas digitales en Arduino. Se buscó controlar el encendido de un LED utilizando un botón físico conectado a una protoboard, aplicando conceptos fundamentales de electrónica como resistencias pull-down y lógica de control.
## Meta de la practica
Comprender el funcionamiento de entradas digitales en Arduino y aplicar este conocimiento para controlar un LED mediante un pulsador físico.
## Organización del Equipo
El equipo se dividió en dos áreas principales: desarrollo del codigo de programacion y desarrollo electrónico.

## Materiales Utilizados
- **Arduino UNO**
- **Protoboard**  
- **LED rojo**  
- **Resistencia de 220Ω** (limitadora para el LED)  
- **Resistencia de 10kΩ** (pull-down para el botón)  
- **Pulsador (botón)**  
- **Cables de conexión (jumpers)**  
- **Cable USB para cargar el programa**
## Tecnologías Utilizadas
- **Lenguajes:** `Python`
- **Hardware:** `ESP32`, `Arduino`
- **Software:** `ARDUINO IDE` 

## Sistema Electrónico
El circuito se armó sobre una protoboard. El LED se conectó al pin digital 13 del Arduino, con una resistencia de 220Ω en serie para evitar sobrecorriente. El botón se conectó al pin digital 2, acompañado de una resistencia de 10kΩ como pull-down para asegurar que el estado del pin sea bajo cuando el botón no está presionado.
La lógica del circuito es simple: al presionar el botón, el pin digital lee un estado alto (HIGH) y enciende el LED; al soltarlo, el estado vuelve a bajo (LOW) y el LED se apaga.

## Programación
El código fue escrito en Arduino IDE y se basa en la lectura del estado del botón para controlar el LED, el codigo se desarrollo en base y apoyo del codigo proporcionado por Oliver.

## Resultados y Observaciones
El circuito respondió correctamente al presionar el botón, encendiendo el LED de forma inmediata. Se comprobó que la resistencia pull-down evitó lecturas erráticas cuando el botón no estaba presionado. La práctica permitió verificar el funcionamiento básico de entradas y salidas digitales en Arduino, y se logró una interacción estable entre hardware y software

## Evidencias
[Ver en YouTube](https://m.youtube.com/watch?v=IkfE9FZbQ9E)

## Reflexiones Finales
Esta práctica fue una excelente introducción al uso de microcontroladores. Se logró entender cómo interactúan los componentes electrónicos básicos con el Arduino y cómo la programación puede controlar el comportamiento físico de un sistema.
Se aprendió la importancia de las resistencias pull-down para evitar lecturas erráticas, y se reforzó el concepto de entradas y salidas digitales. Además, se comprobó que incluso con un circuito sencillo se pueden lograr interacciones útiles entre hardware y software.

---
## Codigo

```bash
const int led=27;
const int button=32;

void setup() {
  Serial.begin(115200); //INICIO LA COMUNICACION EN 115200 char por segundo
  pinMode(led,OUTPUT);
  pinMode(button,INPUT);
}

void loop() {
  int estado = digitalRead(button);
  if(estado == 1){
    digitalWrite(led,1);
  }
  else{
    digitalWrite(led,0);
  }
}
```
