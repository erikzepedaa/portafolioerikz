# 📌 Vehículo Controlado por Bluetooth con Motores DC

## Proposito  
Este proyecto consistió en el diseño y construcción de un vehículo a control remoto, utilizando motores de corriente directa (DC), un puente H para el control de dirección, y un microcontrolador ESP32 para la comunicación vía Bluetooth. El objetivo principal fue aplicar conocimientos básicos de electrónica, mecánica y programación para competir en una dinámica de robótica.

---
## Metas del Proyecto
Meta General:
Desarrollar un prototipo funcional de un coche a control remoto que pueda participar en una competencia de robótica, demostrando habilidades de diseño, integración de sistemas y trabajo en equipo.
Metas Específicas:

Implementar un sistema de control que permita maniobrar el vehículo con precisión y velocidad.
Diseñar una pala frontal que facilite la interacción con objetos (como pelotas) durante la competencia.
Optimizar el rendimiento del coche para lograr una ventaja competitiva en el juego.

## Alcance del Proyecto
El proyecto abarcó desde la conceptualización del diseño hasta la implementación completa del sistema electrónico y mecánico. Se trabajó con materiales accesibles y se estableció un límite de dos semanas para su desarrollo.
Incluye:
- **Diseño estructural del vehículo**
- **Integración de componentes electrónicos**
- **Programación del sistema de control**
- **Pruebas funcionales previas a la competencia**
---
# Proceso de Trabajo 
---
## Organización del Equipo
El equipo se dividió en dos áreas principales: diseño mecánico y desarrollo electrónico. Mientras algunos miembros se encargaron de la estructura del coche, otros se enfocaron en la programación y conexiones.
##Materiales Utilizados

- **2 motores DC**
- **Puente H**
- **ESP32**
- **Protoboard**
- **Jumpers**
- **LED**
- **Batería de 9V**
- **MDF para la base**
- **PLA para impresión 3D de la pala frontal**

## Tecnologías Utilizadas

- **Lenguajes:** `Python`, `C++`
- **Hardware:** `ESP32`, `Arduino`
- **Software:** `SolidWorks`, `PSeInt`
- **Otros:** `CircuitVerse` 

## Sistema Electrónico
Se conectaron los motores al puente H, asegurando una correcta polaridad y conexión a tierra. Los pines IN1 a IN4 se configuraron para controlar la dirección de giro de los motores. La ESP32 se integró como unidad de control, con especial atención a la asignación de pines y la protección contra cortocircuitos.
## Programación
Se desarrolló un programa en Arduino IDE que permite controlar el coche mediante una aplicación Bluetooth. Las funciones principales incluyeron:

- **Avanzar**
- **Retroceder**
- **Girar a la izquierda/derecha**
- **Detenerse**
- **Ajustar velocidad mediante PWM**

La lógica de movimiento se basó en la manipulación de los motores: por ejemplo, para girar, se detiene una rueda mientras la otra sigue girando.
El codigo para controlar el coche se encuentr al final del articulo.

## Resultados y Observaciones
Antes de la competencia, el coche mostró un buen desempeño: respondía a los comandos, la pala funcionaba correctamente y el diseño era estable. Sin embargo, durante el evento surgieron algunos inconvenientes:

Uno de los motores se desprendió tras un choque con una silla.
El control Bluetooth presentaba cierto retraso en la respuesta.
Los movimientos eran algo bruscos, lo que dificultaba la precisión.

A pesar de estos detalles, el coche cumplió con los objetivos técnicos del proyecto.

## Evidencias
[Ver en YouTube](https://www.youtube.com/shorts/-BoEvhaO5zg)

## Reflexiones Finales
El proyecto fue una excelente oportunidad para aplicar conocimientos teóricos en un entorno práctico. Se logró integrar electrónica, mecánica y programación en un sistema funcional. Algunas áreas de mejora identificadas fueron:

Realizar más pruebas antes de la competencia
Mejorar el montaje de los motores
Optimizar el control desde la app

Esta experiencia reforzó la importancia de la planificación, el trabajo colaborativo y la iteración constante para lograr un producto funcional y competitivo.


---
## Codigo para controlar el coche

```bash
#include "BluetoothSerial.h"
 
BluetoothSerial SerialBT;


// Pines del puente H
 
const int IN1 = 12; // Motor izquierdo
 
const int IN2 = 11;
 
const int ENA = 13;
 
const int IN3 = 10; // Motor derecho
 
const int IN4 = 9;
 
const int ENB = 7;
 
int valSpeed = 255;


void setup() {
 
  Serial.begin(115200);
 
  SerialBT.begin("CarroESP32"); // Nombre del dispositivo Bluetooth
 
  pinMode(IN1, OUTPUT);
 
  pinMode(IN2, OUTPUT);
 
  pinMode(ENA, OUTPUT);
 
  pinMode(IN3, OUTPUT);
 
  pinMode(IN4, OUTPUT);
 
  pinMode(ENB, OUTPUT);
 
  stopMotors();
 
}


void loop() {
 
  if (SerialBT.available()) {
 
    char command = SerialBT.read();
 
    Serial.println(command);
 
    switch (command) {
 
      case 'F': forward(); break;
 
      case 'B': backward(); break;
 
      case 'L': turnLeft(); break;
 
      case 'R': turnRight(); break;
 
      case 'S': stopMotors(); break;
 
      case '0': setSpeed(0); break;
 
      case '1': setSpeed(25); break;
 
      case '2': setSpeed(50); break;
 
      case '3': setSpeed(75); break;
 
      case '4': setSpeed(100); break;
 
      case '5': setSpeed(125); break;
 
      case '6': setSpeed(150); break;
 
      case '7': setSpeed(175); break;
 
      case '8': setSpeed(200); break;
 
      case '9': setSpeed(255); break;
 
    }
 
  }
 
}


void forward() {
 
  analogWrite(ENA, valSpeed);
 
  analogWrite(ENB, valSpeed);
 
  digitalWrite(IN1, HIGH);
 
  digitalWrite(IN2, LOW);
 
  digitalWrite(IN3, HIGH);
 
  digitalWrite(IN4, LOW);
 
}


void backward() {
 
  analogWrite(ENA, valSpeed);
 
  analogWrite(ENB, valSpeed);
 
  digitalWrite(IN1, LOW);
 
  digitalWrite(IN2, HIGH);
 
  digitalWrite(IN3, LOW);
 
  digitalWrite(IN4, HIGH);
 
}


void turnLeft() {
 
  analogWrite(ENA, valSpeed / 2);
 
  analogWrite(ENB, valSpeed);
 
  digitalWrite(IN1, HIGH);
 
  digitalWrite(IN2, LOW);
 
  digitalWrite(IN3, HIGH);
 
  digitalWrite(IN4, LOW);
 
}


void turnRight() {
 
  analogWrite(ENA, valSpeed);
 
  analogWrite(ENB, valSpeed / 2);
 
  digitalWrite(IN1, HIGH);
 
  digitalWrite(IN2, LOW);
 
  digitalWrite(IN3, HIGH);
 
  digitalWrite(IN4, LOW);
 
}


void stopMotors() {
 
  analogWrite(ENA, 0);
 
  analogWrite(ENB, 0);
 
  digitalWrite(IN1, LOW);
 
  digitalWrite(IN2, LOW);
 
  digitalWrite(IN3, LOW);
 
  digitalWrite(IN4, LOW);
 
}

 
void setSpeed(int val) {
 
  valSpeed = val;
 
}


```
