# Control de Giro, Velocidad y Posición con ESP32
## Proposito
Explorar el control de motores mediante el ESP32, aplicando técnicas de cambio de giro, variación de velocidad con PWM y posicionamiento de servo motores.
## Meta de la practica
Comprender cómo controlar motores DC y servos utilizando programación en ESP32, aplicando conceptos de lógica digital, modulación por ancho de pulso (PWM) y mapeo de valores para posicionamiento.
## Organización del Equipo
El equipo se dividió en dos áreas principales: desarrollo del codigo de programacion y desarrollo electrónico.

## Materiales Utilizados
- **ESP32**
- **Motor DC**
- **Servo motor**
- **Protoboard**
- **Cables de conexión (jumpers)**
- **Fuente de alimentación**
- **Cable USB para cargar el programa**

  
## Tecnologías Utilizadas
- **Lenguajes:** `Python`
- **Hardware:** `ESP32`, `Arduino`
- **Software:** `ARDUINO IDE` 

## Sistema Electrónico
Cambio de giro del motor DC: Se conectaron dos pines digitales al motor, alternando su estado para cambiar la dirección de giro (adelante y atrás).

Control de velocidad del motor DC: Se utilizó un pin PWM configurado para incrementar la velocidad del motor.

Control de posición de un servo motor: Se configuró un canal PWM con frecuencia de 50 Hz y resolución de 12 bits. Se usó la función map() para convertir grados (0° a 180°) en valores de duty cycle entre 205 y 410, que corresponden al rango de operación del servo.

## Programación
Cambio de giro: alterna los pines in1 e in2 para cambiar la dirección del motor.
Velocidad progresiva: usa ledcWrite() para aumentar gradualmente la velocidad del motor en pasos del 20%.
Servo motor: utiliza PWM de 12 bits para mover el servo a posiciones específicas (0°, 90°, 180°), mostrando los valores en el monitor serial.


## Resultados y Observaciones
El motor DC respondió correctamente al cambio de giro, alternando entre avance y retroceso.
El control de velocidad fue progresivo y estable, mostrando cómo el PWM puede modificar la potencia entregada al motor.
El servo motor se posicionó con precisión en los ángulos programados, y los valores de duty cycle se reflejaron correctamente en el monitor serial.
No se presentaron errores de conexión ni fallas en la ejecución de los códigos.

## Evidencias
[Ver en YouTube](https://www.youtube.com/watch?v=1Hk5i1rHxTI)

## Reflexiones Finales
Esta práctica permitió comprender tres aspectos fundamentales del control de motores con ESP32: dirección, velocidad y posición. Se reforzó el uso de PWM en diferentes resoluciones y frecuencias, y se evidenció cómo la programación puede traducirse en movimientos físicos precisos. Además, se aprendió a mapear valores para controlar servos, y se observó la importancia de los retardos (delay) para estabilizar los cambios.

## Codigo Direccion Basica

```bash
/*Control de 1 solo motor*/
#define in1 27
#define in2 14

void setup() {
  /*Declarar Pines Como salida*/
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

void loop() {
  /*ADELANTE*/
  digitalWrite(in1, 0);
  digitalWrite(in2, 1);
  delay(1000);
  /*ALTO*/
  digitalWrite(in1, 0);
  digitalWrite(in2, 0);
  delay(1000);
  /*ATRAS*/
  digitalWrite(in1, 1);
  digitalWrite(in2, 0);
  delay(1000);
  /*ALTO*/
  digitalWrite(in1, 0);
  digitalWrite(in2, 0);
  delay(1000);
}
```

## Codigo Control de velocidad

```bash
/*Control de 1 solo motor*/
#define in1 27
#define in2 14

void setup() {
  /*Declarar Pines Como salida*/
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

void loop() {
  /*ADELANTE*/
  digitalWrite(in1, 0);
  digitalWrite(in2, 1);
  delay(1000);
  /*ALTO*/
  digitalWrite(in1, 0);
  digitalWrite(in2, 0);
  delay(1000);
  /*ATRAS*/
  digitalWrite(in1, 1);
  digitalWrite(in2, 0);
  delay(1000);
  /*ALTO*/
  digitalWrite(in1, 0);
  digitalWrite(in2, 0);
  delay(1000);
}
```

## Codigo Control de Servo

```bash
/*Control de 1 solo motor*/
#define pwm 12 //Definicion de pin de Velocidad
int duty = 0;
int grados = 0;
void setup() {
  /*Declarar Pines Como salida*/
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  /*Configuracion de pin PWM 
    - Se conecta al pin 12(pwm)
    - Frecuencia de 50hz
    - Resolucion de 12 bit (0-4096)
    - Canal 0
  */
  ledcAttachChannel(pwm, 50, 12, 0);
  Serial.begin(115200);
}

void loop() { 
  /*
  Servo trabaja del ~5% al ~10% del total
  ~5% - 0°
  ~10% - 180°
  5% de 4096 = 204.8
  10% de 4096 = 409.6
  */
  grados=0;
  duty= map(grados, 0, 180, 205, 410);
  Serial.print("Pos: ");
  Serial.println(duty);
  ledcWrite(pwm, duty);
  delay(1000);
  grados=90;
  duty= map(grados, 0, 180, 205, 410);
  Serial.print("Pos: ");
  Serial.println(duty);
  ledcWrite(pwm, duty);
  delay(1000);
  grados=180;
  duty= map(grados, 0, 180, 205, 410);
  Serial.print("Pos: ");
  Serial.println(duty);
  ledcWrite(pwm, duty);
  delay(1000);
}
```
