#include <Servo.h>
#include <ServoEasing.h>
#include <IBusBM.h>
#include <AccelStepper.h>

/* =========================================================
   PINES DE LOS MOTORES DC (2 por rueda: IN1 / IN2)
   Cada rueda se controla con un puente H: un pin para una
   dirección, otro pin (PWM) para la velocidad.
   ========================================================= */
#define motorW1_IN1 6   // Rueda 1 (delantera izquierda)
#define motorW1_IN2 7
#define motorW2_IN1 4   // Rueda 2 (media izquierda)
#define motorW2_IN2 5
#define motorW3_IN1 2   // Rueda 3 (trasera izquierda)
#define motorW3_IN2 3
#define motorW4_IN1 13  // Rueda 4 (delantera derecha)
#define motorW4_IN2 10
#define motorW5_IN1 8   // Rueda 5 (media derecha)
#define motorW5_IN2 9
#define motorW6_IN1 11  // Rueda 6 (trasera derecha)
#define motorW6_IN2 12

/* =========================================================
   SERVOS DE DIRECCIÓN
   Cada servo gira la rueda sobre su eje vertical.
   ServoEasing permite movimientos suaves y controlados.
   ========================================================= */
ServoEasing servoW1;   // Servo dirección rueda delantera izquierda
ServoEasing servoW3;   // Servo dirección rueda trasera izquierda
ServoEasing servoW4;   // Servo dirección rueda delantera derecha
ServoEasing servoW6;   // Servo dirección rueda trasera derecha
ServoEasing servoCamTilt; // Servo inclinación de la cámara (tilt)

/* =========================================================
   MOTOR PASO A PASO PARA LA CÁMARA (PAN)
   Controlado con driver externo (STEP / DIR)
   ========================================================= */
AccelStepper camPanStepper(1, 46, 45);  // Tipo driver, pin STEP=46, pin DIR=45

/* =========================================================
   IBUS: protocolo de receptor RC (FlySky, Turnigy, etc.)
   IBus       -> recibe canales del control remoto
   IBusSensor -> envía telemetría (ej: voltaje de batería)
   ========================================================= */
IBusBM IBus;
IBusBM IBusSensor;

/* =========================================================
   VARIABLES DE ESTADO
   ========================================================= */
int angle = 0;              // ángulo genérico (no usado activamente)
int ch0, ch1, ch2, ch3, ch6 = 0; // valores de los canales RC

int servo1Angle = 90;       // posición base servo 1
int servo3Angle = 90;
int servo4Angle = 90;
int servo6Angle = 90;

int s = 0;                  // velocidad del rover en % (0–100)
int r = 0;                  // radio de giro en mm

int m1, m2, m3, m4, m5, m6; // valores auxiliares motores (no usados)

int camTilt = 90;           // ángulo de inclinación cámara (35–165)
int camPan = 0;             // velocidad de giro horizontal cámara

// Velocidades calculadas por rueda (0–100%)
float speed1, speed2, speed3 = 0;
// Velocidades convertidas a PWM (0–255)
float speed1PWM, speed2PWM, speed3PWM = 0;

// Ángulos calculados para cada servo de dirección
float thetaInnerFront, thetaInnerBack, thetaOuterFront, thetaOuterBack = 0;

/* =========================================================
   GEOMETRÍA DEL ROVER (distancias en mm)
   d1: distancia entre el centro y la fila delantera/trasera
   d2: distancia entre el centro y la fila trasera
   d3: distancia entre el centro y la fila delantera (para rueda media)
   d4: semiancho entre ruedas izquierda y derecha
   ========================================================= */
float d1 = 271;
float d2 = 278;
float d3 = 301;
float d4 = 304;

/* =========================================================
   SETUP
   ========================================================= */
void setup() {

  /*
     Si se necesita cambiar la frecuencia PWM de algunos pines,
     aquí están los registros TCCRxB (descomentar si aplica).
     Frecuencia más baja = menos ruido en motores DC.
  */
  // TCCR4B = TCCR4B & B11111000 | B00000101;  // D6,D7,D8   ~30 Hz
  // TCCR2B = TCCR2B & B11111000 | B00000111;  // D9,D10     ~30 Hz
  // TCCR1B = TCCR1B & B11111000 | B00000101;  // D11,D12    ~30 Hz
  // TCCR5B = TCCR5B & B11111000 | B00000101;  // D4,D13     ~30 Hz
  // TCCR3B = TCCR3B & B11111000 | B00000101;  // D2,D3,D5   ~30 Hz

  Serial.begin(115200);

  // Inicializar IBus (receptor RC en Serial1, telemetría en Serial2)
  IBus.begin(Serial1, IBUSBM_NOTIMER);
  IBusSensor.begin(Serial2, IBUSBM_NOTIMER);

  // Agregar sensor de voltaje (intervalo de envío)
  IBusSensor.addSensor(IBUSS_INTV);

  // Asignar pines a los servos de dirección
  servoW1.attach(22);
  servoW3.attach(23);
  servoW4.attach(24);
  servoW6.attach(25);
  servoCamTilt.attach(26);

  // Posición inicial de los servos (centrados)
  servoW1.write(90);
  servoW3.write(90);
  servoW4.write(90);
  servoW6.write(90);
  servoCamTilt.write(90);

  // Velocidad de giro de cada servo (grados/segundo)
  servoW1.setSpeed(550);
  servoW3.setSpeed(550);
  servoW4.setSpeed(550);
  servoW6.setSpeed(550);
  servoCamTilt.setSpeed(200);

  // Configuración del motor paso a paso de la cámara
  camPanStepper.setMaxSpeed(1000);
  camPan = 0;
  camTilt = 90;

  /* =========================================================
     INICIALIZACIÓN DE MOTORES DC
     Todos los pines quedan en LOW (motores detenidos).
     ========================================================= */
  // Rueda 1 – izquierda delantera
  digitalWrite(motorW1_IN1, LOW);
  digitalWrite(motorW1_IN2, LOW);
  // Rueda 2 – izquierda media
  digitalWrite(motorW2_IN1, LOW);
  digitalWrite(motorW2_IN2, LOW);
  // Rueda 3 – izquierda trasera
  digitalWrite(motorW3_IN1, LOW);
  digitalWrite(motorW3_IN2, LOW);
  // Las ruedas derechas giran en sentido inverso al izquierdo
  // Rueda 4 – derecha delantera
  digitalWrite(motorW4_IN1, LOW);
  digitalWrite(motorW4_IN2, LOW);
  // Rueda 5 – derecha media
  digitalWrite(motorW5_IN1, LOW);
  digitalWrite(motorW5_IN2, LOW);
  // Rueda 6 – derecha trasera
  digitalWrite(motorW6_IN1, LOW);
  digitalWrite(motorW6_IN2, LOW);
}

/* =========================================================
   LOOP PRINCIPAL
   ========================================================= */
void loop() {

  // ---- 1. Leer los canales del receptor RC ----
  IBus.loop();
  ch0 = IBus.readChannel(0);  // Dirección (steering)
  ch1 = IBus.readChannel(1);  // Inclinación cámara (tilt)
  ch2 = IBus.readChannel(2);  // Velocidad (throttle)
  ch3 = IBus.readChannel(3);  // Giro cámara (pan)
  ch6 = IBus.readChannel(6);  // Dirección de avance (forward/back)

  // ---- 2. Convertir dirección RC a radio de giro ----
  // Girar derecha (valores > 1515)
  if (ch0 > 1515) {
    r = map(ch0, 1515, 2000, 1400, 600); // radio: 1400mm a 600mm
  }
  // Girar izquierda (valores < 1485)
  else if (ch0 < 1485) {
    r = map(ch0, 1485, 1000, 1400, 600);
  }

  // ---- 3. Velocidad del rover (0–100%) ----
  s = map(ch2, 1000, 2000, 0, 100);

  // ---- 4. Control de inclinación de cámara (tilt) ----
  if (ch1 < 1485) {
    if (camTilt >= 35) {
      camTilt--;
      delay(20);
    }
  }
  if (ch1 > 1515) {
    if (camTilt <= 165) {
      camTilt++;
      delay(20);
    }
  }
  servoCamTilt.startEaseTo(camTilt);

  // ---- 5. Control de giro horizontal de cámara (pan) ----
  if (ch3 >= 1000 && ch3 < 1485) {
    camPan = map(ch3, 1000, 1485, 400, 0);
  }
  else if (ch3 > 1515 && ch3 <= 2000) {
    camPan = map(ch3, 1515, 2000, 0, -400);
  }
  else {
    camPan = 0;
  }
  camPanStepper.setSpeed(camPan);
  camPanStepper.run();

  // ---- 6. Calcular velocidades y ángulos ----
  calculateMotorsSpeed();
  calculateServoAngle();

  /* =========================================================
     GIRAR DERECHA
     Las ruedas exteriores (izquierda) giran más rápido.
     Las interiores (derecha) reducen su velocidad.
     Los servos apuntan las ruedas hacia el centro de giro.
     ========================================================= */
  if (ch0 > 1515) {

    // Servos: aplicar ángulos calculados
    servoW1.startEaseTo(97 + thetaInnerFront); // delantera izquierda
    servoW3.startEaseTo(97 - thetaInnerBack);  // trasera izquierda
    servoW4.startEaseTo(94 + thetaOuterFront); // delantera derecha
    servoW6.startEaseTo(96 - thetaOuterBack);  // trasera derecha

    // Motores DC
    if (ch6 < 1500) { // Avanzar
      // Lado izquierdo (exterior): máxima velocidad
      analogWrite(motorW1_IN1, speed1PWM); digitalWrite(motorW1_IN2, LOW);
      analogWrite(motorW2_IN1, speed1PWM); digitalWrite(motorW2_IN2, LOW);
      analogWrite(motorW3_IN1, speed1PWM); digitalWrite(motorW3_IN2, LOW);

      // Lado derecho (interior): velocidad reducida
      digitalWrite(motorW4_IN1, LOW); analogWrite(motorW4_IN2, speed2PWM);
      digitalWrite(motorW5_IN1, LOW); analogWrite(motorW5_IN2, speed3PWM);
      digitalWrite(motorW6_IN1, LOW); analogWrite(motorW6_IN2, speed2PWM);
    }
    else if (ch6 > 1500) { // Retroceder
      digitalWrite(motorW1_IN1, LOW); analogWrite(motorW1_IN2, speed1PWM);
      digitalWrite(motorW2_IN1, LOW); analogWrite(motorW2_IN2, speed1PWM);
      digitalWrite(motorW3_IN1, LOW); analogWrite(motorW3_IN2, speed1PWM);

      analogWrite(motorW4_IN1, speed2PWM); digitalWrite(motorW4_IN2, LOW);
      analogWrite(motorW5_IN1, speed3PWM); digitalWrite(motorW5_IN2, LOW);
      analogWrite(motorW6_IN1, speed2PWM); digitalWrite(motorW6_IN2, LOW);
    }
  }

  /* =========================================================
     GIRAR IZQUIERDA (simétrico al caso anterior)
     ========================================================= */
  else if (ch0 < 1485) {

    servoW1.startEaseTo(97 - thetaOuterFront);
    servoW3.startEaseTo(97 + thetaOuterBack);
    servoW4.startEaseTo(94 - thetaInnerFront);
    servoW6.startEaseTo(96 + thetaInnerBack);

    if (ch6 < 1500) { // Avanzar
      // Lado izquierdo (interior): velocidad reducida
      analogWrite(motorW1_IN1, speed2PWM); digitalWrite(motorW1_IN2, LOW);
      analogWrite(motorW2_IN1, speed3PWM); digitalWrite(motorW2_IN2, LOW);
      analogWrite(motorW3_IN1, speed2PWM); digitalWrite(motorW3_IN2, LOW);

      // Lado derecho (exterior): máxima velocidad
      digitalWrite(motorW4_IN1, LOW); analogWrite(motorW4_IN2, speed1PWM);
      digitalWrite(motorW5_IN1, LOW); analogWrite(motorW5_IN2, speed1PWM);
      digitalWrite(motorW6_IN1, LOW); analogWrite(motorW6_IN2, speed1PWM);
    }
    else if (ch6 > 1500) { // Retroceder
      digitalWrite(motorW1_IN1, LOW); analogWrite(motorW1_IN2, speed2PWM);
      digitalWrite(motorW2_IN1, LOW); analogWrite(motorW2_IN2, speed3PWM);
      digitalWrite(motorW3_IN1, LOW); analogWrite(motorW3_IN2, speed2PWM);

      analogWrite(motorW4_IN1, speed1PWM); digitalWrite(motorW4_IN2, LOW);
      analogWrite(motorW5_IN1, speed1PWM); digitalWrite(motorW5_IN2, LOW);
      analogWrite(motorW6_IN1, speed1PWM); digitalWrite(motorW6_IN2, LOW);
    }
  }

  /* =========================================================
     AVANZAR RECTO
     Todas las ruedas a la misma velocidad, servos centrados.
     ========================================================= */
  else {
    servoW1.startEaseTo(97);
    servoW3.startEaseTo(97);
    servoW4.startEaseTo(94);
    servoW6.startEaseTo(96);

    if (ch6 < 1500) { // Avanzar
      analogWrite(motorW1_IN1, speed1PWM); digitalWrite(motorW1_IN2, LOW);
      analogWrite(motorW2_IN1, speed1PWM); digitalWrite(motorW2_IN2, LOW);
      analogWrite(motorW3_IN1, speed1PWM); digitalWrite(motorW3_IN2, LOW);

      digitalWrite(motorW4_IN1, LOW); analogWrite(motorW4_IN2, speed1PWM);
      digitalWrite(motorW5_IN1, LOW); analogWrite(motorW5_IN2, speed1PWM);
      digitalWrite(motorW6_IN1, LOW); analogWrite(motorW6_IN2, speed1PWM);
    }
    else if (ch6 > 1500) { // Retroceder
      digitalWrite(motorW1_IN1, LOW); analogWrite(motorW1_IN2, speed1PWM);
      digitalWrite(motorW2_IN1, LOW); analogWrite(motorW2_IN2, speed1PWM);
      digitalWrite(motorW3_IN1, LOW); analogWrite(motorW3_IN2, speed1PWM);

      analogWrite(motorW4_IN1, speed1PWM); digitalWrite(motorW4_IN2, LOW);
      analogWrite(motorW5_IN1, speed1PWM); digitalWrite(motorW5_IN2, LOW);
      analogWrite(motorW6_IN1, speed1PWM); digitalWrite(motorW6_IN2, LOW);
    }
  }

  /* =========================================================
     TELEMETRÍA: enviar voltaje de batería al receptor RC
     ========================================================= */
  int sensorValue = analogRead(A0);
  // Conversión: lectura ADC -> voltaje real (con divisor resistivo)
  float voltage = sensorValue * (5.00 / 1023.00) * 3.02;
  IBusSensor.loop();
  IBusSensor.setSensorMeasurement(1, voltage * 100);
}

/* =========================================================
   CÁLCULO DE VELOCIDADES DE MOTORES
   ---------------------------------------------------------------------
   Si el rover va recto, todas las ruedas giran igual.
   Si gira, las ruedas exteriores giran más rápido que las interiores.
   Se calcula con la geometría del rover y el radio de giro.
   ========================================================= */
void calculateMotorsSpeed() {

  // Sin dirección: todas iguales
  if (ch0 > 1485 && ch0 < 1515) {
    speed1 = speed2 = speed3 = s;
  }
  else {
    // Exteriores: velocidad máxima
    speed1 = s;

    // Delantera/trasera interior: velocidad reducida por geometría
    speed2 = s * sqrt(pow(d3, 2) + pow((r - d1), 2)) / (r + d4);

    // Rueda interior central: la más lenta (más cerca del centro de giro)
    speed3 = s * (r - d4) / (r + d4);
  }

  // Convertir % (0–100) a PWM (0–255)
  speed1PWM = map(round(speed1), 0, 100, 0, 255);
  speed2PWM = map(round(speed2), 0, 100, 0, 255);
  speed3PWM = map(round(speed3), 0, 100, 0, 255);
}

/* =========================================================
   CÁLCULO DE ÁNGULOS DE SERVOS
   ---------------------------------------------------------------------
   Cada servo gira su rueda para apuntar al centro de giro.
   Se usa trigonometría: atan(distancia / radio).
   - thetaInnerFront: rueda interior delantera
   - thetaInnerBack:  rueda interior trasera
   - thetaOuterFront: rueda exterior delantera
   - thetaOuterBack:  rueda exterior trasera
   ========================================================= */
void calculateServoAngle() {
  thetaInnerFront = round((atan((d3 / (r + d1)))) * 180 / PI);
  thetaInnerBack  = round((atan((d2 / (r + d1)))) * 180 / PI);
  thetaOuterFront = round((atan((d3 / (r - d1)))) * 180 / PI);
  thetaOuterBack  = round((atan((d2 / (r - d1)))) * 180 / PI);
}