/*
 * Sketch Arduino per Hardware Interface ROS2 Control
 * Compatible con DiffDriveArduino plugin
 *
 * Hardware:
 * - Arduino Nano
 * - L298N Motor Driver
 * - 2x Encoder motore (per odometria)
 * - 2x Motori DC con ruote
 *
 * Comunicazione seriale a 57600 baud con Raspberry Pi
 */

// Pin L298N Motor Driver - Motore Sinistro
#define LEFT_MOTOR_EN   5   // Enable (PWM)
#define LEFT_MOTOR_IN1  6   // Direzione 1
#define LEFT_MOTOR_IN2  7   // Direzione 2

// Pin L298N Motor Driver - Motore Destro
#define RIGHT_MOTOR_EN  9   // Enable (PWM)
#define RIGHT_MOTOR_IN1 10  // Direzione 1
#define RIGHT_MOTOR_IN2 11  // Direzione 2

// Pin Encoder - Motore Sinistro
#define LEFT_ENCODER_A  2   // Interrupt pin
#define LEFT_ENCODER_B  4

// Pin Encoder - Motore Destro
#define RIGHT_ENCODER_A 3   // Interrupt pin
#define RIGHT_ENCODER_B 8

// Costanti Encoder
const int ENCODER_CPR = 374;  // Counts per revolution (modifica secondo il tuo encoder)
                              // Se usi encoder con riduttore, moltiplica per il rapporto
                              // Esempio: 11 CPR * 34:1 riduttore = 374 CPR

// Variabili globali encoder
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Variabili per calcolo velocità
long prevLeftCount = 0;
long prevRightCount = 0;
unsigned long prevTime = 0;

// Buffer comunicazione seriale
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

// Setpoint velocità (rad/s)
float leftWheelSetpoint = 0.0;
float rightWheelSetpoint = 0.0;

// Timeout per sicurezza
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 1000; // 1 secondo

void setup() {
  // Inizializza comunicazione seriale
  Serial.begin(57600);

  // Configura pin motori come output
  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  // Configura pin encoder come input
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

  // Attach interrupt per encoder
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);

  // Stop motori all'avvio
  stopMotors();

  // Inizializza timer
  prevTime = millis();
  lastCommandTime = millis();
}

void loop() {
  // Ricevi comandi dalla seriale
  receiveSerialData();

  // Processa nuovi comandi
  if (newData) {
    processCommand();
    newData = false;
  }

  // Controllo timeout - ferma i motori se non riceve comandi
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    leftWheelSetpoint = 0.0;
    rightWheelSetpoint = 0.0;
  }

  // Aggiorna controllo motori
  updateMotors();

  // Invia feedback encoder (10Hz)
  static unsigned long lastFeedback = 0;
  if (millis() - lastFeedback >= 100) {
    sendEncoderFeedback();
    lastFeedback = millis();
  }
}

// Interrupt Service Routine - Encoder Sinistro
void leftEncoderISR() {
  if (digitalRead(LEFT_ENCODER_B) == HIGH) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

// Interrupt Service Routine - Encoder Destro
void rightEncoderISR() {
  if (digitalRead(RIGHT_ENCODER_B) == HIGH) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

// Riceve dati dalla seriale
void receiveSerialData() {
  static byte idx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc == startMarker) {
      idx = 0;
    } else if (rc == endMarker) {
      receivedChars[idx] = '\0';
      idx = 0;
      newData = true;
    } else {
      if (idx < numChars - 1) {
        receivedChars[idx] = rc;
        idx++;
      }
    }
  }
}

// Processa comandi ricevuti
void processCommand() {
  // Formato comando: "L:velocità,R:velocità"
  // Esempio: "L:1.5,R:1.5" oppure "e" per richiesta encoder

  if (receivedChars[0] == 'e') {
    // Comando richiesta encoder - già gestito in sendEncoderFeedback
    return;
  }

  char* token = strtok(receivedChars, ",");

  while (token != NULL) {
    if (token[0] == 'L') {
      leftWheelSetpoint = atof(token + 2);
      lastCommandTime = millis();
    } else if (token[0] == 'R') {
      rightWheelSetpoint = atof(token + 2);
      lastCommandTime = millis();
    }
    token = strtok(NULL, ",");
  }
}

// Aggiorna controllo motori
void updateMotors() {
  // Converti velocità angolare (rad/s) in PWM (0-255)
  // Assumendo velocità massima di ~5 rad/s
  int leftPWM = constrain(abs(leftWheelSetpoint) * 51, 0, 255);   // 255/5 ≈ 51
  int rightPWM = constrain(abs(rightWheelSetpoint) * 51, 0, 255);

  // Motore sinistro
  if (leftWheelSetpoint > 0.05) {
    // Avanti
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    analogWrite(LEFT_MOTOR_EN, leftPWM);
  } else if (leftWheelSetpoint < -0.05) {
    // Indietro
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    analogWrite(LEFT_MOTOR_EN, leftPWM);
  } else {
    // Stop
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    analogWrite(LEFT_MOTOR_EN, 0);
  }

  // Motore destro
  if (rightWheelSetpoint > 0.05) {
    // Avanti
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    analogWrite(RIGHT_MOTOR_EN, rightPWM);
  } else if (rightWheelSetpoint < -0.05) {
    // Indietro
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    analogWrite(RIGHT_MOTOR_EN, rightPWM);
  } else {
    // Stop
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    analogWrite(RIGHT_MOTOR_EN, 0);
  }
}

// Ferma tutti i motori
void stopMotors() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_EN, 0);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(RIGHT_MOTOR_EN, 0);
}

// Invia feedback encoder
void sendEncoderFeedback() {
  // Formato output: "L:count,R:count\n"
  Serial.print("L:");
  Serial.print(leftEncoderCount);
  Serial.print(",R:");
  Serial.println(rightEncoderCount);
}
