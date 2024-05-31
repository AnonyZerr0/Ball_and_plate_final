//***********************************************************************
// ZERGUIT Nour
// 6ème Electronique INRACI
// Projet ball and plate 2023-2024
// Hardware : ESP32 waroom 32E + servomoteurs+ écran résistif tactile
// 31/05/2024
#include <ESP32Servo.h>
#include <PID_v1.h>
#include "BluetoothSerial.h"
#include "esp_bt_device.h"  // Inclure pour esp_bt_gap_set_pin
#include "esp_bt_main.h"

// Définition des broches pour l'écran tactile
#define X1 26
#define X2 27
#define Y1 25
#define Y2 4

// Définition des broches pour les servomoteurs
#define SERVO_X_PIN 12
#define SERVO_Y_PIN 13

// Résolution de l'écran tactile
#define Xresolution 234 // 234 
#define Yresolution 190

// Variables pour stocker les positions des servomoteurs
double positionX = 0, positionY = 0;
double consigneX = 0, consigneY = 0;
double sortieX = 0, sortieY = 0;

// Paramètres PID
double Kp = 5.0, Ki = 0.6, Kd = 0.4;
 
// Initialisation des objets PID
PID PIDX(&positionX, &sortieX, &consigneX, Kp, Ki, Kd, DIRECT);
PID PIDY(&positionY, &sortieY, &consigneY, Kp, Ki, Kd, DIRECT);

// Initialisation des objets Servo
Servo servoX;
Servo servoY;

void setup() {
  Serial.begin(9600);

  // Configuration des servomoteurs
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);

  // Activation des PID
  PIDX.SetMode(AUTOMATIC);
  PIDY.SetMode(AUTOMATIC);

  // Définition des limites de sortie des PID
  PIDX.SetOutputLimits(-180, 180);
  PIDY.SetOutputLimits(-180, 180);

}

void loop() {

  lireEcranTactile();

  // Vérifie si un appui est détecté sur l'écran
  if (positionX == 0 && positionY == 0) {
    // Aucun appui détecté, positionner les servos à 90 degrés
    servoX.write(90);
    servoY.write(90);
    Serial.println("Aucun appui détecté - Servos à 90°");
  } else {
    // Mise à jour des PID
    PIDX.Compute();
    PIDY.Compute();

    // Application des sorties PID aux servomoteurs
    servoX.write(conversionSortieServo(sortieX, 'X'));
    servoY.write(conversionSortieServo(sortieY, 'Y'));
  }

  
}
void lireEcranTactile() {
  // Lecture de la position X et Y
  positionX = readX();
  positionY = readY();

  // Si aucun appui n'est détecté sur l'écran, on centre la bille
  if (positionX == 0 && positionY == 0) {
    // Définir les consignes au centre
    consigneX = 2550;  //  centre de l'axe X
    consigneY = 2500;  //  centre de l'axe Y
  } else {
    // Conversion des valeurs lues en consignes pour le PID
    consigneX = map(positionX, -Xresolution / 2, Xresolution / 2, -90, 90);
    consigneY = map(positionY, -Yresolution / 2, Yresolution / 2, -90, 90);
  }
}

int readX() {
  int X = 0;
  pinMode(Y1, INPUT);
  pinMode(Y2, INPUT);
  digitalWrite(Y2, LOW);
  pinMode(X1, OUTPUT);
  digitalWrite(X1, HIGH);
  pinMode(X2, OUTPUT);
  digitalWrite(X2, LOW);
  delay(5);
  X = analogRead(Y1);
  if (X < 60 || X > 3700) {
    return 0;  // Indique l'absence d'appui
  }
  X = map(X, 60, 3700, -Xresolution / 2, Xresolution / 2);
  return X;
}

int readY() {
  int Y = 0;
  pinMode(X1, INPUT_PULLUP);
  pinMode(X2, INPUT_PULLUP);
  digitalWrite(X2, LOW);
  pinMode(Y1, OUTPUT);
  digitalWrite(Y1, HIGH);
  pinMode(Y2, OUTPUT);
  digitalWrite(Y2, LOW);
  delay(5);
  Y = analogRead(X1);
  if (Y < 50 || Y > 3800) {
    return 0;  // Indique l'absence d'appui
  }
  Y = map(Y, 50, 3800, -Yresolution / 2, Yresolution / 2);
  return Y;
}

int conversionSortieServo(double sortie, char axe) {
  int angle;
  if (axe == 'X') {
    angle = map(sortie, -180, 180, 120, 60);
  } else if (axe == 'Y') {
    angle = map(sortie, 180, -180, 60, 144);
  }
  return angle;
}