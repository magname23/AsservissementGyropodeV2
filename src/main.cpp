#include <Arduino.h>
#include <BluetoothSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

// --- Déclaration des broches moteurs ---
unsigned char PWMGplus = 17;                                              // PWM moteur gauche (sens +)
unsigned char PWMDplus = 16;                                              // PWM moteur droit  (sens +)

unsigned char PWMGmoins = 4;                                              // PWM moteur gauche (sens -)
unsigned char PWMDmoins = 19;                                             // PWM moteur droit  (sens -)

char Batterie = 25;                                                       // Broche de mesure batterie

// --- Objets ---
BluetoothSerial SerialBT;                                                 // Communication Bluetooth
Adafruit_MPU6050 mpu;                                                     // Capteur MPU6050

// --- Variables capteurs / filtres ---
float TetaG, TetaW, Teta;                                                 // Angles bruts
float TetaWF, TetaGF;                                                     // Angles filtrés
char FlagCalcul = 0;                                                      // Indique fin du calcul
float Ve, Vs = 0;
float Te = 10;                                                            // Période d'échantillonnage (ms)
float Tau = 1000;                                                         // Constante du filtre (ms)
float A, B;                                                               // Coefficients du filtre

// --- Mesure batterie ---
float R1 = 22000.0;                                                       // Résistance 22k
float R2 = 10000.0;                                                       // Résistance 10k
float valeurbatterie;

// --- PID ---
float angleConsigne = 0.0;
float kp = 4.74, kd = 0.08, ki = 0.0;
float Ec;                                                                 // Sortie du correcteur
int dutyCycle1, dutyCycle2;                                               // PWM moteurs

// --- PWM ---
unsigned int frequence = 20000;                                           // 20 kHz
unsigned char MOTGplus = 0;                                               // Canal PWM moteur gauche +
unsigned char MOTDplus = 1;                                               // Canal PWM moteur droit  +
unsigned char MOTGmoins = 2;                                              // Canal PWM moteur gauche -
unsigned char MOTDmoins = 3;                                              // Canal PWM moteur droit  -
unsigned char resolution = 10;                                            // Résolution 10 bits (0–1023)

// --- Tâche de contrôle du gyropode ---
void controle(void *parameters)
{
  TickType_t xLastWakeTime;                                               // Temps de réveil FreeRTOS
  xLastWakeTime = xTaskGetTickCount();                                    // Initialisation

  while (1)
  {
    // --- Lecture MPU6050 ---
    sensors_event_t a, g, temp;                                           // Objets pour stocker mesures
    mpu.getEvent(&a, &g, &temp);                                          // Lecture capteur

    // --- Calcul angle via accéléromètre ---
    TetaG = -atan2(a.acceleration.y, a.acceleration.x);                   // Angle brut
    TetaGF = A * TetaG + B * TetaGF;                                      // Filtre passe-bas

    // --- Calcul angle via gyroscope ---
    TetaW = g.gyro.z * Tau / 1000;                                        // Intégration gyro
    TetaWF = A * TetaW + B * TetaWF;                                      // Filtre passe-bas

    // --- Fusion des deux angles ---
    Teta = TetaWF + TetaGF;                                               // Angle final (radians)

    // --- Correcteur PD ---
    Ec = Teta * kp + kd * g.gyro.z;                                       // Sortie du correcteur

    // --- Conversion en PWM ---
    dutyCycle1 = (0.5 + Ec) * 1023;                                       // PWM sens +
    dutyCycle2 = (0.5 - Ec) * 1023;                                       // PWM sens -

    // --- Saturation PWM ---
    if (dutyCycle1 > 1023) dutyCycle1 = 1023;
    if (dutyCycle2 > 1023) dutyCycle2 = 1023;

    if (dutyCycle1 < 0) dutyCycle1 = 0;
    if (dutyCycle2 < 0) dutyCycle2 = 0;

    // --- Envoi PWM moteurs ---
    ledcWrite(MOTGplus, dutyCycle1);                                      // Moteur gauche +
    ledcWrite(MOTDplus, dutyCycle1);                                      // Moteur droit  +

    ledcWrite(MOTGmoins, dutyCycle2);                                     // Moteur gauche -
    ledcWrite(MOTDmoins, dutyCycle2);                                     // Moteur droit  -

    // --- Fin du cycle ---
    FlagCalcul = 1;                                                       // Indique que les calculs sont faits
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));                   // Attente période
  }
}

/*
// --- Lecture tension batterie (désactivée) ---
void Vin(void *parameters)
{
  Ve = 1;
  while (1)
  {
    valeurbatterie = (((3.3/4095.0)*analogRead(Batterie)*(R1+R2))/R2) + 0.3;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
*/

void setup()
{
  Serial.begin(115200);

  // --- Configuration PWM ---
  ledcSetup(MOTGplus, frequence, resolution);
  ledcSetup(MOTDplus, frequence, resolution);

  ledcSetup(MOTGmoins, frequence, resolution);
  ledcSetup(MOTDmoins, frequence, resolution);

  // --- Assignation des broches ---
  ledcAttachPin(PWMGplus, MOTGplus);
  ledcAttachPin(PWMDplus, MOTDplus);
  ledcAttachPin(PWMGmoins, MOTGmoins);
  ledcAttachPin(PWMDmoins, MOTDmoins);

  // --- Initialisation MPU6050 ---
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // --- Création tâche contrôle ---
  xTaskCreate(
      controle,
      "controle",
      10000,
      NULL,
      10,
      NULL);

  // --- Calcul coefficients filtre ---
  A = 1 / (1 + Tau / Te);
  B = Tau / Te * A;
}

// --- Réception commandes série ---
void reception(char ch)
{
  static int i = 0;
  static String chaine = "";
  String commande;
  String valeur;
  int index, length;

  if ((ch == 13) or (ch == 10))
  {
    index = chaine.indexOf(' ');
    length = chaine.length();

    if (index == -1)
    {
      commande = chaine;
      valeur = "";
    }
    else
    {
      commande = chaine.substring(0, index);
      valeur = chaine.substring(index + 1, length);
    }

    // --- Commandes dynamiques ---
    if (commande == "Tau")
    {
      Tau = valeur.toFloat();
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
    }

    if (commande == "Te")
    {
      Te = valeur.toInt();
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
    }

    if (commande == "kp") kp = valeur.toFloat();
    if (commande == "kd") kd = valeur.toFloat();

    chaine = "";
  }
  else
  {
    chaine += ch;
  }
}

// --- Boucle principale ---
void loop()
{
  if (FlagCalcul == 1)
  {
    Serial.printf(" %lf %lf\n", kd, Ec); // Affichage debug
    FlagCalcul = 0;
  }
}

// --- Lecture série ---
void serialEvent()
{
  while (Serial.available() > 0)
  {
    reception(Serial.read());
  }
}