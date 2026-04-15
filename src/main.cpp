#include <Arduino.h>
#include <BluetoothSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <ESP32Encoder.h>


#define RayonRoue 0.0325                                                  // Rayon de la roue en m


// --- Déclaration des broches moteurs ---
unsigned char PWMGplus = 17;                                              // PWM moteur gauche (sens +)
unsigned char PWMDplus = 16;                                              // PWM moteur droit  (sens +)


unsigned char PWMGmoins = 4;                                              // PWM moteur gauche (sens -)
unsigned char PWMDmoins = 19;                                             // PWM moteur droit  (sens -)


char Batterie = 25;                                                       // Broche de mesure batterie


// --- Objets ---
BluetoothSerial SerialBT;                                                 // Communication Bluetooth
Adafruit_MPU6050 mpu;                                                     // Capteur MPU6050


ESP32Encoder encoderL;
ESP32Encoder encoderR;


// --- Variables capteurs / filtres ---
float TetaG, TetaW;                                                 // Angles bruts
float TetaWF, TetaGF, Teta;                                                     // Angles filtrés
char FlagCalcul = 0;                                                      // Indique fin du calcul
float Ve, Vs = 0;
float Te = 10.0;                                                            // Période d'échantillonnage (ms)
float Tau = 1000.0, TauVitesse = 570.0;                                         // Constante du filtre (ms)
float A, B;                                                               // Coefficients du filtre
float AVitesse, BVitesse;


// --- Mesure batterie ---
float R1 = 22000.0;                                                       // Résistance 22k
float R2 = 10000.0;                                                       // Résistance 10k
float valeurbatterie;


// --- Encodeur ---
long TetaMG,TetaMD;
long encodeur_precedentMG=0, encodeur_presentMG=0;
long encodeur_precedentMD=0, encodeur_presentMD=0;
float deltaEncodeurMG, deltaEncodeurMD, deltaMoyenne;
float deltaEncodeurMG_Angulaire, deltaEncodeurMD_Angulaire, vitesseLineaire, vitesseLineaireF;
int TetaMG_par_Tick, TetaMD_par_Tick;
const float Nb_de_ticks = 748.0;


// --- PID ---
  // Position
float kpPosition =3.19, kdPosition = 0.034;
float erreurTeta;
float TetaConsigne = 0.0 ;
  // Vitesse
float kpVitesse = 7.76 , kdVitesse = 0.235 , kiVitesse = 0.001;
float erreurVitesse, deriveVitesse = 0, erreurPrecedentVitesse, integraleVitesse=0;
float VitesseConsigne = 0.0;


float Ec, CO1 = 0.166, CO2 = 0.06;                                                    // Sortie du correcteur
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
   
    // --- MPU6050 ---
      //Calcul angle via accéléromètre
    TetaG = -atan2(a.acceleration.y, a.acceleration.x);                   // Angle brut
    TetaGF = A * TetaG + B * TetaGF;                                      // Filtre passe-bas


      //Calcul angle via gyroscope
    TetaW = g.gyro.z * Tau / 1000;                                        // Intégration gyro
    TetaWF = A * TetaW + B * TetaWF;                                      // Filtre passe-bas


      //Fusion des deux angles
    Teta = TetaWF + TetaGF;                                               // Angle final (radians)


    TetaMG = encoderL.getCount();                                         // Position Angulaire Moteur Gauche
    TetaMD = encoderR.getCount();                                         // Position Angulaire Moteur Droit
   
    deltaEncodeurMG = ((float)TetaMG - (float)encodeur_precedentMG)/((Te/1000.0));// VitesseMoteur Gauche en (rad/s)
    deltaEncodeurMD = ((float)TetaMD - (float)encodeur_precedentMD)/((Te/1000.0));// VitesseMoteur Droit en (rad/s)


    deltaEncodeurMG_Angulaire = deltaEncodeurMG*(2.0*PI/Nb_de_ticks);             // Vitesse Angulaire Moteur Gauche en (rad/s)
    deltaEncodeurMD_Angulaire = deltaEncodeurMD*(2.0*PI/Nb_de_ticks);             // Vitesse Angulaire Moteur Droit en (rad/s)


    deltaMoyenne = (deltaEncodeurMG_Angulaire + deltaEncodeurMD_Angulaire)/2.0;
   
    vitesseLineaire = deltaMoyenne*RayonRoue;                                                 // Vitesse Linéaire des Moteurs en (m/s)  
    vitesseLineaireF = AVitesse*vitesseLineaire + BVitesse*vitesseLineaireF;


    // --- Asservissement du Gyropode ---    
      // Calcul des variables de Asservissement de Vitesse
    erreurVitesse = VitesseConsigne - vitesseLineaireF;                                 // Erreur de la Vitesse
    deriveVitesse = erreurVitesse + erreurPrecedentVitesse;               // Dérivé du kdVitesse
    erreurPrecedentVitesse = erreurVitesse;
    integraleVitesse += erreurVitesse;                                    // Intégrale du kiVitesse
   
      // Asservissement Vitesse
    TetaConsigne = kpVitesse*erreurVitesse - kdVitesse*deriveVitesse+kiVitesse*integraleVitesse;
   
    erreurTeta = TetaConsigne - Teta;                                     // Erreur de Position


      // Asservissement Position
    Ec = -kpPosition * erreurTeta + kdPosition * g.gyro.z ;                // Sortie du correcteur
   
    if(Ec > 0) Ec +=CO1;                                                   // Compensation Force de frottement Sec
    if(Ec > 0) Ec +=CO2;                                                   // Compensation Force de frottement Sec


    if (Ec > 0.45) Ec=0.45;                                               // Saturation à 95% du programme
    if (Ec < -0.45) Ec=-0.45;


    // --- Conversion en PWM ---
    dutyCycle1 = (0.5 + Ec) * 1023 ;                                      // PWM sens +
    dutyCycle2 = (0.5 - Ec) * 1023 ;                                      // PWM sens -
   


    // --- Envoi PWM moteurs ---
    ledcWrite(MOTGplus, dutyCycle1);                                      // Moteur droit  +


    ledcWrite(MOTDplus, dutyCycle1);                        
    ledcWrite(MOTGmoins, dutyCycle2);                                     // Moteur gauche -
    ledcWrite(MOTDmoins, dutyCycle2);                                     // Moteur droit  -






    encodeur_precedentMG = TetaMG;                                        // Memorisation TetaMG
    encodeur_precedentMD = TetaMD;                                        // Memorisation TetaMD
    // --- Fin du cycle ---
    FlagCalcul = 1;                                                       // Indique que les calculs sont faits
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));                   // Attente période
  }
}




/*void Vin(void *parameters)                                              // Tâche pour la lecture de la tension de la batterie
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
  encoderL.attachHalfQuad(34, 35);
  encoderR.attachHalfQuad(27, 13);
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
 
  AVitesse = 1 / (1 + TauVitesse / Te);
  BVitesse = TauVitesse / Te * A;
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
    if (commande == "Tau")                                                      // Acquisition Valeur du Tau via TermMecatro
    {
      Tau = valeur.toFloat();
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
    }




    if (commande == "Te")                                                      // Acquisition Valeur du Te via TermMecatro
    {
      Te = valeur.toInt();
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
    }


    if (commande == "kpPosition") kpPosition = valeur.toFloat();               // Acquisition Valeur du kpPosition via TermMecatro
    if (commande == "kdPosition") kdPosition = valeur.toFloat();               // Acquisition Valeur du kdPosition via TermMecatro


    if (commande == "kpVitesse")  kpVitesse = valeur.toFloat();                // Acquisition Valeur du kpVitesse via TermMecatro
    if (commande == "kdVitesse")  kdVitesse = valeur.toFloat();                // Acquisition Valeur du kdVitesse via TermMecatro
    if (commande == "kiVitesse")  kiVitesse = valeur.toFloat();                // Acquisition Valeur du kiVitesse via TermMecatro
   
    if (commande == "CO1") CO1 = valeur.toFloat();
    if (commande == "CO2") CO2 = valeur.toFloat();
   
    if (commande == "TauVitesse") TauVitesse = valeur.toFloat();


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
    Serial.printf("%f %f %f \n",integraleVitesse,Ec, Teta);
 
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
