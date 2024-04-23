/*
  Titre       DetectionDeMouvementAvecDecision
  Auteur      Crepin Vardin Fouelefack
  Date        25022024
  Description Actionnement du servoMoteur en fonction du capteur de mouvement et affichage a l'ecran
  Version     0.0.1
  Liens utiles
       code de base servoMoteur   5V       ESP32Servo by Kevin Harrington  analogWriteExample
                    capteur de mouvement 5 V httpswiki.dfrobot.comDigital_Infrared_motion_sensor__SKU_SEN0018_
                    Ecran LCD I2C  5V       https://wiki.dfrobot.com/Gravity__I2C_16x2_Arduino_LCD_with_RGB_Font_Display_SKU__DFR0554
 
*/
#include <Arduino.h>
#include "DFRobot_RGBLCD1602.h"
#include "ESP32Servo.h"
#include <WiFi.h>
#include <PubSubClient.h>

const int motionSensorPin = 36; // Broche à laquelle votre capteur de mouvement est connecté
const int ledPin = 23; // Broche à laquelle votre LED est connectée
const int servoPin = 5; // Broche à laquelle votre servo moteur est connecté

char dtaUart[15];
char dtaLen = 0;

bool motionDetected = false; // Variable pour stocker l'état du capteur de mouvement
unsigned long previousMillis = 0; // Variable pour stocker le dernier temps où le capteur a été vérifié
const long interval = 2000; // Intervalle de temps entre chaque vérification du capteur (en millisecondes)

DFRobot_RGBLCD1602 lcd(/*RGBAddr*/0x2D ,/*lcdCols*/16,/*lcdRows*/2);  //16 characters and 2 lines of show
Servo doorServo; // Créer une instance de la classe Servo

// Paramètres du réseau WiFi et du serveur MQTT
const char* ssid = "SM-A520W4657";
const char* password = "crepin23";
const char* mqtt_server = "192.168.203.215";
const char* mqtt_topic_distance = "uc1000/distanceUc1000"; // Topic MQTT pour recevoir la distance de Node-RED

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;

void openDoor() {
    // Déplacer le servo moteur pour simuler l'ouverture de la porte
    doorServo.write(90); // Angle de rotation pour ouvrir la porte (90 degrés)
    delay(2000); // Attendre un moment pour simuler l'ouverture de la porte
    doorServo.write(0); // Retourner le servo à sa position initiale
}

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.println("Message MQTT reçu !");
    String message = "";
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.println("Distance reçue : " + message);

    // Convertir la distance en float
    float distance = message.toFloat();
    // Si la distance est inférieure ou égale à 5.00, fermer le servo moteur en le mettant en position 0
    if (distance <= 5.00) {
        doorServo.write(0); // Mettre le servo moteur en position 0
    }
}

void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connecté...");
}

void setup() {
    Serial.begin(9600);
    lcd.init();
    pinMode(ledPin, OUTPUT);
    pinMode(motionSensorPin, INPUT);
    doorServo.attach(servoPin); // Attacher le servo moteur à la broche

    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback); // Définir la fonction de rappel pour la réception des messages MQTT
    client.subscribe(mqtt_topic_distance); // S'abonner au topic pour recevoir la distance de Node-RED
}



void reconnect() {
  while (!client.connected()) {
    Serial.print("Tentative de connexion au serveur MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("Connecté au serveur MQTT...");
      client.subscribe(mqtt_topic_distance);
    } else {
      Serial.print("Échec de la connexion au serveur MQTT, nouvelle tentative dans 5 secondes...");
      delay(5000);
    }
  }
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    unsigned long currentMillis = millis(); // Obtenez le temps actuel

    // Vérifiez si l'intervalle de temps s'est écoulé depuis la dernière vérification du capteur
    if (currentMillis - previousMillis >= interval) {
        // Mise à jour du temps de dernière vérification
        previousMillis = currentMillis;

        // Lire l'état du capteur de mouvement
        motionDetected = digitalRead(motionSensorPin) == HIGH;

        // Afficher l'état sur l'écran LCD
        lcd.clear(); // Effacer l'écran LCD
        lcd.setCursor(0, 0); // Positionner le curseur en haut à gauche
        if (motionDetected) {
            lcd.print("Motion: Detected");
            digitalWrite(ledPin, HIGH); // Allumer la LED si le mouvement est détecté
            openDoor(); // Ouvrir la porte lorsque le mouvement est détecté
        } else {
            lcd.print("Motion: Not detected");
            digitalWrite(ledPin, LOW); // Éteindre la LED si aucun mouvement n'est détecté
        }
    }

    // Attendre un court instant avant la prochaine itération (non bloquant)
    // Notez que le code ci-dessus peut ne pas s'exécuter exactement toutes les 500 millisecondes, mais il est non bloquant
    // Ce qui signifie que d'autres tâches peuvent être exécutées pendant ce temps
}
