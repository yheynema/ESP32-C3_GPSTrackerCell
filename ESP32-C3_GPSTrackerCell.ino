
/*  --- Entête principale -- information sur le programme
 *   
 *  Programme:        GPS tracker
 *  Date:             22 juin 2024
 *  Auteur:           Yh
 *  Pltform matériel: ESP32-C3 M5 Stamp et module SIM7080G
 *  Pltform develop : Arduino 2.3.2 - ESP32 3.0.1
 *  Description:      Sert à tracert la position du dispositif dans le temps, via cellulaire.
 *  Fonctionnalités:  Position GPS, connectivité MQTT via réseau cellulaire, envoi à TB.
 *  Notes:
 *    
 */
 
/* --- Materiel et composants -------------------------------------------
 * Module ESP32-C3 Stamp de M5
 * Module GSM/GNSS SIM7080G
*/


/* --- HISTORIQUE de développement --------------------------------------
 * v0.1.x: version initiale de mise en commu de 2 codes: lecture GPS et envoi MQTT via réseau cellulaire
 * v0.2.x: version qui marche, reste a optimiser
 * v0.3.x: un peu de rafinement et ajout de divers champs à l'envoi.
 * v0.4.x: augmenter intervention sur un blocage que GPS par hot,warm,col start; fix sur calcul distance entre 2 pt
 * v0.5.x: preparation d'envoi à TB
 *
*/
//-----------------------------------------------------------------------

#define _VERSION "0.5.1"

//--- Déclaration des librairies (en ordre alpha) -----------------------
#define TINY_GSM_MODEM_SIM7080

#include <TinyGsmClient.h>
#include <TimeLib.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <TinyGPSPlus.h>
#include "secrets.h"

//-----------------------------------------------------------------------


//--- Definitions -------------------------------------------------------
//Paramètres pour le port Serial1 (ESP32-C3):
#define RX1 5
#define TX1 4

//Broche d'activation du module SIM7080G (broche K)
#define GSMKeyPin 6

//Broche de la LED indiquant la validité du data GPS
#define LED_PIN 7  //Yh module ESP32-C3 M5 Stamp

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Bon pour un ESP32-C32
#define SerialAT Serial1

#define TINY_GSM_YIELD() { delay(4); }

//Définition de la struture de données dans laquelle la string du GPS est récupérée:
typedef struct gpsDataStruc {
  uint8_t GNSSRunStatus;
  uint8_t fixStatus;
  char UTCDT[19];
  float latitude;
  float longitude;
  float mslAltitude;
  float speedOG;
  float courseOG;
  uint8_t FixMode;
  //n/a			don't care
  float HDOP;
  float PDOP;
  float	VDOP;
  //n/a			don't care
  uint8_t GPSSatView;
  //n/a			don't care
  float HPA;
  float VPA;
};

gpsDataStruc gpsData;

//-----------------------------------------------------------------------


//--- Declaration des objets --------------------------------------------
TinyGsm       modem(SerialAT);
TinyGsmClient newclient(modem);
PubSubClient  localMqtt(newclient);

//-----------------------------------------------------------------------


//--- Constantes --------------------------------------------------------
// Your GPRS credentials, if any
const char apn[]      = "simbase";
const char gprsUser[] = "";
const char gprsPass[] = "";
//Our Montreal timezone as from https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
const char* myLoclaTzStr = "EST5EDT,M3.2.0,M11.1.0";

// MQTT details
const char* brokerShiftr = "technophyscal.cloud.shiftr.io";
const char* brokerTBCAL = "technophys-tb.claurendeau.qc.ca";
const char* topicPos  = "GsmTracker/gps";
const char* topicInit = "GsmTracker/init";
const char* telemetryTopic = "v1/devices/me/telemetry";
const char* attributeTopic = "v1/devices/me/attributes";

//-----------------------------------------------------------------------


//--- Variables globales ------------------------------------------------
uint32_t gpsTimer = 0;
uint32_t gpsTimerDelay = 30*1000;  //30 secondes
bool gpsEnabled = false;  //global status of GPS
enum appModes {GPS,SENDDATA};
appModes currentMode = GPS;
float previousPos[2] = {0.0,0.0};
float currentPos[2] = {0.0,0.0};
bool gpsDataValid=false;
uint8_t gpsAcquisitionFailed=0;

uint32_t GPRSlastReconnectAttempt =0;
const uint32_t GPRSReconnectDelay = 10000;

uint32_t mqttReconnectDelay = 10000;
uint32_t mqttLastReconnectAttempt = 0;

uint32_t changeModeTimer = 0;
const uint32_t modeSendData = 2*60*1000; // 2 minutes... mais devrait plutôt être dicté une fois l'envoi réalisé (ou echec pour manque)
const uint32_t modeGPS = 2*60*1000;  //2 minutes... acquisition pendant 5 minutes. laisse suffisamment de temps au module GPS de récupérer la position.
uint32_t changeModeDelay = modeSendData;

uint16_t iterationCounter = 0;

int ledStatus = LOW;
//-----------------------------------------------------------------------


//--- Prototypes --------------------------------------------------------
//-----------------------------------------------------------------------


//--- Section des routines specifiques ----------------------------------

//*** ENTETE DE ROUTINE/FONCTION (pour CHACUNE)

/* 
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */

bool mqttConnect(PubSubClient *mqtt, bool debug=false) {
  // SerialMon.print("Connecting to ");
  // SerialMon.print(broker);

  bool status = mqtt->connect(MQTT_CLIENT_NAME, MQTT_USER, MQTT_PASS);
  mqtt->loop();

  if (status)
    if (debug) SerialMon.print(" success");
  else
    if (debug) SerialMon.print(" fail");

  return mqtt->connected();
}

bool connectMQTTBroker(PubSubClient *mqtt, int retry=3)
{
  bool retCode = false;
  int loopCount = retry;
  mqttLastReconnectAttempt = 0; //initial start, if required
  while (!mqtt->connected() && loopCount>0) { 
    SerialMon.print("!");
    uint32_t t = millis();
    // Reconnect every 10 seconds
    if (t - mqttLastReconnectAttempt > mqttReconnectDelay) { 
      mqttLastReconnectAttempt = t;
      mqtt->disconnect();
      mqtt->loop();
      //SerialMon.printf("MQTT disconnected. Connecting to %s...", brokerShiftr);
      SerialMon.printf("MQTT disconnected. Connecting to %s...", brokerTBCAL);
      delay(500);
      if (mqttConnect(mqtt,true)) {
        mqttLastReconnectAttempt = 0;
        //mqtt->subscribe(topicPos);
        //mqtt->publish(topicInit, "GsmGPS started");
        //mqtt->loop();
      } else loopCount--;
      SerialMon.println(".");
    } else delay(1000);
  }
  if (loopCount>0) {
    retCode = true;
    mqtt->loop();
    if (loopCount < retry) Serial.println(F("Connected to the MQTT broker!"));
  }
  return retCode;
}

void setTimezone(const char* timezone){
  Serial.printf("  Setting Timezone to %s\n",timezone);
  setenv("TZ",timezone,1);
  tzset();
}

bool initGSMNTP() {
  bool retCode = false;

  if (modem.NTPServerSync("pool.ntp.org", 0))  //Ne procède à aucun ajustement selon le TZ
    retCode = true;

  return retCode;
}  

bool getGSMNTP(int retry=3,bool rtc=false) {

  //Yh: C'est à se demander si cette façon est la bonne, par NTP.
  //   Car il est aussi possible d'obtenir l'heure par le GPS, ou par le réseau cellulaire (GPRS)

  bool retCode = false;
  int   ntp_year     = 0;
  int   ntp_month    = 0;
  int   ntp_day      = 0;
  int   ntp_hour     = 0;
  int   ntp_min      = 0;
  int   ntp_sec      = 0;
  float ntp_timezone = 0;

  int loopCount = retry;

  while (!modem.getNetworkTime(&ntp_year, &ntp_month, &ntp_day, &ntp_hour,&ntp_min, &ntp_sec, &ntp_timezone) && loopCount>0) {
    SerialMon.print("NTP loop: "); SerialMon.println(loopCount);
  }

  if (loopCount>0) {
    retCode = true;
    struct tm tm;
    tm.tm_year = ntp_year - 1900;   // Set date
    tm.tm_mon = ntp_month-1;
    tm.tm_mday = ntp_day;
    tm.tm_hour = ntp_hour;      // Set time
    tm.tm_min = ntp_min;
    tm.tm_sec = ntp_sec;
    tm.tm_isdst = 0;  // 1 or 0 -- Yh: comment dire?
    time_t t = mktime(&tm);
    Serial.printf("Setting time: %s", asctime(&tm));
    struct timeval now = { .tv_sec = t };
    if (rtc) {
      settimeofday(&now, NULL);
      setTimezone(myLoclaTzStr);
    }
  }
  return retCode;
}

bool parseGPSdata(gpsDataStruc &data, bool debug=false) {
  bool retCode = false;

  //Facon alternative puisque getGPS() marche pas...  
  char rawChar[100];      
  char rawGPSdata[100];

  strcpy(rawChar,modem.getGPSraw().c_str());

  //On va conserver une copie puisque strtok() altère le contenu
  strcpy(rawGPSdata,rawChar);

  if (strlen(rawChar)) {
    //1st step: count for the number of found field.
    char * pch;
    if(debug) SerialMon.printf("Splitting string \"%s\" into tokens:\n",rawChar);
    pch = strtok (rawChar,",");
    int count=1;
    while (pch != NULL)
    {
      if (debug) SerialMon.printf("%d:%s\n",count,pch);
      pch = strtok (NULL,",");
      if (pch != NULL) count++;
    }
    if (debug) SerialMon.print("> found "+String(count)+" fields: ");
    //2nd step: found 15 fields? yes, let's start over and retreive specific data
    if (count==15) {
      if (debug) SerialMon.println(" Ok.");
      //Do again and match
      pch = NULL;
      pch = strtok (rawGPSdata,",");
      int count=1;
      while (pch != NULL)
      {
        switch (count) {
          case 1: data.GNSSRunStatus = atoi(pch); break;
          case 2: data.fixStatus = atoi(pch); break;
          case 3: strcpy(data.UTCDT,pch); break;
          case 4: data.latitude = atof(pch); break;
          case 5: data.longitude = atof(pch); break;
          case 6: data.mslAltitude = atof(pch); break;
          case 7: data.speedOG = atof(pch); break;
          case 8: data.courseOG = atof(pch); break;
          case 9: data.FixMode = atoi(pch); break;
          case 10: data.HDOP = atof(pch); break;
          case 11: data.PDOP = atof(pch); break;
          case 12: data.VDOP = atof(pch);break;
          case 13: data.GPSSatView = atoi(pch); break;
          case 14: data.HPA = atof(pch); break;
          case 15: data.VPA = atof(pch); break;
          default: if (debug) SerialMon.println("error?");break;
        }

        if (debug) SerialMon.println("R"+String(count)+":"+String(pch));

        //Next token:
        pch = strtok (NULL,",");
        if (pch != NULL) count++;
      }
      if (debug) SerialMon.println("1) "+String(data.GNSSRunStatus));
      if (debug) SerialMon.println("2) "+String(data.fixStatus));
      if (debug) SerialMon.println("3) "+String(data.UTCDT));
      if (debug) SerialMon.println("4) "+String(data.latitude,6));
      if (debug) SerialMon.println("5) "+String(data.longitude,6));

      // strptime marche pas avec cette forme... pour le moment... - 23 juin 2024
      // // Recover time from UTCDT string
      // struct tm tm;
      // time_t t;
      // strncpy(rawChar,data.UTCDT,strspn(data.UTCDT,"."));
      // if (strptime(rawChar,"%Y%m%d%H%M%S",&tm) != NULL) {
      //   tm.tm_isdst = -1;
      //   t = mktime(&tm);
      //   if (debug) SerialMon.println("epoch: "+String(t));
      // } else if (debug) SerialMon.println("strptime error");

      retCode = true;
    } else if (debug) SerialMon.println(" expecting 15.");
  } else if (debug) SerialMon.println("> GPS:No data");

  return retCode;
}

void activateModem(int dly=0) {
  if (dly==0) dly = 1500;
  pinMode(GSMKeyPin,OUTPUT);
  digitalWrite(GSMKeyPin,LOW);
  delay(dly);
  digitalWrite(GSMKeyPin,HIGH);
  if (dly>8000) delay(8000);  //Cas d'espèce: si on veut resetter le modem, on met un dly à 12-15 sec
  pinMode(GSMKeyPin,INPUT);
}

bool connectGPRSNetwork(int retry=3) {
  bool retCode = false;

  bool networkConnected = false;

  if (!modem.isNetworkConnected()) { 
    int loopCount = retry;
    SerialMon.print("Checking Network: ");
    while (!modem.isNetworkConnected() && loopCount>0) {
      SerialMon.print("disconnected. Attempting connect: ");
      if (!modem.waitForNetwork(12000L, true)) {
        SerialMon.print("fail... ");
        //delay(10000);
        loopCount--;
      }
    }
    if (loopCount>0) {
      SerialMon.println("Network (re)connected");
      networkConnected = true;
    }
  } else { 
    SerialMon.println("Network already connected");
    networkConnected = true;
  }
  if (networkConnected) {
    // and make sure GPRS/EPS is still connected
    int loopCount = retry;
    SerialMon.print("Checking GPRS connection to ");
    SerialMon.print(apn); SerialMon.print(": ");
    if (!modem.isGprsConnected()) {
      while (!modem.isGprsConnected() && loopCount>0) {
        // SerialMon.println("GPRS disconnected!");
        // SerialMon.print(F("Connecting to "));
        // SerialMon.print(apn);
        SerialMon.print(".");
        uint32_t t = millis();
        // Reconnect every 10 seconds
        if (t - GPRSlastReconnectAttempt > GPRSReconnectDelay) { 
          GPRSlastReconnectAttempt = t;
          if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
            SerialMon.print("!");
            loopCount--;
          } else SerialMon.print(" * ");
        } else delay(1000);
      }
      if (loopCount>0) {
        SerialMon.println(" > GPRS (re)connected");
        SerialMon.println("IP is: "+modem.getLocalIP()+" oper:"+modem.getOperator());
        retCode = true;
      } else SerialMon.println(" > GPRS failed!");
    } else {
      SerialMon.println(" > GPRS already connected with IP: "+modem.getLocalIP());
      retCode = true;      
    }
  } 
  return retCode;
}

String getDateTimeStr() {
  int tailleBuf = 36;
  char buf[tailleBuf];
  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);
  int len = strftime(buf,tailleBuf,"%D %T",&timeinfo);
  return String(buf);
}

bool enableGPS(bool disena) {
  bool retCode = false;
  if (disena)
    if (modem.enableGPS()) {
      retCode = true;
    }
  else modem.disableGPS();
  return retCode;
}

bool getGPSCoords(float &lat, float &lon, int retry=3) {
  bool retCode = false;
  float gps_latitude  = 0;
  float gps_longitude = 0;
  int loopCount = retry;
  while (modem.getGPS(&gps_latitude, &gps_longitude) && loopCount>0) {
    SerialMon.print("GPS loop: "); SerialMon.println(loopCount);
    loopCount--;
  }
  if (loopCount>0) {
    lat = gps_latitude;
    lon = gps_longitude;
    retCode = true;
  } else SerialMon.println(" > getGPS failed");

  return retCode;
}

bool performModemInit() {
  bool retCode = false;
  int loopCount = 1;
  int hardCount = 0;
  bool modemInitStatus = false;
  while (!modemInitStatus) {    //Yh: utilise init() au lieu de restart()
    modemInitStatus = modem.init();
    if (modemInitStatus) {
       retCode = true;
    } else {
      SerialMon.print("Failed to restart modem... wait 10sec and retrying: "); SerialMon.println(loopCount);
      if (loopCount%3 == 0) {
        SerialMon.println("Re-activate modem (K pin trick)");
        activateModem(1750);
        hardCount++;
      }
      if (hardCount>2) { //Donc apres 3 fois.
        SerialMon.println("Hard reset on K pin");
        activateModem(12500);  //Hard RESET;
        hardCount = 0;
      }
      loopCount++;
      delay(5000L);
    }
  }
  return retCode;
}


//-----------------------------------------------------------------------


//--- Setup et Loop -----------------------------------------------------
void setup() {
  SerialMon.begin(115200);
  while (!Serial) {yield();}
  for (int i=0; i<8; i++) {  //Boucle de délais de 8 secondes
    SerialMon.print(".");
    delay(1000);
  }
  SerialMon.println("o");
  SerialMon.println("GSM GPS Tracker v."+String(_VERSION));

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

    // Set GSM module baud rate
  SerialAT.begin(115200, SERIAL_8N1, RX1, TX1, false);

  if (!modem.testAT())
    activateModem(1750);

  SerialMon.println("Wait 1sec ...");
  delay(1000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.print("Initializing modem...");
  if (performModemInit()) SerialMon.println(" Success!");
  else SerialMon.println(" Failed!"); //Devrait pas arriver...


  while (!connectGPRSNetwork()) {
    SerialMon.print(".");
  }

  //Set RTC from NTP:
  SerialMon.print("Get RTC (NTP) ready:");
  if (initGSMNTP()) {
    SerialMon.println(" Done.");
    if (getGSMNTP(3,true)) {
      SerialMon.println("NTP sync OK");
    } else
      SerialMon.println("NTP sync Failed");
  } else 
    SerialMon.println(" NTP init Failed.");

  SerialMon.println("Play time! GPS first");

  // MQTT Broker setup
  //localMqtt.setServer(brokerShiftr, 1883);
  localMqtt.setServer(brokerTBCAL, 1883);

  gpsTimer = -30;

}

// 24 juin 2024: à retenir, module REBOOT:   AT+CREBOOT

void loop() {
  if (millis() - changeModeTimer > changeModeDelay && currentMode == GPS) {
    changeModeTimer = millis();
    if (gpsDataValid) { //Seulement si le data GPS est valide
      SerialMon.println("Switching mode to senddata. Turning GPS off");
      changeModeDelay = modeSendData;
      currentMode = SENDDATA;
      enableGPS(false);
      gpsEnabled = false;
      SerialMon.print("Sending +CFUN AT cmd (+8sec delay) ");
      modem.sendAT("+CFUN=1,1");
      for (int i=0; i<8; i++) {  //Boucle de délais de 8 secondes
        SerialMon.print(".");
        delay(1000);
      }
      SerialMon.println("o");
    }
  }
  // Mise en hold car devrait plutôt être au module GSM d'indiquer lorsqu'il a envoyé le data et de retourner en mode GPS pour l'acquisition
  // if (millis() - changeModeTimer > changeModeDelay && currentMode == SENDDATA) {
  //   changeModeTimer = millis();
  //   changeModeDelay = modeGPS;
  //   currentMode = GPS;
  //   disableGNSS();
  // }
  if (currentMode == GPS) {
    if (millis() - gpsTimer > gpsTimerDelay) {
      gpsTimer = millis();
      if(gpsEnabled) {
        static gpsDataStruc dataLoc;
        if (parseGPSdata(dataLoc,false)) {
          SerialMon.println("GPS data parsed successfully");
          if (dataLoc.latitude != 0.0) { //Si la position n'est pas 0,0, on considérera qu'elle est plausible...
            digitalWrite(LED_PIN, HIGH);
            currentPos[0]=dataLoc.latitude;
            currentPos[1]=dataLoc.longitude;
            gpsDataValid = true;
            gpsAcquisitionFailed=0;
          }
        } else {
           SerialMon.println("GPS data parsed FAILED");
           gpsAcquisitionFailed++;
           digitalWrite(LED_PIN, LOW);
           if (gpsAcquisitionFailed>4 && gpsDataValid) gpsDataValid=false;
           if (gpsAcquisitionFailed == 6) {
             //Something wrong, let's try recycling:
             enableGPS(false);
             delay(2000);
             enableGPS(true);
             delay(1000);
           }
           if (gpsAcquisitionFailed == 10) {
             //Hot start
             enableGPS(false);
             delay(1000);
             modem.sendAT("+CGNSHOT");
             delay(1000);
             enableGPS(true);
             delay(1000);
           }
           if (gpsAcquisitionFailed == 14) {
             //Cold start
             enableGPS(false);
             delay(1000);
             modem.sendAT("+CGNSWARM");
             delay(1000);
             enableGPS(true);
             delay(1000);
           }
           if (gpsAcquisitionFailed >= 18) {
             //Cold start
             enableGPS(false);
             delay(1000);
             modem.sendAT("+CGNSCOLD");
             delay(1000);
             enableGPS(true);
             delay(1000);
             gpsAcquisitionFailed=0; //Start over
           }
        }
      } else {
        //Try again enabling GPS feature
        digitalWrite(LED_PIN, LOW);
        gpsDataValid=false;
        gpsAcquisitionFailed=0;
        if (enableGPS(true))
          gpsEnabled = true;
      }
    }
  }
  if (currentMode == SENDDATA) {
    if (gpsDataValid) {
      localMqtt.loop();

      if (connectMQTTBroker(&localMqtt,1)) {
        localMqtt.loop();
        String dateTime;
        dateTime = getDateTimeStr();
        SerialMon.println("Heure: "+dateTime);

        // Calcul distance p/r au point précédent:
        unsigned long distFromLastPt = 0;
        if (previousPos[0] != 0.0 && previousPos[1] != 0.0) // If previous not set, do not calculate. Pourrait aussi etre selon le iterationCounter
          distFromLastPt = (unsigned long)TinyGPSPlus::distanceBetween(currentPos[0],currentPos[1],previousPos[0],previousPos[1]);

// Standard pour envoyer a shiftr.io:
        // JsonDocument doc;
        // String content;
        // doc["IC"] = iterationCounter++;
        // doc["fwver"] = String(_VERSION);
        // doc["heure"] = dateTime;
        // doc["uptime"] = millis()/1000;
        // doc["latitude"] = currentPos[0];
        // doc["longitude"] = currentPos[1];
        // doc["distFromPrev"] = distFromLastPt;
        // doc["operator"] = modem.getOperator();
        // doc["IP"] = modem.getLocalIP();
        // doc["sigQual"] = modem.getSignalQuality();
        // serializeJson(doc,content);

// Json document telemetry et attribute pour TB:
        JsonDocument telemetry;
        JsonDocument attribute;
        String telemetryStr;
        String attribStr;
        telemetry["IC"] = iterationCounter++;
        attribute["fwver"] = String(_VERSION);
        attribute["heure"] = dateTime;
        telemetry["uptime"] = millis()/1000;
        telemetry["latitude"] = currentPos[0];
        telemetry["longitude"] = currentPos[1];
        telemetry["distFromPrev"] = distFromLastPt;
        attribute["operator"] = modem.getOperator();
        attribute["IP"] = modem.getLocalIP();
        telemetry["sigQual"] = modem.getSignalQuality();
        serializeJson(telemetry,telemetryStr);
        serializeJson(attribute,attribStr);

        if (localMqtt.publish(telemetryTopic, telemetryStr.c_str())) {
          localMqtt.loop();
          SerialMon.print("Position telemetry data sent success: ");
          SerialMon.println(telemetryStr);
          if (localMqtt.publish(attributeTopic, attribStr.c_str())) {
            SerialMon.print("Position attribute data sent success: ");
            SerialMon.println(attribStr);
            localMqtt.loop();
          }

          changeModeTimer = millis();
          changeModeDelay = modeGPS;
          currentMode = GPS;
          localMqtt.disconnect();
          localMqtt.loop();
          SerialMon.println("Disconnecting MQTT");
          previousPos[0] = currentPos[0];
          previousPos[1] = currentPos[1];
        } else
          SerialMon.println("Failure sending Position");
      } else {
        connectGPRSNetwork();
      }
    } else {
      SerialMon.println("Sending invalid data??!! What the heck??");
      changeModeTimer = millis();
      changeModeDelay = modeGPS;
      currentMode = GPS;
    }
  }
}
//-----------------------------------------------------------------------
// TGP CAL - Modele v1.2.1 - Avril 2024
/* Version de modèle: B - Yh H23 */