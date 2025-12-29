// NS-DSPL-CDU v1.2-BETA1
// Decoder DCC per 4 motori Circuitron Toortoise,
//                 2 passaggi a livello,
//                 4 sensori (TOF, HALL EFFECT E ASSORBIMENTO,
//                 2 uscite led)
// Configurazione tramite CV
// NeXtorSystem 2025
// Gabriele Manfreda
//
// Ultimo Aggiornamento: 28/12/2025
// Ultima modifica: Introduzione supporto sensori assorbimento tipo zht103
#include <avr/wdt.h>
#include <Adafruit_MCP23X17.h>
#include <VL53L0X.h>
#include <LocoNet.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_TiCoServo.h>

//#define DEBUG
//#define VERSION         "1.0"
#define IOPINS          9
//#define CONFIGVALID     111
#define CMDLINESIZE     100
#define DEV_TURNOUT     0
#define DEV_SENSOR      1
#define DEV_SENSORTOF   2
#define DEV_MSENSOR     3
#define DEV_PLIVELLO    4
#define DEV_SENSORABS   5
//#define SWITCH_MS       200

#define PLIV_MOT1_PIN   5
#define PLIV_MOT2_PIN   9

#define ON_DISTANCE     50
#define THRESHOLD       5
const uint16_t DEFAULT_ZHT103_THRESHOLD = 400; // soglia analogica per il nuovo sensore
const uint8_t SENSOR_COUNT = IOPINS;
const uint8_t HALL_TOLERANCE = 30;
const unsigned long SENSOR_MIN_INTERVAL = 50;
const unsigned long SENSOR_ERROR_RETRY_MS = 1000;

static const uint8_t analog_pins[] = {A0,A1,A2,A3};
constexpr uint8_t ANALOG_PIN_COUNT = sizeof(analog_pins) / sizeof(analog_pins[0]);
const uint8_t xshut_pins[] = {7,10,11,12};
const uint8_t vl53l0xCount = 4;

/******* Inizio Dichiarazione Indirizzo e Valori di default delle CV *******/

// const uint16_t SV_ADDR_SW_VERSION = 2 ;       
const uint8_t VALUE_SW_VERSION = 1 ;
const char VALUE_SW_REV[] = "2-BETA1";
const char MANUFACTURER[] = "NeXtorSystem" ;
const char ENGINEER[] = "Gabriele Manfreda" ;

// const uint16_t SV_ADDR_NODE_ID_L = 3 ;  
// const uint16_t SV_ADDR_NODE_ID_H = 4 ;   
const uint8_t VALUE_NODE_ID_L = 0 ;                 // default address = 0
const uint8_t VALUE_NODE_ID_H = 0 ;                 // address = SV_ADDR_NODE_ID_H * 256 + SV_ADDR_NODE_ID_L

//  SV_ADDR_SERIAL_NUMBER_L = 5,
//  SV_ADDR_SERIAL_NUMBER_H = 6,
const uint8_t VALUE_SERIAL_NUMBER_L = 1 ;           // default value = 1
const uint8_t VALUE_SERIAL_NUMBER_H = 0 ;           // address = SV_ADDR_NODE_ID_H * 256 + SV_ADDR_NODE_ID_L

// const uint16_t SV_ADDR_USER_BASE = 7 ;  

const uint8_t SV_ADDR_MANUFACTURER_ID = 8 ;
const uint8_t VALUE_MANUFACTURER_ID = 13 ;
const uint8_t SV_ADDR_DEVELOPER_ID = 9 ;
const uint8_t VALUE_DEVELOPER_ID = 21 ;
const uint8_t SV_ADDR_PRODUCT_ID_L = 10 ;
const uint8_t VALUE_PRODUCT_ID_L = 1 ;
const uint8_t SV_ADDR_PRODUCT_ID_H = 11 ;
const uint8_t VALUE_PRODUCT_ID_H = 0 ;

const uint8_t SV_ADDR_NUM_SENSORS = 12 ;            // Indirizzo EEPROM della variabile del numero di sensori abilitati
const uint8_t VALUE_NUM_SENSORS = 4 ;               // Numero di Sensori abilitati

const uint16_t SV_ADDR_ADDR_SENSORS_L = 13 ;   
const uint16_t SV_ADDR_ADDR_SENSORS_H = 14 ;  
const uint8_t VALUE_ADDR_SENSORS_L = 1 ;            // Sensors default address = 257
const uint8_t VALUE_ADDR_SENSORS_H = 1 ;            // address = SV_ADDR_ADDR_SENSORS_H * 256 + SV_ADDR_ADDR_SENSORS_L

const uint8_t SV_ADDR_CHANGE_ID_L = 15 ;            // default address = 1
const uint8_t SV_ADDR_CHANGE_ID_H = 16 ;            // address = SV_ADDR_NODE_ID_H * 256 + SV_ADDR_NODE_ID_L
const uint8_t VALUE_CHANGE_ID_L = 0 ;               // default address = 1
const uint8_t VALUE_CHANGE_ID_H = 0 ;               // address = SV_ADDR_NODE_ID_H * 256 + SV_ADDR_NODE_ID_L

const uint8_t SV_ADDR_AUXILIARY = 17 ;
const uint8_t VALUE_AUXILIARY = 7 ;

const uint8_t SV_ADDR_SENSOR1_LNADR = 20 ;
const uint8_t SV_ADDR_SENSOR1_ISINPUT = 21 ;
const uint8_t SV_ADDR_SENSOR1_TYPE = 22 ;
const uint8_t SV_ADDR_SENSOR1_ENABLED = 23 ;

const uint8_t SV_ADDR_SENSOR2_LNADR = 24 ;
const uint8_t SV_ADDR_SENSOR2_ISINPUT = 25 ;
const uint8_t SV_ADDR_SENSOR2_TYPE = 26 ;
const uint8_t SV_ADDR_SENSOR2_ENABLED = 27 ;

const uint8_t SV_ADDR_SENSOR3_LNADR = 28 ;
const uint8_t SV_ADDR_SENSOR3_ISINPUT = 29 ;
const uint8_t SV_ADDR_SENSOR3_TYPE = 30 ;
const uint8_t SV_ADDR_SENSOR3_ENABLED = 31 ;

const uint8_t SV_ADDR_SENSOR4_LNADR = 32 ;
const uint8_t SV_ADDR_SENSOR4_ISINPUT = 33 ;
const uint8_t SV_ADDR_SENSOR4_TYPE = 34 ;
const uint8_t SV_ADDR_SENSOR4_ENABLED = 35 ;

const uint8_t SV_ADDR_PLIV_LNADR = 36 ;
const uint8_t SV_ADDR_PLIV_ISINPUT = 37 ;
const uint8_t SV_ADDR_PLIV_TYPE = 38 ;
const uint8_t SV_ADDR_PLIV_ENABLED = 39 ;

const uint8_t SV_ADDR_MOT1_LNADR = 40 ;
const uint8_t SV_ADDR_MOT1_ISINPUT = 41 ;
const uint8_t SV_ADDR_MOT1_TYPE = 42 ;
const uint8_t SV_ADDR_MOT1_ENABLED = 43 ;

const uint8_t SV_ADDR_MOT2_LNADR = 44 ;
const uint8_t SV_ADDR_MOT2_ISINPUT = 45 ;
const uint8_t SV_ADDR_MOT2_TYPE = 46 ;
const uint8_t SV_ADDR_MOT2_ENABLED = 47 ;

const uint8_t SV_ADDR_MOT3_LNADR = 48 ;
const uint8_t SV_ADDR_MOT3_ISINPUT = 49 ;
const uint8_t SV_ADDR_MOT3_TYPE = 50 ;
const uint8_t SV_ADDR_MOT3_ENABLED = 51 ;

const uint8_t SV_ADDR_MOT4_LNADR = 52 ;
const uint8_t SV_ADDR_MOT4_ISINPUT = 53 ;
const uint8_t SV_ADDR_MOT4_TYPE = 54 ;
const uint8_t SV_ADDR_MOT4_ENABLED = 55 ;

const uint8_t SV_ADDR_SENSOR1_SENSDISTANCE = 56 ;   // default value = 50 mm
const uint8_t SV_ADDR_SENSOR2_SENSDISTANCE = 57 ;   // default value = 50 mm
const uint8_t SV_ADDR_SENSOR3_SENSDISTANCE = 58 ;   // default value = 50 mm
const uint8_t SV_ADDR_SENSOR4_SENSDISTANCE = 59 ;   // default value = 50 mm

const uint8_t SV_ADDR_SENSOR1_ZHTTHRESHOLD = 60 ;   // default value = 400 (ADC)
const uint8_t SV_ADDR_SENSOR2_ZHTTHRESHOLD = 61 ;   // default value = 400 (ADC)
const uint8_t SV_ADDR_SENSOR3_ZHTTHRESHOLD = 62 ;   // default value = 400 (ADC)
const uint8_t SV_ADDR_SENSOR4_ZHTTHRESHOLD = 63 ;   // default value = 400 (ADC)

const uint8_t SV_ADDR_HW_VERSION = 99 ;   // default value = 1
const uint8_t SV_ADDR_DEBUG_FLAG = 100 ;
const uint8_t VALUE_DEBUG_DEFAULT = 0 ;

// address we will assign if dual sensor is present
#define LOX1_ADDRESS     0x30
//#define LOX2_ADDRESS     0x31

// set the pins to shutdown
//#define SHT_LOX1            7
//#define SHT_LOX2           10

#define LNET_TX_PIN          8
#define POS_PLIV1           A4
#define POS_PLIV2           A5
#define LED_PLIV             6

// objects for the vl53l0x
//Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
//Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
//VL53L0X_RangingMeasurementData_t measure1;
//VL53L0X_RangingMeasurementData_t measure2;

typedef struct {
  uint8_t dccAddress;  //int
  uint8_t isInput;      //bool
  uint8_t deviceType;   //int
  uint8_t isEnabled;
} PinInfo;

// Variabili Globali
Adafruit_MCP23X17 mcp1;
Adafruit_TiCoServo servo[2];
// Servo servo1PLIV;
// Servo servo2PLIV;

VL53L0X sensorTOF[vl53l0xCount];

LocoNetSystemVariableClass LocoNetSV;
lnMsg *LnPacket;
SV_STATUS   svStatus = SV_OK;
boolean     deferredProcessingNeeded = false;

//uint16_t sensorsAddress ;
//uint8_t  sensorsNumber ;

PinInfo pinInfo[IOPINS];
uint16_t sensorStateMask = 0; // bitmask per ridurre RAM e accessi

bool isPinStraight[IOPINS];
char cmdline[CMDLINESIZE];
bool debugEnabled = false;
bool failedStart = false;
uint8_t cmdlinepos = 0;
uint8_t aperto = 109;
uint8_t chiuso = 9;
int POS_PLIV1_VAL;
int sensDistance[vl53l0xCount];
uint16_t zhtThreshold[vl53l0xCount];
int hallSensorSTDVAL = 543;
unsigned long sensorNextDue[SENSOR_COUNT];
uint8_t sensorScanCursor = 0;
int8_t hallSensorLastState[SENSOR_COUNT];
const uint8_t tortoiseDirectionPins[][3] = {
  {8, 9, 4},   // Motore Tortoise #1
  {10, 11, 5}, // Motore Tortoise #2
  {12, 13, 6}, // Motore Tortoise #3
  {14, 15, 7}  // Motore Tortoise #4
};
const uint8_t tortoiseEnablePins[] = {0, 1, 2, 3};

inline bool isSensorOn(uint8_t idx) {
  if (idx >= 16) return false;
  return (sensorStateMask & (uint16_t(1u) << idx)) != 0;
}

inline void setSensorState(uint8_t idx, bool on) {
  if (idx >= 16) return;
  uint16_t bit = (uint16_t(1u) << idx);
  if (on) sensorStateMask |= bit;
  else    sensorStateMask &= (uint16_t)~bit;
}

void configureXshutForSensor(uint8_t activeIndex) {
  for (uint8_t pin = 0; pin < vl53l0xCount; ++pin) {
    digitalWrite(xshut_pins[pin], pin <= activeIndex ? HIGH : LOW);
  }
  delay(10);
}

bool initializeTofSensor(uint8_t index) {
  configureXshutForSensor(index);
  sensorTOF[index].setTimeout(200);
  sensorTOF[index].setMeasurementTimingBudget(50000);
  if (!sensorTOF[index].init()) {
    return false;
  }
  sensorTOF[index].setAddress(0x2A + index);
  return true;
}

void publishSensorState(uint8_t index, bool active, const __FlashStringHelper *sensorName, uint16_t measurement = 0, bool includeValue = false) {
  setSensorState(index, active);
  LocoNet.reportSensor(pinInfo[index].dccAddress, active ? 1 : 0);
  if (debugEnabled) {
    Serial.print(sensorName);
    Serial.print(index + 1);
    Serial.print(F(" Sent sensor state - address: "));
    Serial.print(pinInfo[index].dccAddress);
    Serial.print(F(", state: "));
    Serial.print(active ? F("ON") : F("OFF"));
    if (includeValue) {
      Serial.print(F(" - Valore: "));
      Serial.print(measurement);
    }
    Serial.println();
  }
}

void publishHallState(uint8_t index, uint8_t state, uint16_t analogValue, const __FlashStringHelper *sensorName) {
  hallSensorLastState[index] = state;
  LocoNet.reportSensor(pinInfo[index].dccAddress, state);
  if (debugEnabled) {
    Serial.print(sensorName);
    Serial.print(index + 1);
    Serial.print(F(" Sent sensor state - address: "));
    Serial.print(pinInfo[index].dccAddress);
    Serial.print(F(", state: "));
    Serial.print(state);
    Serial.print(F(" - Valore: "));
    Serial.println(analogValue);
  }
}

void setup() {
  delay(2500);

  uint8_t adrLo = LocoNetSV.readSVStorage(SV_ADDR_CHANGE_ID_L) ;
  uint8_t adrHi = LocoNetSV.readSVStorage(SV_ADDR_CHANGE_ID_H) ;
  uint8_t loconetADR = (adrHi * 256 + adrLo);

  Serial.begin(115200);
  Serial.print(F("NS-DSPL-CDU v"));
  Serial.print(VALUE_SW_VERSION);
  Serial.print(F("."));
  Serial.println(VALUE_SW_REV);
  Serial.print(F("Data Revisione: "));
  Serial.println(F(__DATE__));
  Serial.print(F("Produttore: "));
  Serial.println(MANUFACTURER);
  Serial.print(F("Engineer: "));
  Serial.println(ENGINEER);
  Serial.println();
  Serial.print(F("Indirizzo Loconet: "));
  Serial.println(loconetADR);
  Serial.println();

  Wire.begin();

  uint8_t storedDebug = LocoNetSV.readSVStorage(SV_ADDR_DEBUG_FLAG);
  debugEnabled = (storedDebug != 0);
  Serial.print(F("Debug seriale: "));
  Serial.println(debugEnabled ? F("ABILITATO") : F("DISABILITATO"));

  // check if the configuration in the EEPROM is valid
  uint8_t auxiliary = LocoNetSV.readSVStorage(SV_ADDR_AUXILIARY) ;
  if (auxiliary != VALUE_AUXILIARY) {
    setFactoryDefault();
  } else {
    readConfigFromStorage();
  }

  LocoNet.init(LNET_TX_PIN);
  uint16_t productId = VALUE_PRODUCT_ID_L + 256 * VALUE_PRODUCT_ID_H ;
  LocoNetSV.init(VALUE_MANUFACTURER_ID, VALUE_DEVELOPER_ID, productId, VALUE_SW_VERSION);
  Serial.println(F("- Bus LocoNet avviato correttamente"));

  initLocalVariables() ;

  mcp1.begin_I2C(0x20);
  Serial.println("- MCP23017 avviato correttamente");

  //Inizializzo Pin GBA0-GBA7 MCP23017 come OUTPUT
  for (uint8_t i = 0; i < 8; i++) {
    mcp1.pinMode(i, OUTPUT);
    mcp1.digitalWrite(i, LOW);
  }  

  //Inizializzo i pin legati alle CV
  for (uint8_t i = 0; i < IOPINS; i++) {
    isPinStraight[i] = true;
    setPinDirection(i, pinInfo[i].isInput);
    if (!pinInfo[i].isInput) writePinOutput(i, LOW);
  } 
  //for (int i = 0; i < IOPINS; i++) if (!pinInfo[i].isInput) writePinOutput(i, LOW);

  pinMode(POS_PLIV1, INPUT);
  pinMode(POS_PLIV2, INPUT);
  pinMode(PLIV_MOT1_PIN, OUTPUT);
  pinMode(PLIV_MOT2_PIN, OUTPUT);
  pinMode(LED_PLIV, OUTPUT);

  for (uint8_t i = 0; i < vl53l0xCount; i++) {
    pinMode(xshut_pins[i], OUTPUT);
    digitalWrite(xshut_pins[i], LOW);
  }
  for (uint8_t i = 0; i < vl53l0xCount; i++) digitalWrite(xshut_pins[i], HIGH);
  delay(10);

  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < vl53l0xCount; i++) {    
    if (pinInfo[i].deviceType == DEV_SENSORTOF && pinInfo[i].isEnabled) {
      if (!initializeTofSensor(i)) {
        Serial.print(F("!!! Impossibile inizializzare il sensore #"));
        Serial.print(i + 1);
        Serial.println(F(" !!!"));
        failedStart = true;
      }
    }
  }

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    sensorNextDue[i] = 0;
    hallSensorLastState[i] = -1;
  }

  Serial.println();
  Serial.println(F("* Invia HELP per ottenere l'elenco dei comandi *"));
  Serial.println();

  if (failedStart) Serial.println(F("Avvio completato con errori"));
  else Serial.println(F("Avvio completato con successo!"));
  Serial.println();

  POS_PLIV1_VAL = analogRead(POS_PLIV1);
  POS_PLIV1_VAL = map(POS_PLIV1_VAL, 109, 400, chiuso, aperto);

  if (pinInfo[4].isEnabled) {
    servo[0].write(POS_PLIV1_VAL);
    servo[1].write(POS_PLIV1_VAL);
    servo[0].attach(5);
    servo[1].attach(9);
  }

}

void checkSerial () {
  while (Serial.available() > 0) {

    int inByte = Serial.read();

    // Ignora ritorni carrello per compatibilità con CRLF
    if (inByte == '\r') continue;

    if (inByte == '\n') {
      if (cmdlinepos > 0) {
        cmdline[cmdlinepos] = '\0';
        parseCmdLine();
      }
      cmdlinepos = 0;
      continue;
    }

    if (cmdlinepos < CMDLINESIZE - 1) {
      cmdline[cmdlinepos] = inByte;
      cmdlinepos++;
    } else {
      // Protegge da overflow del buffer
      cmdlinepos = 0;
      Serial.println(F("Comando troppo lungo, input ignorato"));
    }
  }
}

void checkLocoNet() {
  LnPacket = LocoNet.receive();
  if (LnPacket) {
    svStatus = LocoNetSV.processMessage(LnPacket);
    
    deferredProcessingNeeded = (svStatus == SV_DEFERRED_PROCESSING_NEEDED);

    if (deferredProcessingNeeded) {
      int counter = 11 ;
      do {
        counter-- ;
        deferredProcessingNeeded = (LocoNetSV.doDeferredProcessing() != SV_OK) ;
       
      } while ((deferredProcessingNeeded) && (counter > 0)) ;
    }
   
    LocoNet.processSwitchSensorMessage(LnPacket);
  }
}

void loop() {
  // Controllo dati su interfaccia seriale
  checkSerial();

  // Controllo bus LocoNet
  checkLocoNet();  

  // Controllo dei sensori
  serviceSensors();
}

void initLocalVariables() {

  uint8_t changeLo = LocoNetSV.readSVStorage(SV_ADDR_CHANGE_ID_L) ;
  uint8_t changeHi = LocoNetSV.readSVStorage(SV_ADDR_CHANGE_ID_H) ;
  LocoNetSV.writeSVStorage(SV_ADDR_NODE_ID_H, changeHi) ;
  LocoNetSV.writeSVStorage(SV_ADDR_NODE_ID_L, changeLo) ;
  
}

void notifySwitchRequest(uint16_t Address, uint8_t Output, uint8_t Direction) {
  if (debugEnabled) {
    Serial.print(("Received switch command - "));
    Serial.print(("address: ")); Serial.print(Address);
    Serial.print((", direction: ")); if (Direction > 0) Serial.println(("DIVERGING")); else Serial.println(("STRAIGHT"));
    Serial.println(Direction);
  }
  for (int i = 0; i < IOPINS; i++) {
    if (pinInfo[i].dccAddress == Address && (pinInfo[i].deviceType == DEV_TURNOUT || pinInfo[i].deviceType == DEV_PLIVELLO)) {
      isPinStraight[i] = (Direction == 0);

      // Aziona passaggio a livello
      if (i == 4 && !pinInfo[i].isInput) {
        slowMoveServo(30, Direction);
        digitalWrite(LED_PLIV, Direction);
      }

      if (i >= 5 && i <= 8 && !pinInfo[i].isInput && pinInfo[i].isEnabled) {
        const uint8_t motorIndex = i - 5;
        const uint8_t *directionPins = tortoiseDirectionPins[motorIndex];
        mcp1.digitalWrite(tortoiseEnablePins[motorIndex], HIGH);
        delay(10);
        const bool isStraight = (Direction == 0);
        mcp1.digitalWrite(directionPins[0], isStraight ? LOW : HIGH);
        mcp1.digitalWrite(directionPins[1], isStraight ? HIGH : LOW);
        mcp1.digitalWrite(directionPins[2], isStraight ? HIGH : LOW);
      }
    }
  }
}

// set pin direction
void setPinDirection(uint8_t p, bool isInput) {
    //Serial.print("Pin: "); Serial.print(p); Serial.print("TIPO: "); Serial.println(isInput);
  if (!isInput) {
    if (p == 4) pinMode(5, OUTPUT);
    //if (p == 5) pinMode(9, OUTPUT);
    if (p == 5) { mcp1.pinMode(8, OUTPUT); mcp1.pinMode(9, OUTPUT); }
    if (p == 6) { mcp1.pinMode(10, OUTPUT); mcp1.pinMode(11, OUTPUT); }
    if (p == 7) { mcp1.pinMode(12, OUTPUT); mcp1.pinMode(13, OUTPUT); }
    if (p == 8) { mcp1.pinMode(14, OUTPUT); mcp1.pinMode(15, OUTPUT); }
  } else if (p < ANALOG_PIN_COUNT) {
    if (pinInfo[p].deviceType == DEV_SENSORABS) {
      pinMode(analog_pins[p], INPUT);
    } else if (pinInfo[p].deviceType == DEV_SENSOR) {
      pinMode(analog_pins[p], INPUT);
    } else {
      pinMode(analog_pins[p], INPUT_PULLUP);
    }
  }
}

// write output
void writePinOutput(uint8_t p, uint8_t d) {
  if (p < 5) digitalWrite(p, d);
  if (p == 5) mcp1.digitalWrite(8, d);
  if (p == 6) mcp1.digitalWrite(10, d);
  if (p == 7) mcp1.digitalWrite(12, d);
  if (p == 8) mcp1.digitalWrite(14, d);
}

// write MCP output
/*void writeMCPPinOutput(uint8_t p, uint8_t d) {
  mcp1.digitalWrite(p, d);
}*/

void serviceSensors() {
  unsigned long now = millis();

  if (failedStart) {
    static unsigned long nextErrorReport = 0;
    if (now - nextErrorReport >= SENSOR_ERROR_RETRY_MS) {
      LocoNet.reportSensor(999, 1);
      nextErrorReport = now;
    }
    return;
  }

  for (uint8_t attempt = 0; attempt < SENSOR_COUNT; attempt++) {
    uint8_t idx = (sensorScanCursor + attempt) % SENSOR_COUNT;
    if (!pinInfo[idx].isInput || !pinInfo[idx].isEnabled) continue;

    if (now < sensorNextDue[idx]) continue;

    checkSensor(idx);
    sensorNextDue[idx] = now + SENSOR_MIN_INTERVAL;
    sensorScanCursor = (idx + 1) % SENSOR_COUNT;
    break; // gestisce al massimo un sensore per iterazione del loop
  }
}

void checkSensor(uint8_t i) {
  if (!pinInfo[i].isInput || !pinInfo[i].isEnabled) return;

  switch (pinInfo[i].deviceType) {
    case DEV_SENSORTOF: {
      if (i >= vl53l0xCount) return;
      const uint16_t distance = sensorTOF[i].readRangeSingleMillimeters();
      if (sensorTOF[i].timeoutOccurred()) return;

      const uint16_t target = sensDistance[i];
      const bool currentlyOn = isSensorOn(i);

      if (distance <= target - THRESHOLD && !currentlyOn) {
        publishSensorState(i, true, F("VL53L0X #"), distance, true);
      } else if (distance >= target + THRESHOLD && currentlyOn) {
        publishSensorState(i, false, F("VL53L0X #"), distance, true);
      }
      break;
    }

    case DEV_SENSOR: {
      if (i >= ANALOG_PIN_COUNT) return;
      const uint16_t analogValue = analogRead(analog_pins[i]);

      if (debugEnabled) {
        Serial.print(F("Analog Value HALL Sensor #"));
        Serial.print(i + 1);
        Serial.print(F(": "));
        Serial.println(analogValue);
      }

      uint8_t state = 0;
      if (analogValue < (hallSensorSTDVAL - HALL_TOLERANCE)) state = 1;
      else if (analogValue > (hallSensorSTDVAL + HALL_TOLERANCE)) state = 2;

      if (hallSensorLastState[i] != state) {
        publishHallState(i, state, analogValue, F("HALLSENSOR #"));
      }
      break;
    }

    case DEV_MSENSOR: {
      if (i >= ANALOG_PIN_COUNT) return;
      const bool active = (digitalRead(analog_pins[i]) == LOW);
      if (active != isSensorOn(i)) publishSensorState(i, active, F("Sensore #"));
      break;
    }

    case DEV_SENSORABS: {
      if (i >= ANALOG_PIN_COUNT) return;
      const uint16_t analogValue = analogRead(analog_pins[i]);
      const uint16_t threshold = zhtThreshold[i] > 0 ? zhtThreshold[i] : DEFAULT_ZHT103_THRESHOLD;
      const bool active = analogValue >= (threshold + THRESHOLD);
      const bool inactive = analogValue <= (threshold - THRESHOLD);

      if (active && !isSensorOn(i)) {
        publishSensorState(i, true, F("ABSSENSOR #"), analogValue, true);
      } else if (inactive && isSensorOn(i)) {
        publishSensorState(i, false, F("ABSSENSOR #"), analogValue, true);
      }
      break;
    }

    default:
      break;
  }
}

void slowMoveServo(int delayTime, int to) {
  servo[0].attach(5);
  servo[1].attach(9);
  delay(10);
  // servo1PLIV.attach(5);
  // servo2PLIV.attach(9);
  POS_PLIV1_VAL = analogRead(POS_PLIV1);                            // reads the value of the potentiometer (value between 0 and 1023)
  POS_PLIV1_VAL = map(POS_PLIV1_VAL, 109, 420, 14, 124);     // scale it for use with the servo (value between 0 and 180)
  //POS_PLIV1_VAL = map(POS_PLIV1_VAL, 109, 400, chiuso, aperto);     // scale it for use with the servo (value between 0 and 180)
  
  if (to == 0) to = 12;
  else to = aperto;

  if (POS_PLIV1_VAL == to) return;  // Nothing to do if it's already there

  else if (to > POS_PLIV1_VAL) {
    for (int i = POS_PLIV1_VAL; i < to + 1; i++) {
      servo[0].write(i);
      servo[1].write(i);
      // servo1PLIV.write(i);
      // servo2PLIV.write(i);
      delay(delayTime);
    }
  }
  else {
    for (int i = POS_PLIV1_VAL; i > to - 1; i--) {
      servo[0].write(i);
      servo[1].write(i);
      // servo1PLIV.write(i);
      // servo2PLIV.write(i);
      delay(delayTime);
    }
    // servo1PLIV.detach();
    // servo2PLIV.detach();
    servo[0].detach();
    servo[1].detach();
  }
}
