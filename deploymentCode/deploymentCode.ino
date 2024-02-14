//UART Communication
#include <SoftwareSerial.h>

//Barometers
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_DPS310.h>

#include <string.h>

#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

//IMU
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Define the RX and TX pins for your SoftwareSerial connection
const int rxPin = 2;
const int txPin = 3;

// SD Card
// const int svPin = 11;
// const int sdPin = 10;

// Define the digital pins for the deployment relays
const int rly1 = 7;
const int rly2 = 8;

// Define the digital pin for the buzzer
const int buzz = 4;

const bool TEST = true;

// Create a SoftwareSerial object
SoftwareSerial rfSerial(rxPin, txPin);

const byte numChars = 32;
char receivedChars[numChars];

boolean newData = false;

enum { Pad,
       Ascent,
       Descent };

unsigned long startTime;
unsigned char deployState;

const float DEPLOY_ALTI = 800 * 0.3048;
const float TOL_GROUND = 20;
const int AVG_COUNT = 50;

//Adafruit_MPL3115A2 mpl;
Adafruit_DPS310 dps;

Adafruit_BNO055 imuBNO = Adafruit_BNO055(55);
File sdFile;
//float ground_alti = 0;
float ground_pressure = 0;
float alti;

void setup() {
  pinMode(buzz, OUTPUT);
  for (int i = 0; i < 1; i++) {
    digitalWrite(buzz, HIGH);
    delay(500);
    digitalWrite(buzz, LOW);
    delay(500);
  }

  // Start the serial communication with the Arduino IDE Serial Monitor
  Serial.begin(57600);
  Serial.print("Serial set up");
  //Start the serial communication with the RFX 900x modem
  rfSerial.begin(57600);
  rfSerial.println("RF set up");

  // rfSerial.println("Init SD Card");
  // pinMode(sdPin, OUTPUT);
  // if (!SD.begin(10)) {
  //   rfSerial.println("Init Failed!");
  //   return;
  // }

  rfSerial.println("Init Done.");
  // sdFile = SD.open("log_file.txt", FILE_WRITE);

  //mplSetup();
  dpsSetup();
  // imuSetup();

  //MPL Ground Zero
  // for(int i=1; i<=AVG_COUNT; i++){
  //   ground_alti = ground_alti*(i-1)/i + mpl.getAltitude()/i;
  // }

  //DPS Ground Zero
  sensors_event_t temp_event, pressure_event;
  for (int i = 1; i <= AVG_COUNT; i++) {
    dps.getEvents(&temp_event, &pressure_event);

    //Ground Altitude Approach
    //ground_alti = ((ground_alti * (i - 1)) / i) + (dps.readAltitude(1013.25) / i);

    //Ground Pressure Approach
    ground_pressure = ((ground_pressure * (i - 1)) / i) + (pressure_event.pressure / i);
  }

  //Make the relay's digital pins outputs
  pinMode(rly1, OUTPUT);
  pinMode(rly2, OUTPUT);

  rfSerial.println("Arduino is ready");
  deployState = Pad;

  startTime = millis();
}

void loop() {
  //alti = mpl.getAltitude();

  alti = dps.readAltitude(ground_pressure);

  // sensors_event_t event;
  // imuBNO.getEvent(&event);
  // imu::Quaternion quat = imuBNO.getQuat();
  // imu::Vector<3> linacce = imuBNO.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  // imu::Vector<3> gvec = imuBNO.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  //print2RF(String(alti));
  //Serial.println(alti);
  // print2RF(String(linacce.x())+" "+linacce.y()+" "+linacce.z());
  // print2RF(String(quat.w())+" "+quat.x()+" "+quat.y()+" "+quat.z());
  // print2RF(String(gvec.x())+" "+gvec.y()+" "+gvec.z());

  getCommand();
  switch (deployState) {

    case Pad:
      if (alti > TOL_GROUND || TEST && newData == true && (strcmp(receivedChars, "ascent") == 0)) {
        deployState = Ascent;
        print2RF("Ascent in action...");
        Serial.println("Ascent in action");
      }
      break;

    case Ascent:
      if (alti > DEPLOY_ALTI || TEST && newData == true && (strcmp(receivedChars, "desc") == 0)) {
        deployState = Descent;
        print2RF("Descent in action...");
        Serial.println("Descent in action");
      }
      break;

    case Descent:
      if (alti < DEPLOY_ALTI || newData == true && (strcmp(receivedChars, "deploy") == 0)) {

        //Activate relay 1
        digitalWrite(rly1, HIGH);
        print2RF("Relay 1 Fire...");
        delay(1000);
        //Activate relay 2
        digitalWrite(rly2, HIGH);
        print2RF("Relay 2 Fire...");
        print2RF("Payload Deployed...");
        Serial.println("Deployed");

        deployState = Pad;
      }
      break;

    default:
      print2RF("default case");
      deployState = Pad;
      break;
  }
  showCommand();
  if (newData) {
    newData = false;
  }
  
}

void getCommand() {
  static boolean inProgress = false;
  static byte ndx = 0;
  char startMark = '(';
  char endMark = ')';
  char rc;

  while (rfSerial.available() > 0 && newData == false) {
    rc = rfSerial.read();
    rfSerial.write(rc);

    if (inProgress == true) {
      if (rc != endMark) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0';  // terminate the string
        inProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMark) {
      inProgress = true;
    }
  }
}

void showCommand() {
  if (newData == true) {
    //Serial.print("This just in ... ");
    Serial.println(receivedChars);
    // newData = false;
  }
}

// void mplSetup() {
//   if (!mpl.begin()) {
//     print2RF("NoAlti");
//     mplSetup();
//   }
//   mpl.setSeaPressure(1013.25);
// }

void dpsSetup() {
  if (!dps.begin_I2C()) {
    print2RF("NoAlti");
    Serial.print("NoAlti");
    dpsSetup();
  }

  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
}

// void imuSetup() {
//   if (!imuBNO.begin()) {
//     print2RF("NoIMU");
//     imuSetup();
//   }
//   imuBNO.setExtCrystalUse(true);
// }

void print2RF(String message) {
  unsigned long sysClock = millis() - startTime;
  rfSerial.println(String(sysClock) + " " + message);
  if (sdFile) {
    sdFile.println(String(sysClock) + " " + message);
    sdFile.flush();
  }
}
