//UART Communication
//#include <SoftwareSerial.h>
#include <string.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
//Barometers
#include <Adafruit_DPS310.h>

#define TEST true
#define DEPLOY_ALTI 243.84 // 800 ft in m
#define TOL_GROUND 20
#define AVG_COUNT 50
#define APOGEE 1000

// Define the digital pins for the deployment relays
#define rly1 42
#define rly2 40

// Define the digital pin for the buzzer
#define buzz 4

// Define SD card pin
#define sdPin 44


const byte numChars = 10;
char receivedChars[numChars];

enum { Pad,
       Ascent,
       Descent
     };

unsigned long startTime;
unsigned char deployState;
File sdFile;
unsigned int sysClock;

boolean newData = false;

float ground_pressure = 0;
float alti;
Adafruit_DPS310 dps;

void setup()
{
  // put your setup code here, to run once:
  startTime = millis();

  // pinMode(buzz, OUTPUT);
  // for (int i = 0; i < 2; i++) {
  //   digitalWrite(buzz, HIGH);
  //   delay(500);
  //   digitalWrite(buzz, LOW);
  //   delay(500);
  // }

  //Start the serial communication with the Arduino IDE Serial Monitor and RF Modem
  Serial.begin(57600);
  Serial.print("Complete Serial Setup");

  //Set up SD card first for logging data
  pinMode(sdPin, OUTPUT);

  if (!SD.begin(sdPin))
  {
    print2log("SD Init Failed!");
    print2RF("SD Init Failed!");
    return;
  }

  sdFile = SD.open("Log.txt", FILE_WRITE);

  if (!sdFile)
    Serial.print("Error: Failed to open file on first try!");

  print2log("Init SD card done");

  while (!dps.begin_I2C()) {
    Serial.println("NoAlti");
  }
  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  Serial.println("Complete DPS Initialization");

  //DPS Ground Zero
  Serial.println("Begin DPS Ground Altitude Setup");
  sensors_event_t temp_event, pressure_event;
  for (int i = 1; i <= AVG_COUNT; i++) {
    dps.getEvents(&temp_event, &pressure_event);
    //Ground Pressure Approach
    ground_pressure = ((ground_pressure * (i - 1)) / i) + (pressure_event.pressure / i);
  }
  Serial.println("Complete DPS Ground Altitude Setup");

  //Make the relay's digital pins outputs
  pinMode(rly1, OUTPUT);
  pinMode(rly2, OUTPUT);

  Serial.println("Arduino is ready");
  print2log("Arduino is ready");
  deployState = Pad;
}

void loop()
{
  // put your main code here, to run repeatedly:
  alti = dps.readAltitude(ground_pressure);
  print2RF(String(alti));
  print2log(String(alti));

  getCommand();

  print2RF(String(deployState));
  print2log(String(deployState));

  switch (deployState)
  {
    case Pad:
      if (alti > TOL_GROUND || TEST && newData && strcmp(receivedChars, "a") == 0)
      {
        deployState = Ascent;
      }
      break;

    case Ascent:
      if (alti > APOGEE || TEST && newData && strcmp(receivedChars, "d") == 0)
      {
        deployState = Descent;
      }
      break;

    case Descent:
      if (alti < DEPLOY_ALTI)
      {
        if (strcmp(receivedChars, "f") == 0) {
          print2RF("Approval Received");
          print2log("Approval Received");
        }
        //Activate relay 1
        digitalWrite(rly1, HIGH);
        print2RF("Relay 1 Fire...");
        print2log("Relay 1 Fire...");
        //Activate relay 2
        digitalWrite(rly2, HIGH);
        print2RF("Relay 2 Fire...");
        print2log("Relay 2 Fire...");
        print2RF("Payload Deployed...");
        print2log("Payload Deployed...");
        deployState = Pad;
      }
  }

  if (newData)
  {
    newData = false;
  }
  delay(50);
}

void getCommand()
{
  static boolean inProgress = false;
  static byte ndx = 0;
  char startMark = '(';
  char endMark = ')';
  char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();
    print2RF(String(rc));
    print2log(String(rc));
    Serial.println(rc);

    if (inProgress == true)
    {
      if (rc != endMark)
      { // Check for newline character as the end of command
        receivedChars[ndx] = rc; // Null-terminate the string
        ndx++; // Reset index for the next command

        if (ndx >= 10)
        {
          ndx = 10 - 1;
        }
      }
      else
      {
        receivedChars[ndx] = '\0';
        inProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMark)
    {
      inProgress = true;
    }
  }
}

void print2RF(String message)
{
  sysClock = millis() - startTime;
  Serial.println(String(sysClock) + " " + message);
}

//Write statements to SD log file
void print2log(String message)
{
  unsigned long sysClock = millis() - startTime;

  if (sdFile)
  {
    sdFile.println("[" + String(sysClock) + " ms] " + message);
    sdFile.flush();
  }
  else
  {
    Serial.println("Error: failed to open file!");
  }
}
