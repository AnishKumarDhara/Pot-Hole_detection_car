#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ctype.h>
#include <BlynkSimpleEsp32.h>

// ============================================================
// BLYNK
// ============================================================

#define BLYNK_TEMPLATE_ID "Enter your templete id"
#define BLYNK_TEMPLATE_NAME "Pothole ESP32"
#define BLYNK_AUTH_TOKEN "Enter your token"

// ============================================================
// OBJECTS
// ============================================================

Adafruit_MPU6050 mpu;
WebServer server(80);
BlynkTimer timer;

// ============================================================
// WIFI
// ============================================================

// CHANGE THESE TWO VALUES TO YOUR PHONE HOTSPOT DETAILS

const char* WIFI_SSID = "Vivo T3";
const char* WIFI_PASSWORD = "HYDROGEN";

// WiFi connection settings
constexpr unsigned long WIFI_RETRY_INTERVAL = 10000;
constexpr unsigned long WIFI_CONNECT_TIMEOUT = 20000;

unsigned long lastWiFiAttempt = 0;
bool wifiConnecting = false;

// ============================================================
// MOTOR PINS
// ============================================================

constexpr uint8_t PIN_ENA = 25;
constexpr uint8_t PIN_IN1 = 13;
constexpr uint8_t PIN_IN2 = 14;

constexpr uint8_t PIN_IN3 = 27;
constexpr uint8_t PIN_IN4 = 26;
constexpr uint8_t PIN_ENB = 33;

// ============================================================
// MPU6050
// ============================================================

constexpr uint8_t PIN_SDA = 21;
constexpr uint8_t PIN_SCL = 22;

// ============================================================
// MOTOR CONFIGURATION
// ============================================================

constexpr bool LEFT_REVERSED = false;
constexpr bool RIGHT_REVERSED = true;

constexpr uint32_t PWM_FREQUENCY = 1000;
constexpr uint8_t PWM_RESOLUTION = 8;

uint8_t speedValue = 210;

// ============================================================
// SAFETY
// ============================================================

constexpr unsigned long COMMAND_TIMEOUT_MS = 3000;

char activeCommand = 'S';
unsigned long lastCommandTime = 0;

// ============================================================
// MPU VARIABLES
// ============================================================

bool mpuReady = false;

constexpr unsigned long IMU_PERIOD_US = 10000;
constexpr unsigned long PRINT_PERIOD_MS = 50;

unsigned long lastImuTime = 0;
unsigned long lastPrintTime = 0;

float rawX = 0.0f;
float rawY = 0.0f;
float rawZ = 0.0f;

float gravityEstimateZ = 0.0f;
float dynamicZ = 0.0f;

bool gravityInitialized = false;

constexpr float GRAVITY_FILTER_ALPHA = 0.02f;


// ============================================================
// MOTOR FUNCTIONS
// ============================================================

void driveOneSide(
  uint8_t input1,
  uint8_t input2,
  uint8_t enablePin,
  int direction,
  bool reversed
) {

  if (reversed) {
    direction = -direction;
  }

  // Stop PWM before changing direction
  ledcWrite(enablePin, 0);

  if (direction > 0) {

    digitalWrite(input1, HIGH);
    digitalWrite(input2, LOW);

  }

  else if (direction < 0) {

    digitalWrite(input1, LOW);
    digitalWrite(input2, HIGH);

  }

  else {

    digitalWrite(input1, LOW);
    digitalWrite(input2, LOW);

  }

  if (direction != 0) {
    ledcWrite(enablePin, speedValue);
  }
}


// ============================================================
// LEFT MOTOR
// ============================================================

void driveLeft(int direction) {

  driveOneSide(
    PIN_IN1,
    PIN_IN2,
    PIN_ENA,
    direction,
    LEFT_REVERSED
  );
}


// ============================================================
// RIGHT MOTOR
// ============================================================

void driveRight(int direction) {

  driveOneSide(
    PIN_IN3,
    PIN_IN4,
    PIN_ENB,
    direction,
    RIGHT_REVERSED
  );
}


// ============================================================
// STOP CAR
// ============================================================

void stopCar() {

  driveLeft(0);
  driveRight(0);
}


// ============================================================
// FORWARD
// ============================================================

void moveForward() {

  driveLeft(1);
  driveRight(1);
}


// ============================================================
// BACKWARD
// ============================================================

void moveBackward() {

  driveLeft(-1);
  driveRight(-1);
}


// ============================================================
// PIVOT LEFT
// ============================================================

void pivotLeft() {

  driveLeft(-1);
  driveRight(1);
}


// ============================================================
// PIVOT RIGHT
// ============================================================

void pivotRight() {

  driveLeft(1);
  driveRight(-1);
}


// ============================================================
// APPLY CURRENT COMMAND
// ============================================================

void applyActiveCommand() {

  switch (activeCommand) {

    case 'F':
      moveForward();
      break;

    case 'B':
      moveBackward();
      break;

    case 'L':
      pivotLeft();
      break;

    case 'R':
      pivotRight();
      break;

    default:
      stopCar();
      activeCommand = 'S';
      break;
  }
}


// ============================================================
// PROCESS COMMAND
// ============================================================

void processCommand(char rawCharacter) {

  if (
    rawCharacter == '\n' ||
    rawCharacter == '\r' ||
    rawCharacter == ' '
  ) {

    if (rawCharacter == ' ') {

      activeCommand = 'S';

      stopCar();

      lastCommandTime = millis();
    }

    return;
  }


  char command = static_cast<char>(
    toupper(
      static_cast<unsigned char>(rawCharacter)
    )
  );


  // ----------------------------------------------------------
  // SPEED 0-9
  // ----------------------------------------------------------

  if (command >= '0' && command <= '9') {

    speedValue = static_cast<uint8_t>(
      map(
        command - '0',
        0,
        9,
        0,
        230
      )
    );

    applyActiveCommand();

    Serial.print("Speed: ");
    Serial.println(speedValue);

    return;
  }


  // ----------------------------------------------------------
  // FULL SPEED
  // ----------------------------------------------------------

  if (command == 'Q') {

    speedValue = 255;

    applyActiveCommand();

    Serial.println("Speed: 255");

    return;
  }


  // ----------------------------------------------------------
  // MOVEMENT COMMANDS
  // ----------------------------------------------------------

  switch (command) {

    case 'F':
    case 'B':
    case 'L':
    case 'R':

      activeCommand = command;

      lastCommandTime = millis();

      applyActiveCommand();

      break;


    case 'S':

      activeCommand = 'S';

      stopCar();

      lastCommandTime = millis();

      break;


    default:

      return;
  }


  Serial.print("ACK: ");
  Serial.println(command);
}


// ============================================================
// MPU6050 SAMPLING
// ============================================================

void sampleMpu6050() {

  if (!mpuReady) {
    return;
  }


  unsigned long currentTime = micros();


  if (currentTime - lastImuTime < IMU_PERIOD_US) {
    return;
  }


  lastImuTime = currentTime;


  sensors_event_t acceleration;
  sensors_event_t gyroscope;
  sensors_event_t temperature;


  mpu.getEvent(
    &acceleration,
    &gyroscope,
    &temperature
  );


  rawX = acceleration.acceleration.x;
  rawY = acceleration.acceleration.y;
  rawZ = acceleration.acceleration.z;


  if (!gravityInitialized) {

    gravityEstimateZ = rawZ;

    gravityInitialized = true;
  }


  gravityEstimateZ +=
    GRAVITY_FILTER_ALPHA *
    (rawZ - gravityEstimateZ);


  dynamicZ = rawZ - gravityEstimateZ;
}


// ============================================================
// SERIAL SENSOR GRAPH
// ============================================================

void printSensorGraph() {

  if (!mpuReady) {
    return;
  }


  unsigned long currentTime = millis();


  if (currentTime - lastPrintTime < PRINT_PERIOD_MS) {
    return;
  }


  lastPrintTime = currentTime;


  Serial.print("X:");
  Serial.print(rawX, 3);

  Serial.print("\tY:");
  Serial.print(rawY, 3);

  Serial.print("\tRawZ:");
  Serial.print(rawZ, 3);

  Serial.print("\tDynamicZ:");
  Serial.print(dynamicZ, 3);

  Serial.print("\tSpeed:");
  Serial.print(speedValue);

  Serial.print("\tCommand:");
  Serial.println(activeCommand);
}


// ============================================================
// WEB PAGE
// ============================================================

const char MAIN_PAGE[] PROGMEM = R"rawliteral(

<!DOCTYPE html>

<html>

<head>

<meta name="viewport"
content="width=device-width,initial-scale=1">

<title>Pothole Car</title>


<style>

body{

font-family:Arial;

text-align:center;

background:#111;

color:white;

margin:0;

padding:20px;

}


h1{

margin-bottom:10px;

}


button{

width:110px;

height:70px;

font-size:25px;

margin:6px;

border-radius:15px;

border:none;

}


.stop{

background:#e53935;

color:white;

}


.info{

font-size:18px;

margin:15px;

}


input{

width:80%;

}

</style>

</head>


<body>


<h1>Pothole Car</h1>


<div class="info">

Command:

<span id="command">S</span>

</div>


<div>

<button

ontouchstart="sendCommand('F')"

onmousedown="sendCommand('F')"

ontouchend="sendCommand('S')"

onmouseup="sendCommand('S')">

↑

</button>

</div>


<div>


<button

ontouchstart="sendCommand('L')"

onmousedown="sendCommand('L')"

ontouchend="sendCommand('S')"

onmouseup="sendCommand('S')">

←

</button>


<button

class="stop"

onclick="sendCommand('S')">

STOP

</button>


<button

ontouchstart="sendCommand('R')"

onmousedown="sendCommand('R')"

ontouchend="sendCommand('S')"

onmouseup="sendCommand('S')">

→

</button>


</div>


<div>


<button

ontouchstart="sendCommand('B')"

onmousedown="sendCommand('B')"

ontouchend="sendCommand('S')"

onmouseup="sendCommand('S')">

↓

</button>


</div>


<div class="info">

Speed:

<br>


<input

type="range"

min="0"

max="255"

value="210"

oninput="setSpeed(this.value)">


<br>


<span id="speed">210</span>


</div>


<div class="info">

X:

<span id="x">0</span>

<br>


Y:

<span id="y">0</span>

<br>


Raw Z:

<span id="z">0</span>

<br>


Dynamic Z:

<span id="dz">0</span>

<br>


MPU:

<span id="mpu">Checking</span>

</div>


<script>


let currentCommand='S';


function sendCommand(c){

currentCommand=c;

fetch('/cmd?c='+c);

document.getElementById('command').innerText=c;

}


function setSpeed(v){

document.getElementById('speed').innerText=v;

fetch('/speed?v='+v);

}


async function updateSensor(){

try{

let response=await fetch('/sensor');

let data=await response.json();


document.getElementById('x').innerText=
data.x.toFixed(2);


document.getElementById('y').innerText=
data.y.toFixed(2);


document.getElementById('z').innerText=
data.z.toFixed(2);


document.getElementById('dz').innerText=
data.dynamicZ.toFixed(2);


document.getElementById('mpu').innerText=
data.mpu ? 'Detected' : 'Not detected';

}

catch(e){}

}


setInterval(updateSensor,200);


document.addEventListener('keydown',function(e){


if(e.key==='w' || e.key==='W'){

sendCommand('F');

}


if(e.key==='s' || e.key==='S'){

sendCommand('B');

}


if(e.key==='a' || e.key==='A'){

sendCommand('L');

}


if(e.key==='d' || e.key==='D'){

sendCommand('R');

}


if(e.code==='Space'){

sendCommand('S');

}

});


document.addEventListener('keyup',function(e){


if(

e.key==='w' ||

e.key==='W' ||

e.key==='s' ||

e.key==='S' ||

e.key==='a' ||

e.key==='A' ||

e.key==='d' ||

e.key==='D'

){

sendCommand('S');

}

});


</script>


</body>

</html>

)rawliteral";


// ============================================================
// BLYNK
// ============================================================

void sendToBlynk() {

  if (!mpuReady) {
    return;
  }

  // Only send if WiFi is actually connected
  if (WiFi.status() == WL_CONNECTED) {

    Blynk.virtualWrite(V3, dynamicZ);

  }
}


// ============================================================
// WEB ROOT
// ============================================================

void handleRoot() {

  server.send(
    200,
    "text/html",
    MAIN_PAGE
  );
}


// ============================================================
// COMMAND HANDLER
// ============================================================

void handleCommand() {

  if (!server.hasArg("c")) {

    server.send(
      400,
      "text/plain",
      "Missing command"
    );

    return;
  }


  String value = server.arg("c");


  if (value.length() > 0) {

    processCommand(
      value.charAt(0)
    );

  }


  server.send(
    200,
    "text/plain",
    "OK"
  );
}


// ============================================================
// SPEED HANDLER
// ============================================================

void handleSpeed() {

  if (!server.hasArg("v")) {

    server.send(
      400,
      "text/plain",
      "Missing speed"
    );

    return;
  }


  int value =
    server.arg("v").toInt();


  value =
    constrain(
      value,
      0,
      255
    );


  speedValue =
    static_cast<uint8_t>(value);


  applyActiveCommand();


  Serial.print("Speed: ");
  Serial.println(speedValue);


  server.send(
    200,
    "text/plain",
    "OK"
  );
}


// ============================================================
// SENSOR HANDLER
// ============================================================

void handleSensor() {

  String json = "{";


  json += "\"x\":";
  json += String(rawX, 3);


  json += ",\"y\":";
  json += String(rawY, 3);


  json += ",\"z\":";
  json += String(rawZ, 3);


  json += ",\"dynamicZ\":";
  json += String(dynamicZ, 3);


  json += ",\"mpu\":";
  json += mpuReady ? "true" : "false";


  json += ",\"speed\":";
  json += String(speedValue);


  json += ",\"command\":\"";
  json += activeCommand;
  json += "\"";


  json += "}";


  server.send(
    200,
    "application/json",
    json
  );
}


// ============================================================
// WIFI CONNECTION FUNCTION
// ============================================================

void connectToWiFi() {

  Serial.println();
  Serial.println("==============================");
  Serial.println("Connecting to phone hotspot");
  Serial.println("==============================");

  Serial.print("SSID: ");
  Serial.println(WIFI_SSID);


  WiFi.mode(WIFI_STA);

  WiFi.setAutoReconnect(true);

  WiFi.persistent(true);


  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );


  wifiConnecting = true;

  lastWiFiAttempt = millis();


  Serial.println(
    "WiFi connection attempt started..."
  );
}


// ============================================================
// WIFI MAINTENANCE
// ============================================================

void maintainWiFi() {

  wl_status_t status = WiFi.status();


  // ----------------------------------------------------------
  // CONNECTED
  // ----------------------------------------------------------

  if (status == WL_CONNECTED) {

    if (wifiConnecting) {

      wifiConnecting = false;

      Serial.println();
      Serial.println("==============================");
      Serial.println("WiFi connected!");
      Serial.print("ESP32 IP address: ");
      Serial.println(WiFi.localIP());
      Serial.println("==============================");

    }

    return;
  }


  // ----------------------------------------------------------
  // NOT CONNECTED
  // ----------------------------------------------------------

  if (
    millis() - lastWiFiAttempt >=
    WIFI_RETRY_INTERVAL
  ) {

    Serial.println();
    Serial.println("WiFi not connected.");
    Serial.println("Retrying hotspot connection...");


    WiFi.disconnect();

    delay(100);


    WiFi.begin(
      WIFI_SSID,
      WIFI_PASSWORD
    );


    lastWiFiAttempt = millis();

    wifiConnecting = true;
  }
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(500);


  Serial.println();
  Serial.println("================================");
  Serial.println("POTHOLE DETECTION RC CAR");
  Serial.println("Starting ESP32...");
  Serial.println("================================");


  // ==========================================================
  // MOTOR PINS
  // ==========================================================

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);

  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);


  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);

  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);


  // ==========================================================
  // PWM
  // ==========================================================

  bool leftPwmReady = ledcAttach(
    PIN_ENA,
    PWM_FREQUENCY,
    PWM_RESOLUTION
  );


  bool rightPwmReady = ledcAttach(
    PIN_ENB,
    PWM_FREQUENCY,
    PWM_RESOLUTION
  );


  if (
    !leftPwmReady ||
    !rightPwmReady
  ) {

    Serial.println(
      "ERROR: PWM setup failed."
    );


    while (true) {

      delay(1000);

    }
  }


  ledcWrite(
    PIN_ENA,
    0
  );


  ledcWrite(
    PIN_ENB,
    0
  );


  stopCar();


  // ==========================================================
  // MPU6050
  // ==========================================================

  Wire.begin(
    PIN_SDA,
    PIN_SCL
  );


  Wire.setClock(400000);


  mpuReady = mpu.begin(
    0x68,
    &Wire
  );


  if (mpuReady) {

    mpu.setAccelerometerRange(
      MPU6050_RANGE_8_G
    );


    mpu.setGyroRange(
      MPU6050_RANGE_500_DEG
    );


    mpu.setFilterBandwidth(
      MPU6050_BAND_44_HZ
    );


    Serial.println(
      "MPU6050 detected."
    );

  }

  else {

    Serial.println(
      "WARNING: MPU6050 not detected."
    );

  }


  // ==========================================================
  // WIFI
  // ==========================================================

  connectToWiFi();


  // ==========================================================
  // BLYNK
  // ==========================================================

  // We don't block the entire ESP32 waiting for Blynk.
  // Blynk will be attempted when WiFi is available.

  if (WiFi.status() == WL_CONNECTED) {

    Blynk.config(
      BLYNK_AUTH_TOKEN
    );

    Serial.println(
      "Connecting to Blynk..."
    );


    if (Blynk.connect(10000)) {

      Serial.println(
        "Blynk connected!"
      );

    }

    else {

      Serial.println(
        "Blynk connection failed."
      );

    }

  }

  else {

    Serial.println(
      "WiFi not connected yet."
    );

    Serial.println(
      "Blynk will be available after WiFi connects."
    );

  }


  // ==========================================================
  // BLYNK TIMER
  // ==========================================================

  timer.setInterval(
    100L,
    sendToBlynk
  );


  // ==========================================================
  // WEB SERVER
  // ==========================================================

  server.on(
    "/",
    handleRoot
  );


  server.on(
    "/sensor",
    handleSensor
  );


  server.on(
    "/cmd",
    handleCommand
  );


  server.on(
    "/speed",
    handleSpeed
  );


  server.begin();


  Serial.println(
    "Web server started."
  );


  Serial.println();
  Serial.println(
    "Setup complete."
  );

}


// ============================================================
// LOOP
// ============================================================

void loop() {

  // ----------------------------------------------------------
  // CHECK / RECONNECT WIFI
  // ----------------------------------------------------------

  maintainWiFi();


  // ----------------------------------------------------------
  // WEB SERVER
  // ----------------------------------------------------------

  server.handleClient();


  // ----------------------------------------------------------
  // BLYNK
  // ----------------------------------------------------------

  if (WiFi.status() == WL_CONNECTED) {

    Blynk.run();

  }


  timer.run();


  // ----------------------------------------------------------
  // MPU6050
  // ----------------------------------------------------------

  sampleMpu6050();


  printSensorGraph();


  // ----------------------------------------------------------
  // SAFETY TIMEOUT
  // ----------------------------------------------------------

  if (

    activeCommand != 'S' &&

    millis() - lastCommandTime >
    COMMAND_TIMEOUT_MS

  ) {

    activeCommand = 'S';

    stopCar();


    Serial.println(
      "Safety timeout: motors stopped."
    );

  }


  delay(1);
}
