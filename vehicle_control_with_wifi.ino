#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>

// WiFi AP credentials
const char* ssid = "RoadDryingVehicle"; // ESP32 AP SSID
const char* password = "xai12345"; // ESP32 AP password

// Web server
WebServer server(80);

// L298N: Front Wheels (Module 1)
const int wheelIN1 = 25, wheelIN2 = 26, wheelIN3 = 33, wheelIN4 = 32;
const int wheelENA = 27, wheelENB = 14;
// L298N: Mopper and Pump (Module 2)
const int mopperIN1 = 19, mopperIN2 = 18, mopperENA = 5;
const int pumpIN3 = 21, pumpIN4 = 22, pumpENB = 2;
const int ledPump = 23;
// Servos
const int servoLeftPin = 13, servoRightPin = 12, servoRollerPin = 15, servoSensorPin = 4;
// Sensors
const int waterLevelPin = 34, tempSensorPin = 35;
// PWM
const int pwmResolution = 8; // 0-255

// Servos
Servo servoLeft, servoRight, servoRoller, servoSensor;

// Thresholds and state
const int waterThreshold = 500;
int wheelSpeed = 26; // Initial speed (10% ≈ 26)
int steerAngle = 90; // Center
int servoLeftAngle = 90, servoRightAngle = 90, servoRollerAngle = 180, servoSensorAngle = 180;
int mopperSpeed = 128, pumpSpeed = 128;
bool mopperOverride = false, pumpOverride = false;
bool steerOverride = false, speedOverride = false, sensorOverride = false, rollerOverride = false;
bool wheelsForward = true, mopperForward = true;
bool sensorDown = false; // Sensor position state
bool frontWheelsOn = false; // Front wheels state

// Web server handler
void handleRoot() {
  Serial.println("Handling root request");
  int waterLevel = analogRead(waterLevelPin);
  float tempVoltage = analogRead(tempSensorPin) * 3.3 / 4096.0; // 3.3V ADC
  float temperature = tempVoltage * 100.0; // LM35: 10mV/°C
  // Validate sensor data
  if (waterLevel < 0 || waterLevel > 4095) waterLevel = 0;
  if (temperature < 0 || temperature > 100) temperature = 25.0;
  // Categorize water level
  float waterPercent = (waterLevel / 4095.0) * 100.0;
  String waterCategory;
  if (waterPercent <= 20) waterCategory = "Less Water";
  else if (waterPercent <= 60) waterCategory = "Medium Water";
  else if (waterPercent <= 90) waterCategory = "High Level Water";
  else waterCategory = "Very High Level Water";
  // Categorize temperature
  String tempCategory;
  if (temperature < 20) tempCategory = "Cold";
  else if (temperature <= 40) tempCategory = "Normal";
  else tempCategory = "High";

  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Vehicle Control</title>
  <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/css/bootstrap.min.css" rel="stylesheet">
  <style>
    body {
      background-color: #fff8e1; /* Cream background */
      font-family: Arial, sans-serif;
    }
    .container { padding: 20px; }
    .card { border: 1px solid #ccc; border-radius: 5px; margin-bottom: 20px; }
    .card-header-green {
      background-color: #28a745; /* Green strap */
      color: white;
      padding: 10px;
      border-radius: 5px 5px 0 0;
    }
    .card-header-blue {
      background-color: #1a73e8; /* Blue strap */
      color: white;
      padding: 10px;
      border-radius: 5px 5px 0 0;
    }
    .btn-green { background-color: #28a745; color: white; }
    .btn-blue { background-color: #1a73e8; color: white; }
    .btn-red { background-color: #dc3545; color: white; }
    .status-on { color: #28a745; font-weight: bold; }
    .status-off { color: #dc3545; font-weight: bold; }
    .slider { width: 100%; }
    .sensor-category { font-weight: bold; }
  </style>
</head>
<body>
  <div class="container">
    <h1 class="text-center mb-4">Vehicle Control Dashboard</h1>
    
    <!-- Dashboard -->
    <div class="card">
      <div class="card-header-green">Status</div>
      <div class="card-body">
        <div class="row">
          <div class="col-md-4 mb-2">Water Level: <span id="waterLevel">)rawliteral";
  html += String(waterLevel) + "</span> (0-4095, <span class=\"sensor-category\">" + waterCategory + "</span>)";
  html += R"rawliteral(</div>
          <div class="col-md-4 mb-2">Temperature: <span id="temp">)rawliteral";
  html += String(temperature, 1) + "</span> °C (<span class=\"sensor-category\">" + tempCategory + "</span>)";
  html += R"rawliteral(</div>
          <div class="col-md-4 mb-2">Front Wheels: <span id="frontWheels" class=")rawliteral";
  html += frontWheelsOn ? "status-on\">ON" : "status-off\">OFF";
  html += R"rawliteral(</span></div>
          <div class="col-md-4 mb-2">Wheel Direction: <span id="wheelDir">)rawliteral";
  html += wheelsForward ? "Forward" : "Reverse";
  html += R"rawliteral(</span></div>
          <div class="col-md-4 mb-2">Mopper: <span id="mopperStatus" class=")rawliteral";
  html += mopperOverride ? "status-on\">ON" : "status-off\">OFF";
  html += R"rawliteral(</span></div>
          <div class="col-md-4 mb-2">Pump: <span id="pumpStatus" class=")rawliteral";
  html += pumpOverride ? "status-on\">ON" : "status-off\">OFF";
  html += R"rawliteral(</span></div>
        </div>
      </div>
    </div>
    
    <!-- Controls -->
    <div class="row">
      <!-- Wheel Controls -->
      <div class="col-md-4">
        <div class="card">
          <div class="card-header-blue">Wheel Controls</div>
          <div class="card-body">
            <div class="mb-3">
              <label>Front Wheels:</label>
              <a href="/front_wheels_on" class="btn btn-green btn-sm ms-2">ON</a>
              <a href="/front_wheels_off" class="btn btn-red btn-sm">OFF</a>
            </div>
            <div class="mb-3">
              <label>Direction:</label>
              <a href="/wheels_forward" class="btn btn-green btn-sm ms-2">Forward</a>
              <a href="/wheels_reverse" class="btn btn-red btn-sm">Reverse</a>
            </div>
            <div class="mb-3">
              <label>Speed: <span id="wheelSpeedValue">)rawliteral";
  html += String(wheelSpeed) + "</span> (0-255)</label>";
  html += R"rawliteral(
              <input type="range" class="form-range slider" id="wheelSpeedSlider" min="0" max="255" value=")rawliteral";
  html += String(wheelSpeed) + R"rawliteral(" onchange="updateWheelSpeed(this.value)">
            </div>
            <div class="mb-3">
              <label>Steering:</label>
              <a href="/steer_center" class="btn btn-green btn-sm ms-2">Center</a>
            </div>
            <div class="mb-3">
              <label>Steering Angle: <span id="steerValue">)rawliteral";
  html += String(steerAngle) + "</span>°</label>";
  html += R"rawliteral(
              <input type="range" class="form-range slider" id="steerSlider" min="0" max="180" value=")rawliteral";
  html += String(steerAngle) + R"rawliteral(" onchange="updateSteering(this.value)">
            </div>
          </div>
        </div>
      </div>
      
      <!-- Servo Controls -->
      <div class="col-md-4">
        <div class="card">
          <div class="card-header-green">Servo Controls</div>
          <div class="card-body">
            <div class="mb-3">
              <label>Left Steering: <span id="servoLeftValue">)rawliteral";
  html += String(servoLeftAngle) + "</span>°</label>";
  html += R"rawliteral(
              <input type="range" class="form-range slider" id="servoLeftSlider" min="0" max="180" value=")rawliteral";
  html += String(servoLeftAngle) + R"rawliteral(" onchange="updateServoLeft(this.value)">
            </div>
            <div class="mb-3">
              <label>Right Steering: <span id="servoRightValue">)rawliteral";
  html += String(servoRightAngle) + "</span>°</label>";
  html += R"rawliteral(
              <input type="range" class="form-range slider" id="servoRightSlider" min="0" max="180" value=")rawliteral";
  html += String(servoRightAngle) + R"rawliteral(" onchange="updateServoRight(this.value)">
            </div>
            <div class="mb-3">
              <label>Roller:</label>
              <a href="/roller_toggle" class="btn btn-green btn-sm ms-2">Toggle</a>
              <span id="rollerStatus" class=")rawliteral";
  html += rollerOverride ? "status-on\">Deployed" : "status-off\">Retracted";
  html += R"rawliteral(</span>
            </div>
            <div class="mb-3">
              <label>Roller Angle: <span id="servoRollerValue">)rawliteral";
  html += String(servoRollerAngle) + "</span>°</label>";
  html += R"rawliteral(
              <input type="range" class="form-range slider" id="servoRollerSlider" min="0" max="180" value=")rawliteral";
  html += String(servoRollerAngle) + R"rawliteral(" onchange="updateServoRoller(this.value)">
            </div>
            <div class="mb-3">
              <label>Water Sensor:</label>
              <a href="/sensor_toggle" class="btn btn-green btn-sm ms-2">Toggle</a>
              <span id="sensorStatus" class=")rawliteral";
  html += sensorOverride ? (servoSensorAngle == 0 ? "status-on\">Lowered" : "status-off\">Raised") : "status-off\">Auto";
  html += R"rawliteral(</span>
            </div>
            <div class="mb-3">
              <label>Sensor Angle: <span id="servoSensorValue">)rawliteral";
  html += String(servoSensorAngle) + "</span>°</label>";
  html += R"rawliteral(
              <input type="range" class="form-range slider" id="servoSensorSlider" min="0" max="180" value=")rawliteral";
  html += String(servoSensorAngle) + R"rawliteral(" onchange="updateServoSensor(this.value)">
            </div>
          </div>
        </div>
      </div>
      
      <!-- Mopper and Pump Controls -->
      <div class="col-md-4">
        <div class="card">
          <div class="card-header-blue">Mopper & Pump</div>
          <div class="card-body">
            <div class="mb-3">
              <label>Mopper: <span id="mopperToggleStatus" class=")rawliteral";
  html += mopperOverride ? "status-on\">ON" : "status-off\">OFF";
  html += R"rawliteral(</span></label>
              <a href="/mopper_on" class="btn btn-green btn-sm ms-2">ON</a>
              <a href="/mopper_off" class="btn btn-red btn-sm">OFF</a>
            </div>
            <div class="mb-3">
              <label>Mopper Direction:</label>
              <a href="/mopper_forward" class="btn btn-green btn-sm ms-2">Forward</a>
              <a href="/mopper_reverse" class="btn btn-red btn-sm">Reverse</a>
            </div>
            <div class="mb-3">
              <label>Mopper Speed: <span id="mopperSpeedValue">)rawliteral";
  html += String(mopperSpeed) + "</span> (0-255)</label>";
  html += R"rawliteral(
              <input type="range" class="form-range slider" id="mopperSpeedSlider" min="0" max="255" value=")rawliteral";
  html += String(mopperSpeed) + R"rawliteral(" onchange="updateMopperSpeed(this.value)">
            </div>
            <div class="mb-3">
              <label>Pump: <span id="pumpToggleStatus" class=")rawliteral";
  html += pumpOverride ? "status-on\">ON" : "status-off\">OFF";
  html += R"rawliteral(</span></label>
              <a href="/pump_on" class="btn btn-green btn-sm ms-2">ON</a>
              <a href="/pump_off" class="btn btn-red btn-sm">OFF</a>
            </div>
            <div class="mb-3">
              <label>Pump Speed: <span id="pumpSpeedValue">)rawliteral";
  html += String(pumpSpeed) + "</span> (0-255)</label>";
  html += R"rawliteral(
              <input type="range" class="form-range slider" id="pumpSpeedSlider" min="0" max="255" value=")rawliteral";
  html += String(pumpSpeed) + R"rawliteral(" onchange="updatePumpSpeed(this.value)">
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
  
  <script>
    const serverUrl = 'http://192.168.4.1'; // ESP32 AP IP
    function updateWheelSpeed(value) {
      fetch(serverUrl + '/wheel_speed?value=' + value);
      document.getElementById('wheelSpeedValue').innerText = value;
    }
    function updateSteering(value) {
      fetch(serverUrl + '/steer?angle=' + value);
      document.getElementById('steerValue').innerText = value;
    }
    function updateServoLeft(value) {
      fetch(serverUrl + '/servo_left?angle=' + value);
      document.getElementById('servoLeftValue').innerText = value;
    }
    function updateServoRight(value) {
      fetch(serverUrl + '/servo_right?angle=' + value);
      document.getElementById('servoRightValue').innerText = value;
    }
    function updateServoRoller(value) {
      fetch(serverUrl + '/servo_roller?angle=' + value);
      document.getElementById('servoRollerValue').innerText = value;
    }
    function updateServoSensor(value) {
      fetch(serverUrl + '/servo_sensor?angle=' + value);
      document.getElementById('servoSensorValue').innerText = value;
    }
    function updateMopperSpeed(value) {
      fetch(serverUrl + '/mopper_speed?value=' + value);
      document.getElementById('mopperSpeedValue').innerText = value;
    }
    function updatePumpSpeed(value) {
      fetch(serverUrl + '/pump_speed?value=' + value);
      document.getElementById('pumpSpeedValue').value = value;
    }
    // Auto-refresh every 5 seconds
    setTimeout(() => location.reload(), 5000);
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

// Server handlers
void handleFrontWheelsOn() {
  frontWheelsOn = true;
  handleRoot();
}
void handleFrontWheelsOff() {
  frontWheelsOn = false;
  handleRoot();
}
void handleWheelsForward() {
  wheelsForward = true;
  speedOverride = true;
  handleRoot();
}
void handleWheelsReverse() {
  wheelsForward = false;
  speedOverride = true;
  handleRoot();
}
void handleWheelSpeed() {
  if (server.hasArg("value")) {
    wheelSpeed = server.arg("value").toInt();
    speedOverride = true;
  }
  handleRoot();
}
void handleServoLeft() {
  if (server.hasArg("angle")) {
    servoLeftAngle = server.arg("angle").toInt();
    steerOverride = true;
  }
  handleRoot();
}
void handleServoRight() {
  if (server.hasArg("angle")) {
    servoRightAngle = server.arg("angle").toInt();
    steerOverride = true;
  }
  handleRoot();
}
void handleServoRoller() {
  if (server.hasArg("angle")) {
    servoRollerAngle = server.arg("angle").toInt();
    rollerOverride = true;
  }
  handleRoot();
}
void handleServoSensor() {
  if (server.hasArg("angle")) {
    servoSensorAngle = server.arg("angle").toInt();
    sensorOverride = true;
  }
  handleRoot();
}
void handleMopperForward() {
  mopperForward = true;
  handleRoot();
}
void handleMopperReverse() {
  mopperForward = false;
  handleRoot();
}
void handleMopperSpeed() {
  if (server.hasArg("value")) {
    mopperSpeed = server.arg("value").toInt();
  }
  handleRoot();
}
void handlePumpSpeed() {
  if (server.hasArg("value")) {
    pumpSpeed = server.arg("value").toInt();
  }
  handleRoot();
}
void handleMopperOn() {
  mopperOverride = true;
  handleRoot();
}
void handleMopperOff() {
  mopperOverride = false;
  handleRoot();
}
void handlePumpOn() {
  pumpOverride = true;
  handleRoot();
}
void handlePumpOff() {
  pumpOverride = false;
  handleRoot();
}
void handleSteer() {
  if (server.hasArg("angle")) {
    steerAngle = server.arg("angle").toInt();
    servoLeftAngle = steerAngle;
    servoRightAngle = steerAngle;
    steerOverride = true;
  }
  handleRoot();
}
void handleSteerCenter() {
  steerAngle = 90;
  servoLeftAngle = steerAngle;
  servoRightAngle = steerAngle;
  steerOverride = true;
  handleRoot();
}
void handleRollerToggle() {
  rollerOverride = true;
  servoRollerAngle = (servoRollerAngle == 180) ? 0 : 180;
  handleRoot();
}
void handleSensorToggle() {
  sensorOverride = true;
  servoSensorAngle = (servoSensorAngle == 180) ? 0 : 180;
  sensorDown = (servoSensorAngle == 0);
  handleRoot();
}

void setup() {
  Serial.begin(115200);
  delay(100); // Allow Serial to stabilize

  // WiFi AP setup
  WiFi.softAP(ssid, password);
  Serial.println("WiFi AP Started");
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP()); // Typically 192.168.4.1

  // Web server routes
  Serial.println("Setting up server routes...");
  server.on("/", handleRoot);
  server.on("/front_wheels_on", handleFrontWheelsOn);
  server.on("/front_wheels_off", handleFrontWheelsOff);
  server.on("/wheels_forward", handleWheelsForward);
  server.on("/wheels_reverse", handleWheelsReverse);
  server.on("/wheel_speed", handleWheelSpeed);
  server.on("/servo_left", handleServoLeft);
  server.on("/servo_right", handleServoRight);
  server.on("/servo_roller", handleServoRoller);
  server.on("/servo_sensor", handleServoSensor);
  server.on("/mopper_forward", handleMopperForward);
  server.on("/mopper_reverse", handleMopperReverse);
  server.on("/mopper_speed", handleMopperSpeed);
  server.on("/pump_speed", handlePumpSpeed);
  server.on("/mopper_on", handleMopperOn);
  server.on("/mopper_off", handleMopperOff);
  server.on("/pump_on", handlePumpOn);
  server.on("/pump_off", handlePumpOff);
  server.on("/steer", handleSteer);
  server.on("/steer_center", handleSteerCenter);
  server.on("/roller_toggle", handleRollerToggle);
  server.on("/sensor_toggle", handleSensorToggle);
  server.begin();
  Serial.println("Server started");

  // Delay for power stabilization
  Serial.println("Delaying for power stabilization...");
  delay(1000);

  // Memory check
  Serial.print("Free heap before servos: "); Serial.println(ESP.getFreeHeap());

  // Attach servos
  Serial.println("Attaching servos...");
  if (!servoLeft.attach(servoLeftPin)) Serial.println("Failed to attach left servo");
  delay(200);
  if (!servoRight.attach(servoRightPin)) Serial.println("Failed to attach right servo");
  delay(200);
  if (!servoRoller.attach(servoRollerPin)) Serial.println("Failed to attach roller servo");
  delay(200);
  if (!servoSensor.attach(servoSensorPin)) Serial.println("Failed to attach sensor servo");
  delay(200);
  Serial.println("All servos attached");

  // Memory check
  Serial.print("Free heap after servos: "); Serial.println(ESP.getFreeHeap());

  // Pins
  Serial.println("Configuring pins...");
  pinMode(wheelIN1, OUTPUT); pinMode(wheelIN2, OUTPUT);
  pinMode(wheelIN3, OUTPUT); pinMode(wheelIN4, OUTPUT);
  pinMode(wheelENA, OUTPUT); pinMode(wheelENB, OUTPUT);
  pinMode(mopperIN1, OUTPUT); pinMode(mopperIN2, OUTPUT);
  pinMode(mopperENA, OUTPUT); pinMode(pumpIN3, OUTPUT);
  pinMode(pumpIN4, OUTPUT); pinMode(pumpENB, OUTPUT);
  pinMode(waterLevelPin, INPUT); pinMode(tempSensorPin, INPUT);
  pinMode(ledPump, OUTPUT);
  Serial.println("Pins configured");

  // Initial state
  Serial.println("Setting initial state...");
  servoLeft.write(90); servoRight.write(90); servoRoller.write(180); servoSensor.write(180);
  digitalWrite(wheelIN1, LOW); digitalWrite(wheelIN2, LOW);
  digitalWrite(wheelIN3, LOW); digitalWrite(wheelIN4, LOW);
  digitalWrite(mopperIN1, LOW); digitalWrite(mopperIN2, LOW);
  digitalWrite(pumpIN3, LOW); digitalWrite(pumpIN4, LOW);
  analogWrite(wheelENA, 0); analogWrite(wheelENB, 0);
  analogWrite(mopperENA, 0); analogWrite(pumpENB, 0);
  digitalWrite(ledPump, LOW);
  Serial.println("Setup complete");
}

void loop() {
  Serial.println("Entering loop...");
  server.handleClient();
  int waterLevel = analogRead(waterLevelPin);
  float tempVoltage = analogRead(tempSensorPin) * 3.3 / 4096.0;
  float temperature = tempVoltage * 100.0;

  // Serial logging
  Serial.print("Time: "); Serial.print(millis() / 1000); Serial.print("s, ");
  Serial.print("Water: "); Serial.print(waterLevel); Serial.print(" ADC, ");
  Serial.print("Temp: "); Serial.print(temperature, 1); Serial.print(" °C, ");
  Serial.print("Wheel Speed: "); Serial.print(wheelSpeed); Serial.print(" PWM, ");
  Serial.print("Front Wheels: "); Serial.print(frontWheelsOn ? "ON" : "OFF"); Serial.print(", ");
  Serial.print("Steer: "); Serial.print(steerAngle); Serial.print(" °, ");
  Serial.print("ServoLeft: "); Serial.print(servoLeftAngle); Serial.print(" °, ");
  Serial.print("ServoRight: "); Serial.print(servoRightAngle); Serial.print(" °, ");
  Serial.print("Roller: "); Serial.print(servoRollerAngle); Serial.print(" °, ");
  Serial.print("Sensor: "); Serial.print(servoSensorAngle); Serial.print(" °, ");
  Serial.print("Mopper: "); Serial.print(mopperOverride ? "ON" : "OFF"); Serial.print(", ");
  Serial.print("Mopper Dir: "); Serial.print(mopperForward ? "FWD" : "REV"); Serial.print(", ");
  Serial.print("Pump: "); Serial.print(pumpOverride ? "ON" : "OFF"); Serial.println();

  // Wheel speed and direction
  if (!speedOverride) {
    wheelSpeed = (waterLevel > waterThreshold) ? 20 : 26; // 20 when wet, 26 when dry
  }
  if (wheelSpeed > 0 && frontWheelsOn) {
    digitalWrite(wheelIN1, wheelsForward ? HIGH : LOW);
    digitalWrite(wheelIN2, wheelsForward ? LOW : HIGH);
    digitalWrite(wheelIN3, wheelsForward ? HIGH : LOW);
    digitalWrite(wheelIN4, wheelsForward ? LOW : HIGH);
    analogWrite(wheelENA, wheelSpeed);
  } else {
    digitalWrite(wheelIN1, LOW); digitalWrite(wheelIN2, LOW);
    digitalWrite(wheelIN3, LOW); digitalWrite(wheelIN4, LOW);
    analogWrite(wheelENA, 0);
  }
  // Rear wheels (always controlled by wheelSpeed)
  if (wheelSpeed > 0) {
    analogWrite(wheelENB, wheelSpeed); // Rear wheels follow wheelSpeed
  } else {
    analogWrite(wheelENB, 0);
  }

  // Steering
  if (!steerOverride) {
    steerAngle = 90;
    servoLeftAngle = steerAngle;
    servoRightAngle = steerAngle;
  }
  servoLeft.write(servoLeftAngle);
  servoRight.write(servoRightAngle);

  // Water level sensor position
  if (!sensorOverride) {
    if (waterLevel > waterThreshold) {
      servoSensorAngle = 0; // Lower
      sensorDown = true;
    } else {
      servoSensorAngle = 180; // Up
      sensorDown = false;
    }
  }
  servoSensor.write(servoSensorAngle);

  // Roller
  if (!rollerOverride && waterLevel > waterThreshold) {
    servoRollerAngle = 0; // Deploy
  } else if (!rollerOverride) {
    servoRollerAngle = 180; // Retract
  }
  servoRoller.write(servoRollerAngle);

  // Mopper
  if (mopperOverride) {
    digitalWrite(mopperIN1, mopperForward ? HIGH : LOW);
    digitalWrite(mopperIN2, mopperForward ? LOW : HIGH);
    analogWrite(mopperENA, mopperSpeed);
  } else {
    digitalWrite(mopperIN1, LOW); digitalWrite(mopperIN2, LOW);
    analogWrite(mopperENA, 0);
  }

  // Pump
  bool pumpActive = (waterLevel > waterThreshold || pumpOverride);
  if (pumpActive) {
    digitalWrite(pumpIN3, HIGH); digitalWrite(pumpIN4, LOW);
    analogWrite(pumpENB, pumpSpeed);
    digitalWrite(ledPump, HIGH);
  } else {
    digitalWrite(pumpIN3, LOW); digitalWrite(pumpIN4, LOW);
    analogWrite(pumpENB, 0);
    digitalWrite(ledPump, LOW);
  }

  delay(100); // Prevent watchdog reset
}