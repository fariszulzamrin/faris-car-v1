#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID ""
#define BLYNK_TEMPLATE_NAME ""
#define BLYNK_AUTH_TOKEN ""


#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


BlynkTimer timer;   // <-- REQUIRED (since you use timer.setInterval)
WidgetTerminal terminal(V9);

char ssid[] = "YOUR_WIFI";
char pass[] = "YOUR_PASSWORD";
char auth[] = "YOUR_BLYNK_TOKEN";

// ===== L298N Pins =====
const int IN1 = 13;
const int IN2 = 14;
const int ENA = 27;

const int IN3 = 5;
const int IN4 = 18;
const int ENB = 19;

// ===== Obstacle sensor (TCRT5000) =====
const int OBST_PIN = 34;           // input only pin = perfect
const bool OBST_ACTIVE_LOW = true; // most modules are LOW when detected
bool obstacleLock = false;

// ===== OLED (0.96" SSD1306 I2C) =====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

String oledMsg = "Ready";   // last message to show


// ===== light pins =====
const int FRONT_LIGHT_PIN = 26;   // choose a free GPIO
const int REAR_RED_PIN    = 25;   // V6 + auto on reverse
const int SIG_RIGHT_PIN    = 33; // V7 + auto on V2 (2s)
const int SIG_LEFT_PIN     = 32; // V8 + auto on V3 (2s)

bool frontLightOn = false;
bool rearRedManualOn = false;     // V6 state
bool isReversing = false;         // set true only while moving backward

// ===== Turn signal timers =====
const unsigned long SIGNAL_ON_MS = 2000; // auto off after 2 seconds
unsigned long rightSignalUntil = 0;
unsigned long leftSignalUntil  = 0;
bool manualRightOn = false;
bool manualLeftOn  = false;

// ===== PWM config =====
const int PWM_FREQ = 15000;   // 20kHz (usually silent + good torque)
const int PWM_RES  = 8;       // 0-255
const int CH_A = 0;
const int CH_B = 1;

// ===== HONK (V11) -> PAM8403 =====
const int HORN_PIN  = 4;     // ESP32 -> PAM8403 L input
const int HORN_FREQ = 500;  // try 800..1500
const int HORN_RES  = 8;     // 0-255
const int HORN_DUTY = 30;   // loudness (PWM amplitude)

// Speed (0-255)
int speedPWM = 180;           // default; you can change in app

// Optional: minimum to avoid stall (after starting)
const int MIN_PWM_RUN = 220;    // set to 0 if you want true low speeds

// ===== Helpers =====
void setSpeed(int spd) {
  spd = constrain(spd, 0, 255);
  ledcWrite(ENA, spd);
  ledcWrite(ENB, spd);

}

void setSpeedClamped(int spd) {
  spd = constrain(spd, 0, 255);
  if (spd > 0 && spd < MIN_PWM_RUN) spd = MIN_PWM_RUN;
  setSpeed(spd);
}

// Kick-start to overcome static friction / L298N voltage drop
void kickStart(int target) {
  target = constrain(target, 0, 255);
  if (target == 0) {
    setSpeed(0);
    return;
  }
  setSpeed(255);
  delay(150);          // tweak 80–250 ms if you want
  setSpeedClamped(target);
}

void updateRearRed() {
  // Rear red is ON if reversing OR manual switch is ON
  bool rearOn = isReversing || rearRedManualOn;
  digitalWrite(REAR_RED_PIN, rearOn ? HIGH : LOW);
}

void triggerRightSignal() {
  rightSignalUntil = millis() + SIGNAL_ON_MS;
}

void triggerLeftSignal() {
  leftSignalUntil = millis() + SIGNAL_ON_MS;
}

void taskUpdateSignals() {
  unsigned long now = millis();

  bool autoRight = (now < rightSignalUntil);
  bool autoLeft  = (now < leftSignalUntil);

  // Manual overrides / combines with auto
  digitalWrite(SIG_RIGHT_PIN, (manualRightOn || autoRight) ? HIGH : LOW);
  digitalWrite(SIG_LEFT_PIN,  (manualLeftOn  || autoLeft)  ? HIGH : LOW);
}

bool obstacleDetected() {
  int v = digitalRead(OBST_PIN);
  return OBST_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

void oledShowMessage(const String& msg) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("Faris Car");

  display.setCursor(0, 16);
  display.println(msg);   // Adafruit lib will wrap long text to next line

  display.display();
}

// ===== HONK =====
void hornOn()  { ledcWrite(HORN_PIN, HORN_DUTY); }
void hornOff() { ledcWrite(HORN_PIN, 0); }


void stopMotors() {
  isReversing = false;

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  setSpeed(0);

  updateRearRed(); // keep manual state if enabled
}

void taskCheckObstacle() {
  if (obstacleDetected()) {
    if (!obstacleLock) {
      obstacleLock = true;
      stopMotors();

      terminal.println("🚫 Obstacle detected! Car STOPPED + LOCKED");
      terminal.flush();

      // optional: also print to Serial
      Serial.println("Obstacle detected -> LOCK");
    }
  } else {
    if (obstacleLock) {
      obstacleLock = false;

      terminal.println("✅ Obstacle cleared. Control UNLOCKED");
      terminal.flush();

      Serial.println("Obstacle cleared -> UNLOCK");
    }
  }
}


// ===== Motion =====
void forward() {
  //if (obstacleLock) return;

  isReversing = false;
  updateRearRed();

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  kickStart(speedPWM);
}

void backward() {
  //if (obstacleLock) return;

  isReversing = true;
  updateRearRed(); // force ON

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  kickStart(speedPWM);
}

// Your current turning style (pivot). If you want “soft turn”, tell me.
void right() {
  //if (obstacleLock) return;

  isReversing = false;
  updateRearRed();
  triggerRightSignal(); // auto ON then OFF after 2s

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  kickStart(speedPWM);
}

void left() {
  //if (obstacleLock) return;

  isReversing = false;
  updateRearRed();
  triggerLeftSignal(); // auto ON then OFF after 2s
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  kickStart(speedPWM);
}

// ===== Blynk handlers =====
BLYNK_WRITE(V0) { if (param.asInt()) forward();  else stopMotors(); }
BLYNK_WRITE(V1) { if (param.asInt()) backward(); else stopMotors(); }
BLYNK_WRITE(V2) { if (param.asInt()) right();    else stopMotors(); }
BLYNK_WRITE(V3) { if (param.asInt()) left();     else stopMotors(); }

// Speed slider V4: 0..255
BLYNK_WRITE(V4) {
  speedPWM = constrain(param.asInt(), 0, 255);
  Serial.print("SpeedPWM = ");
  Serial.println(speedPWM);

  // Optional: if you want speed to update while moving without re-pressing:
  // setSpeedClamped(speedPWM);
}

// Front light V5
BLYNK_WRITE(V5) {
  frontLightOn = param.asInt();
  digitalWrite(FRONT_LIGHT_PIN, frontLightOn ? HIGH : LOW);
}

// Rear red manual V6 (rear stays ON if reversing)
BLYNK_WRITE(V6) {
  rearRedManualOn = param.asInt();
  updateRearRed();
}

// Right signal manual V7 (momentary or switch both ok)
// If you use a PUSH button: it will retrigger each press
BLYNK_WRITE(V7) {  // Right signal manual (switch)
  manualRightOn = param.asInt();
}

BLYNK_WRITE(V8) {  // Left signal manual (switch)
  manualLeftOn = param.asInt();
}

BLYNK_WRITE(V10) {
  oledMsg = param.asString();
  Serial.print("OLED msg: ");
  Serial.println(oledMsg);
  oledShowMessage(oledMsg);
}

// HONK button V11 (push)
BLYNK_WRITE(V11) {
  if (param.asInt()) hornOn();
  else hornOff();
}


void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA, SCL

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // 0x3C is most common
   Serial.println("OLED not found (try 0x3D or check wiring)");
    } else {
      display.clearDisplay();
      display.display();
      oledShowMessage("Booting...");
  }


  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

   // Lights
  pinMode(FRONT_LIGHT_PIN, OUTPUT);
  digitalWrite(FRONT_LIGHT_PIN, LOW);

  pinMode(REAR_RED_PIN, OUTPUT);
  digitalWrite(REAR_RED_PIN, LOW);

  pinMode(SIG_RIGHT_PIN, OUTPUT);
  digitalWrite(SIG_RIGHT_PIN, LOW);

  pinMode(SIG_LEFT_PIN, OUTPUT);
  digitalWrite(SIG_LEFT_PIN, LOW);

  // Obstacle
  pinMode(OBST_PIN, INPUT);

  
  // Attach PWM channels to pins (ESP32 core 3.x)
  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  ledcAttach(ENB, PWM_FREQ, PWM_RES);

  // Horn PWM (ESP32 core 3.x)
  ledcAttach(HORN_PIN, HORN_FREQ, HORN_RES);
  hornOff();


  stopMotors();

  Blynk.begin(auth, ssid, pass);

  // Keep signals auto-off working without blocking
  timer.setInterval(50L, taskUpdateSignals);
  timer.setInterval(50L, taskCheckObstacle); // fast reaction
}

void loop() {
  Blynk.run();
  timer.run();
}

