#include <Wire.h>
#include <Mouse.h>
#include <Keyboard.h>

// ══════════════════════════════════════════════════════════
//  Pico Controller — FPS + Asphalt (Potentiometer Steering)
// ══════════════════════════════════════════════════════════
//
// MPU-6500:      SDA → GP4, SCL → GP5, VCC → 3V3, GND → GND
// Joystick:      VRx → GP26, VRy → GP27, SW → GP16, VCC → 3V3, GND → GND
// Buttons:       Trigger → GP6, Zoom → GP9, Reload → GP10  (other leg → GND)
// LEDs:          Blue1 → GP7, Blue2 → GP8  (cathode → GND via 220Ω)
// Potentiometer: Wiper → GP28 (ADC2), one end → 3V3, other end → GND
//
// ══════════════════════════════════════════════════════════

// ── Pin assignments ─────────────────────────────────────
const uint8_t JOY_Y_PIN    = 26;
const uint8_t JOY_X_PIN    = 27;
const uint8_t JOY_SW_PIN   = 16;
const uint8_t TRIGGER_PIN  = 6;
const uint8_t ZOOM_PIN     = 9;
const uint8_t RELOAD_PIN   = 10;
const uint8_t LED1         = 7;
const uint8_t LED2         = 8;
const uint8_t POT_PIN      = 28;   // Potentiometer wiper (ADC2)

// ── MPU-6500 registers ──────────────────────────────────
#define MPU_ADDR    0x68
#define PWR_MGMT_1  0x6B
#define CONFIG_REG  0x1A
#define GYRO_CONFIG 0x1B
#define SMPLRT_DIV  0x19
#define GYRO_XOUT_H 0x43

// ── Screen center (1280×720) ────────────────────────────
const int SCREEN_WIDTH  = 1280;
const int SCREEN_HEIGHT = 720;
const int CENTER_X = SCREEN_WIDTH  / 2;
const int CENTER_Y = SCREEN_HEIGHT / 2;

// ── Gyro mouse tuning (FPS Mode) ────────────────────────
const float GYRO_LSB_PER_DPS   = 131.0f;
const float AIM_SENSITIVITY_X  = 0.60f;
const float AIM_SENSITIVITY_Y  = 0.60f;

// Soft Deadzone: set higher to filter resting noise completely
const float GYRO_DEADZONE      = 250.0f;
const float MOUSE_MIN_MOVE     = 0.5f;
const int   MAX_MOUSE_STEP     = 63;

// ── Potentiometer Steering tuning (FIXED — no auto-calibration) ──
// Pot range: 0 to 4095. Center = 2048. Deadzone = ±150.
// Straight: 1898 to 2198. Left: < 1898. Right: > 2198.
const int POT_STEER_CENTER   = 2048;
const int POT_STEER_DEADZONE = 120;

// ── LED animation ───────────────────────────────────────
const int   CHASE_BLINKS      = 6;
const unsigned long CHASE_INTERVAL = 150;
const int   FADE_STEP         = 3;
const unsigned long FADE_INTERVAL  = 15;

// ── State variables ─────────────────────────────────────
float gyroOffsetX=0, gyroOffsetY=0, gyroOffsetZ=0;
float accumX=0, accumY=0;

int centerJoyX=2048, centerJoyY=2048;
const int JOY_THRESHOLD=400;

bool oldForward=false, oldBackward=false, oldLeft=false, oldRight=false;
bool oldSprint=false, oldReload=false;

unsigned long reloadPressTime=0;
bool reloadKeyHeld=false;

unsigned long comboStartTime=0;
bool comboActive=false;

// LED animation state
int  ledState    = 0;
int  chaseCount  = 0;
bool chaseToggle = false;
int  brightness  = 0;
unsigned long lastLedTime = 0;

// Potentiometer steering state
bool steerLeftHeld = false;
bool steerRightHeld = false;

// Progressive Throttle variables (Asphalt Mode)
unsigned long zoomPressStart = 0;
bool zoomWasPressed = false;
unsigned long throttleCycleStart = 0;
const unsigned long THROTTLE_CYCLE_PERIOD = 60; // 60ms cycle period

// Mode state: Default is FPS Mode
bool currentModeAsphalt = false; 

// Online Calibration tracking (prevents slow drift at rest)
unsigned long staticStartTime = 0;
bool isStatic = false;
long staticSumX = 0, staticSumY = 0, staticSumZ = 0;
int staticSampleCount = 0;
int16_t lastRx = 0, lastRy = 0, lastRz = 0;

// ═════════════════════════════════════════════════════════
//  Low-level MPU-6500 helper
// ═════════════════════════════════════════════════════════

void writeRegister(uint8_t reg, uint8_t value){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(value);
  Wire.endTransmission();
}

void readGyro(int16_t &gx, int16_t &gy, int16_t &gz){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)6);
  if(Wire.available()<6){ gx=gy=gz=0; return; }
  gx=((int16_t)Wire.read()<<8)|Wire.read();
  gy=((int16_t)Wire.read()<<8)|Wire.read();
  gz=((int16_t)Wire.read()<<8)|Wire.read();
}

// ═════════════════════════════════════════════════════════
//  Cursor centering — FAST version (~25 ms total)
// ═════════════════════════════════════════════════════════

void centerCursor(){
  // Step 1: Slam to top-left (0,0)
  for(int i = 0; i < 15; i++){
    Mouse.move(-127, -127, 0);
    delay(1);
  }

  // Step 2: Move exactly to center using max-size steps
  int remainX = CENTER_X;
  int remainY = CENTER_Y;

  while(remainX > 0 || remainY > 0){
    int dx = remainX > 127 ? 127 : remainX;
    int dy = remainY > 127 ? 127 : remainY;
    Mouse.move(dx, dy, 0);
    remainX -= dx;
    remainY -= dy;
    delay(1);
  }

  accumX = 0;
  accumY = 0;
  Serial.println("Cursor centered.");
}

// ═════════════════════════════════════════════════════════
//  Calibration (Gyro + Joystick only — NO potentiometer)
// ═════════════════════════════════════════════════════════

void calibrateGyro(){
  const int samples = 2000;
  long sx=0, sy=0, sz=0;
  Serial.println("\nKEEP CONTROLLER STILL. Gyro calibration in 3 seconds...");
  delay(3000);
  for(int i=0; i<samples; i++){
    int16_t x,y,z; readGyro(x,y,z);
    sx+=x; sy+=y; sz+=z;
    delay(2);
  }
  gyroOffsetX=(float)sx/samples;
  gyroOffsetY=(float)sy/samples;
  gyroOffsetZ=(float)sz/samples;
  Serial.print("Gyro offsets: ");
  Serial.print(gyroOffsetX,2); Serial.print(", ");
  Serial.print(gyroOffsetY,2); Serial.print(", ");
  Serial.println(gyroOffsetZ,2);
}

void calibrateJoystick(){
  const int samples = 500;
  long sx=0, sy=0;
  Serial.println("Keep joystick centered...");
  delay(1000);
  for(int i=0; i<samples; i++){
    sx+=analogRead(JOY_X_PIN);
    sy+=analogRead(JOY_Y_PIN);
    delay(2);
  }
  centerJoyX=sx/samples; centerJoyY=sy/samples;
  Serial.print("Joystick center: X=");
  Serial.print(centerJoyX); Serial.print(" Y=");
  Serial.println(centerJoyY);
}

// ═════════════════════════════════════════════════════════
//  Online Auto-Zero Drift Calibration
// ═════════════════════════════════════════════════════════
void updateOnlineCalibration(int16_t rx, int16_t ry, int16_t rz) {
  if (abs(rx - lastRx) < 3 && abs(ry - lastRy) < 3 && abs(rz - lastRz) < 3) {
    if (!isStatic) {
      isStatic = true;
      staticStartTime = millis();
      staticSumX = 0; staticSumY = 0; staticSumZ = 0;
      staticSampleCount = 0;
    }
    
    staticSumX += rx;
    staticSumY += ry;
    staticSumZ += rz;
    staticSampleCount++;
    
    if (millis() - staticStartTime > 1500 && staticSampleCount > 100) {
      gyroOffsetX = (float)staticSumX / staticSampleCount;
      gyroOffsetY = (float)staticSumY / staticSampleCount;
      gyroOffsetZ = (float)staticSumZ / staticSampleCount;
      
      staticStartTime = millis();
      staticSumX = 0; staticSumY = 0; staticSumZ = 0;
      staticSampleCount = 0;
    }
  } else {
    isStatic = false;
  }
  
  lastRx = rx;
  lastRy = ry;
  lastRz = rz;
}

// ═════════════════════════════════════════════════════════
//  Keyboard helper
// ═════════════════════════════════════════════════════════

void setKey(bool active, bool &oldState, uint8_t key){
  if(active && !oldState)  Keyboard.press(key);
  if(!active && oldState)  Keyboard.release(key);
  oldState = active;
}

// ═════════════════════════════════════════════════════════
//  Joystick & Mode Toggle (FIXED WITH DEBOUNCE)
// ═════════════════════════════════════════════════════════

void updateJoystick(){
  // --- DEBOUNCED BUTTON LOGIC ---
  static unsigned long btnDownTime = 0;
  static bool isBtnDown = false;
  static bool actionFired = false;
  static unsigned long lastDebounceTime = 0;
  static bool lastRawState = HIGH;

  bool rawState = digitalRead(JOY_SW_PIN);
  
  // Filter out microscopic physical bounces
  if (rawState != lastRawState) {
    lastDebounceTime = millis();
  }
  lastRawState = rawState;

  // Once the button signal is stable for 30ms, we process it
  if ((millis() - lastDebounceTime) > 30) {
    bool isPressed = (rawState == LOW);

    if (isPressed && !isBtnDown) {
      // Button was just pushed down solidly
      isBtnDown = true;
      btnDownTime = millis();
      actionFired = false;
    } 
    else if (isPressed && isBtnDown) {
      // Button is being held continuously
      if (!actionFired && (millis() - btnDownTime > 1200)) {
        // --- LONG PRESS DETECTED: TOGGLE MODE ---
        currentModeAsphalt = !currentModeAsphalt;
        actionFired = true; // Lock it so Jump won't trigger on release
        
        // Safety: Release all inputs to prevent stuck keys
        Keyboard.releaseAll();
        Mouse.release(MOUSE_LEFT);
        Mouse.release(MOUSE_RIGHT);
        oldForward = false;
        oldBackward = false;
        oldLeft = false;
        oldRight = false;
        oldSprint = false;
        steerLeftHeld = false;
        steerRightHeld = false;
        zoomWasPressed = false;

        // Show visual confirmation on LEDs
        if (currentModeAsphalt) {
          for (int i = 0; i < 4; i++) {
            analogWrite(LED1, 255); analogWrite(LED2, 255); delay(80);
            analogWrite(LED1, 0);   analogWrite(LED2, 0);   delay(80);
          }
          Serial.println("MODE: Asphalt (Potentiometer Steering Active)");
        } else {
          analogWrite(LED1, 255); analogWrite(LED2, 255);
          delay(600);
          analogWrite(LED1, 0);   analogWrite(LED2, 0);
          Serial.println("MODE: FPS (Gyro Aiming active)");
        }
      }
    } 
    else if (!isPressed && isBtnDown) {
      // Button was released solidly
      isBtnDown = false;
      
      // If we didn't toggle modes, it was a short press
      if (!actionFired) {
        if (!currentModeAsphalt) {
          // --- SHORT PRESS DETECTED: JUMP ---
          Keyboard.press(' ');
          delay(50);
          Keyboard.release(' ');
        }
      }
    }
  }
  // --- END BUTTON LOGIC ---

  // If in Asphalt mode, joystick WASD is completely disabled
  if (currentModeAsphalt) {
    return;
  }

  // FPS Mode Joystick behavior
  int x = analogRead(JOY_X_PIN), y = analogRead(JOY_Y_PIN);
  int dx = x - centerJoyX, dy = y - centerJoyY;
  int ax = abs(dx), ay = abs(dy);

  bool forward=false, backward=false, left=false, right=false;

  if(ax < JOY_THRESHOLD && ay < JOY_THRESHOLD){
    // inside deadzone
  } else if(ay > ax){
    if(dy > JOY_THRESHOLD)       right = true;
    else if(dy < -JOY_THRESHOLD) left  = true;
  } else {
    if(dx < -JOY_THRESHOLD)      forward  = true;
    else if(dx > JOY_THRESHOLD)  backward = true;
  }

  setKey(forward,  oldForward,  'w');
  setKey(backward, oldBackward, 's');
  setKey(left,     oldLeft,     'a');
  setKey(right,    oldRight,    'd');
}

// ═════════════════════════════════════════════════════════
//  Potentiometer → Steers Left (A) / Right (D)
// ═════════════════════════════════════════════════════════

void updatePotentiometer(){
  if (!currentModeAsphalt) {
    if (steerLeftHeld)  { Keyboard.release('a'); steerLeftHeld = false; }
    if (steerRightHeld) { Keyboard.release('d'); steerRightHeld = false; }
    return;
  }

  int potVal = analogRead(POT_PIN);

  bool steerLeft  = (potVal < (POT_STEER_CENTER - POT_STEER_DEADZONE));
  bool steerRight = (potVal > (POT_STEER_CENTER + POT_STEER_DEADZONE));

  if (steerLeft && !steerLeftHeld) {
    Keyboard.press('a');
    steerLeftHeld = true;
  } else if (!steerLeft && steerLeftHeld) {
    Keyboard.release('a');
    steerLeftHeld = false;
  }

  if (steerRight && !steerRightHeld) {
    Keyboard.press('d');
    steerRightHeld = true;
  } else if (!steerRight && steerRightHeld) {
    Keyboard.release('d');
    steerRightHeld = false;
  }
}

// ═════════════════════════════════════════════════════════
//  Gyro Mouse Aiming (Only active in FPS Mode)
// ═════════════════════════════════════════════════════════

void updateGyro(){
  if (currentModeAsphalt) {
    accumX = 0;
    accumY = 0;
    return;
  }

  int16_t rx, ry, rz;
  readGyro(rx, ry, rz);

  updateOnlineCalibration(rx, ry, rz);

  float gx = (float)rx - gyroOffsetX;
  float gy = (float)ry - gyroOffsetY;
  float gz = (float)rz - gyroOffsetZ;

  float inputX = -gz;
  float inputY = -gy;

  if (inputX > GYRO_DEADZONE) {
    inputX -= GYRO_DEADZONE;
  } else if (inputX < -GYRO_DEADZONE) {
    inputX += GYRO_DEADZONE;
  } else {
    inputX = 0;
  }

  if (inputY > GYRO_DEADZONE) {
    inputY -= GYRO_DEADZONE;
  } else if (inputY < -GYRO_DEADZONE) {
    inputY += GYRO_DEADZONE;
  } else {
    inputY = 0;
  }

  float moveX = (inputX / GYRO_LSB_PER_DPS) * AIM_SENSITIVITY_X;
  float moveY = (inputY / GYRO_LSB_PER_DPS) * AIM_SENSITIVITY_Y;

  if(fabs(moveX) < MOUSE_MIN_MOVE) moveX = 0;
  if(fabs(moveY) < MOUSE_MIN_MOVE) moveY = 0;

  accumX += moveX;
  accumY += (-moveY);

  int mx = (int)accumX;
  int my = (int)accumY;

  if(mx >  MAX_MOUSE_STEP) mx =  MAX_MOUSE_STEP;
  if(mx < -MAX_MOUSE_STEP) mx = -MAX_MOUSE_STEP;
  if(my >  MAX_MOUSE_STEP) my =  MAX_MOUSE_STEP;
  if(my < -MAX_MOUSE_STEP) my = -MAX_MOUSE_STEP;

  accumX -= mx;
  accumY -= my;

  if (moveX == 0) accumX = 0;
  if (moveY == 0) accumY = 0;

  if(mx || my) Mouse.move(mx, my, 0);
}

// ═════════════════════════════════════════════════════════
//  LED animations (non-blocking)
// ═════════════════════════════════════════════════════════

void updateLeds(){
  unsigned long now = millis();

  switch(ledState){
    case 0:
      if(now - lastLedTime >= CHASE_INTERVAL){
        lastLedTime = now;
        if(chaseToggle){
          analogWrite(LED1, 0);
          analogWrite(LED2, 255);
        } else {
          analogWrite(LED1, 255);
          analogWrite(LED2, 0);
        }
        chaseToggle = !chaseToggle;
        chaseCount++;
        if(chaseCount >= CHASE_BLINKS * 2){
          chaseCount = 0;
          brightness = 0;
          ledState = 1;
          analogWrite(LED1, 0);
          analogWrite(LED2, 0);
        }
      }
      break;

    case 1:
      if(now - lastLedTime >= FADE_INTERVAL){
        lastLedTime = now;
        brightness += FADE_STEP;
        if(brightness >= 255){
          brightness = 255;
          ledState = 2;
        }
        analogWrite(LED1, brightness);
        analogWrite(LED2, brightness);
      }
      break;

    case 2:
      if(now - lastLedTime >= FADE_INTERVAL){
        lastLedTime = now;
        brightness -= FADE_STEP;
        if(brightness <= 0){
          brightness = 0;
          ledState = 0;
        }
        analogWrite(LED1, brightness);
        analogWrite(LED2, brightness);
      }
      break;
  }
}

// ═════════════════════════════════════════════════════════
//  Buttons: Trigger, Zoom, Reload + re-center combo
// ═════════════════════════════════════════════════════════

void updateButtons(){
  bool trigger = digitalRead(TRIGGER_PIN) == LOW;
  bool zoom    = digitalRead(ZOOM_PIN)    == LOW;
  bool reload  = digitalRead(RELOAD_PIN)  == LOW;

  // Re-center combo: hold Zoom + Reload for 0.5 s
  if (!currentModeAsphalt && zoom && reload){
    if(!comboActive){
      comboActive   = true;
      comboStartTime = millis();
    } else if(millis() - comboStartTime >= 500){
      centerCursor();
      comboActive = false;
      while(digitalRead(ZOOM_PIN)==LOW || digitalRead(RELOAD_PIN)==LOW){
        delay(10);
      }
      return;
    }
  } else {
    comboActive = false;
  }

  if (currentModeAsphalt) {
    // ──────── ASPHALT MODE BUTTONS ────────
    if (Mouse.isPressed(MOUSE_LEFT))  Mouse.release(MOUSE_LEFT);
    if (Mouse.isPressed(MOUSE_RIGHT)) Mouse.release(MOUSE_RIGHT);

    // 1. Zoom -> Progressive Throttle (W)
    if (zoom) {
      if (!zoomWasPressed) {
        zoomWasPressed = true;
        zoomPressStart = millis();
        throttleCycleStart = millis();
      }
      
      unsigned long elapsed = millis() - zoomPressStart;
      unsigned long dutyCycle = 30 + (elapsed * 70 / 1500); 
      if (dutyCycle > 100) dutyCycle = 100;

      unsigned long cycleElapsed = millis() - throttleCycleStart;
      if (cycleElapsed >= THROTTLE_CYCLE_PERIOD) {
        throttleCycleStart = millis();
        cycleElapsed = 0;
      }

      unsigned long activeDuration = (THROTTLE_CYCLE_PERIOD * dutyCycle) / 100;

      if (cycleElapsed < activeDuration) {
        if (!oldForward) {
          Keyboard.press('w');
          oldForward = true;
        }
      } else {
        if (oldForward) {
          Keyboard.release('w');
          oldForward = false;
        }
      }
    } else {
      if (zoomWasPressed) {
        zoomWasPressed = false;
        if (oldForward) {
          Keyboard.release('w');
          oldForward = false;
        }
      }
    }

    // 2. Reload -> Brake (S)
    setKey(reload, oldBackward, 's');

    // 3. Trigger -> Boost / Nitro (Spacebar)
    setKey(trigger, oldSprint, ' ');

  } else {
    // ──────── FPS MODE BUTTONS ────────

    // Trigger -> Left Click (Shoot)
    if(trigger){
      if(!Mouse.isPressed(MOUSE_LEFT)) Mouse.press(MOUSE_LEFT);
    } else if(Mouse.isPressed(MOUSE_LEFT)){
      Mouse.release(MOUSE_LEFT);
    }

    // Zoom -> Right Click (Zoom)
    if(zoom){
      if(!Mouse.isPressed(MOUSE_RIGHT)) Mouse.press(MOUSE_RIGHT);
    } else if(Mouse.isPressed(MOUSE_RIGHT)){
      Mouse.release(MOUSE_RIGHT);
    }

    // Reload -> R Key
    if(reload && !oldReload){
      Keyboard.press('r');
      reloadPressTime = millis();
      reloadKeyHeld   = true;
    }
    oldReload = reload;

    if(reloadKeyHeld && (millis() - reloadPressTime >= 50)){
      Keyboard.release('r');
      reloadKeyHeld = false;
    }
  }
}

// ═════════════════════════════════════════════════════════
//  Setup
// ═════════════════════════════════════════════════════════

void setup(){
  Serial.begin(115200);
  delay(2000);
  analogReadResolution(12);

  pinMode(JOY_SW_PIN,  INPUT_PULLUP);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(ZOOM_PIN,    INPUT_PULLUP);
  pinMode(RELOAD_PIN,  INPUT_PULLUP);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  analogWrite(LED1, 0);
  analogWrite(LED2, 0);

  Wire.begin();
  writeRegister(PWR_MGMT_1, 0x00);
  delay(100);
  writeRegister(CONFIG_REG, 0x03);
  writeRegister(SMPLRT_DIV, 0x01);
  writeRegister(GYRO_CONFIG, 0x00);
  delay(100);

  calibrateGyro();
  calibrateJoystick();

  Mouse.begin();
  Keyboard.begin();
  delay(500);

  centerCursor();
  Serial.println("\nCONTROLLER READY");
}

// ═════════════════════════════════════════════════════════
//  Main loop
// ═════════════════════════════════════════════════════════

void loop(){
  updateJoystick();
  updatePotentiometer();
  updateGyro();
  updateButtons();
  updateLeds();
  delay(4);
}
