#include <Arduino.h>
#include <math.h>

// =====================================================
// 9-Channel Thermal Experiment Controller
// v4 - 4 modes + 3-stage PWM control
// CH color map (Right Hand):
//   CH1 = Purple  = Thumb tip
//   CH2 = Green   = Index tip
//   CH3 = Blue    = Middle tip
//   CH4 = Brown   = Ring tip
//   CH5 = White   = Pinky tip
//   CH6 = Orange  = Thumb-palm junction
//   CH7 = Yellow  = Index-palm junction
//   CH8 = Red     = Palm center-lower
//   CH9 = White   = Pinky base / hypothenar
// =====================================================

#define CHANNELS 9

// ---------------- Channel mapping ----------------
// CH1~CH5 = fingertip group
// CH6~CH9 = palm group
// FIX: D2~D10, avoid D0/D1 serial conflict
const int heaterPin[CHANNELS] = {2, 3, 4, 5, 6, 7, 8, 9, 10};
const int tempPin[CHANNELS]   = {A4, A5, A6, A7, A8, A0, A1, A2, A3};

// ---------------- NTC parameters ----------------
const float R_FIXED = 10000.0;
const float R0      = 10000.0;
const float BETA    = 3950.0;
const float T0K     = 298.15;
const float VREF    = 3.3;
const int   ADCMAX  = 4095;

// ---------------- Control parameters ----------------
float hysteresis = 0.5;
float maxSafeTemp = 45.0;

// 3-stage PWM thresholds
// > target-3C  : full power
// target-3C ~ target-0.5C : half power
// target-0.5C ~ target+0.5C : low hold
// > target+0.5C : off
const float PWM_STAGE_FAR  = 3.0;   // below target by this much -> full power
const int   PWM_FULL       = 255;
const int   PWM_HALF       = 128;
const int   PWM_HOLD       = 64;

// 自动刷新
bool verboseMode = true;
unsigned long printInterval = 1500;
unsigned long lastPrintTime = 0;

// ---------------- Runtime channel state ----------------
float currentTemp[CHANNELS] = {0};
int currentPWM[CHANNELS] = {0};
String channelState[CHANNELS] = {
  "OFF","OFF","OFF","OFF","OFF","OFF","OFF","OFF","OFF"
};

// ---------------- Modes ----------------
enum MainMode {
  MODE_IDLE   = 0,
  MODE_BASE   = 1,
  MODE_SINGLE = 2,
  MODE_FLOW   = 3,
  MODE_PET    = 4
};

MainMode currentMode = MODE_IDLE;

// =====================================================
// MODE 1 : BASE TEMPERATURE
// =====================================================
bool mode1Active = false;
float mode1Target = 36.0;

// =====================================================
// MODE 2 : SINGLE POINT TEST
// =====================================================
bool mode2TempSet = false;
float mode2Target = 38.0;
int mode2SelectedPoint = -1;
bool mode2PointRunning = false;
bool mode2PointHolding = false;
unsigned long mode2HoldStart = 0;
const int mode2HoldTimeSec = 5;

// =====================================================
// MODE 3 : FLOW TEST
// =====================================================
bool mode3TempSet = false;
float mode3Target = 38.0;
const int mode3HoldTimeSec = 2;

const int LEN_A = 5;
const int LEN_B = 5;
const int LEN_C = 4;
const int LEN_D = 4;

int SEQ_A[5] = {0, 1, 2, 3, 4};
int SEQ_B[5] = {4, 3, 2, 1, 0};
int SEQ_C[4] = {5, 6, 7, 8};
int SEQ_D[4] = {8, 7, 6, 5};

int activeSequence[CHANNELS] = {0};
int activeSequenceLen = 0;
bool mode3Running = false;

enum FlowState {
  FLOW_NOT_STARTED = 0,
  FLOW_HEATING,
  FLOW_HOLDING,
  FLOW_DONE
};

FlowState flowState[CHANNELS];
unsigned long flowHoldStart[CHANNELS];

// =====================================================
// MODE 4 : CAT PETTING SEQUENCE
// Right hand stroking motion, fingertip to palm
//
// Wave timing (ms):
//   0ms   : CH2+CH3+CH4 ON  (index+middle+ring tips)
//   400ms : CH1+CH5 ON      (thumb+pinky tips)
//   1000ms: CH7+CH9 ON      (index-palm + hypothenar)
//   1400ms: CH2+CH3+CH4 OFF (fingertips leave)
//   1600ms: CH6+CH8 ON      (thumb-palm + palm center)
//   2000ms: CH1+CH5 OFF
//   2400ms: CH6+CH7+CH8+CH9 OFF -> one stroke done
//   Repeat x2 then stop
// =====================================================
bool mode4TempSet  = false;
float mode4Target  = 38.0;
bool mode4Running  = false;
int  mode4RepeatCount = 0;
const int MODE4_TOTAL_REPEATS = 2;

// stroke phase timing
unsigned long mode4StrokeStart = 0;

// channel active flags for mode4
bool mode4ChActive[CHANNELS] = {false};

// =====================================================
// Utility: 3-stage PWM control for a channel
// =====================================================
void applyThreeStageControl(int ch, float target) {
  float t = currentTemp[ch];
  if (t < target - PWM_STAGE_FAR) {
    analogWrite(heaterPin[ch], PWM_FULL);
    currentPWM[ch] = PWM_FULL;
    channelState[ch] = "HEATING";
  } else if (t < target - hysteresis) {
    analogWrite(heaterPin[ch], PWM_HALF);
    currentPWM[ch] = PWM_HALF;
    channelState[ch] = "WARMING";
  } else if (t <= target + hysteresis) {
    analogWrite(heaterPin[ch], PWM_HOLD);
    currentPWM[ch] = PWM_HOLD;
    channelState[ch] = "HOLDING";
  } else {
    analogWrite(heaterPin[ch], 0);
    currentPWM[ch] = 0;
    channelState[ch] = "PAUSE";
  }
}

// =====================================================
// Utility
// =====================================================
float readTemperatureC(int pin) {
  int raw = analogRead(pin);
  float V = raw * VREF / ADCMAX;

  if (V <= 0.001 || V >= VREF - 0.001) {
    return -999.0;
  }

  float R_ntc = R_FIXED * V / (VREF - V);
  float invT = (1.0 / T0K) + (1.0 / BETA) * log(R_ntc / R0);
  float tempC = (1.0 / invT) - 273.15;

  return tempC;
}

void updateAllTemperatures() {
  for (int i = 0; i < CHANNELS; i++) {
    currentTemp[i] = readTemperatureC(tempPin[i]);
  }
}

void setHeaterPWM(int ch, int pwm, const String &stateName) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(heaterPin[ch], pwm);
  currentPWM[ch] = pwm;
  channelState[ch] = stateName;
}

void stopChannel(int ch) {
  analogWrite(heaterPin[ch], 0);
  currentPWM[ch] = 0;
  channelState[ch] = "OFF";
}

void stopAllHeaters() {
  for (int i = 0; i < CHANNELS; i++) {
    stopChannel(i);
  }
}

bool sensorSafe(int ch) {
  return !(currentTemp[ch] < -100 || currentTemp[ch] > 100 || currentTemp[ch] > maxSafeTemp);
}

void resetFlowStates() {
  for (int i = 0; i < CHANNELS; i++) {
    flowState[i] = FLOW_NOT_STARTED;
    flowHoldStart[i] = 0;
  }
}

void resetModeFlags() {
  mode1Active = false;

  mode2TempSet = false;
  mode2SelectedPoint = -1;
  mode2PointRunning = false;
  mode2PointHolding = false;

  mode3TempSet = false;
  mode3Running = false;
  activeSequenceLen = 0;
  resetFlowStates();

  mode4TempSet = false;
  mode4Running = false;
  mode4RepeatCount = 0;
  for (int i = 0; i < CHANNELS; i++) mode4ChActive[i] = false;
}

String modeName() {
  if (currentMode == MODE_BASE)   return "MODE1_BASE";
  if (currentMode == MODE_SINGLE) return "MODE2_SINGLE";
  if (currentMode == MODE_FLOW)   return "MODE3_FLOW";
  if (currentMode == MODE_PET)    return "MODE4_PET";
  return "IDLE";
}

void printRealtimeStatus() {
  Serial.println("==================================================");
  Serial.print("Mode: ");
  Serial.println(modeName());

  Serial.print("TEMP: ");
  for (int i = 0; i < CHANNELS; i++) {
    Serial.print("CH");
    Serial.print(i + 1);
    Serial.print("=");
    if (currentTemp[i] < -100 || currentTemp[i] > 150) {
      Serial.print("ERR");
    } else {
      Serial.print(currentTemp[i], 1);
    }
    Serial.print("  ");
  }
  Serial.println();

  for (int i = 0; i < CHANNELS; i++) {
    Serial.print("CH");
    Serial.print(i + 1);
    Serial.print(" | D");
    Serial.print(heaterPin[i]);
    Serial.print(" | Temp=");
    if (currentTemp[i] < -100 || currentTemp[i] > 150) {
      Serial.print("ERR");
    } else {
      Serial.print(currentTemp[i], 2);
    }
    Serial.print(" | PWM=");
    Serial.print(currentPWM[i]);
    Serial.print(" | State=");
    Serial.println(channelState[i]);
  }
}

void globalSafetyCheck() {
  for (int i = 0; i < CHANNELS; i++) {
    if (!sensorSafe(i)) {
      stopAllHeaters();
      resetModeFlags();
      currentMode = MODE_IDLE;
      Serial.print("Safety stop on CH");
      Serial.println(i + 1);
      return;
    }
  }
}

int pointToChannel(int pointNum) {
  if (pointNum >= 1 && pointNum <= 9) return pointNum - 1;
  return -1;
}

// =====================================================
// MODE 1
// =====================================================
void runMode1() {
  for (int i = 0; i < CHANNELS; i++) {
    if (!sensorSafe(i)) {
      stopAllHeaters();
      resetModeFlags();
      currentMode = MODE_IDLE;
      Serial.println("Mode1 stopped by safety.");
      return;
    }
    applyThreeStageControl(i, mode1Target);
  }
}

// =====================================================
// MODE 2
// =====================================================
void startMode2Point(int pointNum) {
  int ch = pointToChannel(pointNum);
  if (ch < 0) {
    Serial.println("Invalid point. Use 1~9.");
    return;
  }
  stopAllHeaters();
  mode2SelectedPoint = ch;
  mode2PointRunning = true;
  mode2PointHolding = false;
  mode2HoldStart = 0;
  Serial.print("Mode2 start CH");
  Serial.print(ch + 1);
  Serial.print(" target=");
  Serial.println(mode2Target);
}

void runMode2() {
  if (!mode2PointRunning || mode2SelectedPoint < 0) {
    stopAllHeaters();
    return;
  }
  int ch = mode2SelectedPoint;
  for (int i = 0; i < CHANNELS; i++) {
    if (i != ch) stopChannel(i);
  }
  if (!sensorSafe(ch)) {
    stopAllHeaters();
    mode2PointRunning = false;
    Serial.println("Mode2 stopped by safety.");
    return;
  }
  if (!mode2PointHolding) {
    applyThreeStageControl(ch, mode2Target);
    if (currentTemp[ch] >= mode2Target - hysteresis) {
      mode2PointHolding = true;
      mode2HoldStart = millis();
      Serial.print("Mode2 CH");
      Serial.print(ch + 1);
      Serial.println(" reached target, start hold.");
    }
  } else {
    applyThreeStageControl(ch, mode2Target);
    if (millis() - mode2HoldStart >= (unsigned long)mode2HoldTimeSec * 1000UL) {
      stopChannel(ch);
      mode2PointRunning = false;
      mode2PointHolding = false;
      Serial.print("Mode2 CH");
      Serial.print(ch + 1);
      Serial.println(" finished.");
    }
  }
}

// =====================================================
// MODE 3
// =====================================================
void selectFlowSequence(char code) {
  resetFlowStates();
  if (code == 'A') {
    activeSequenceLen = LEN_A;
    for (int i = 0; i < LEN_A; i++) activeSequence[i] = SEQ_A[i];
  } else if (code == 'B') {
    activeSequenceLen = LEN_B;
    for (int i = 0; i < LEN_B; i++) activeSequence[i] = SEQ_B[i];
  } else if (code == 'C') {
    activeSequenceLen = LEN_C;
    for (int i = 0; i < LEN_C; i++) activeSequence[i] = SEQ_C[i];
  } else if (code == 'D') {
    activeSequenceLen = LEN_D;
    for (int i = 0; i < LEN_D; i++) activeSequence[i] = SEQ_D[i];
  }
}

void startMode3Sequence(char code) {
  stopAllHeaters();
  selectFlowSequence(code);
  if (activeSequenceLen <= 0) {
    Serial.println("Invalid sequence.");
    return;
  }
  flowState[0] = FLOW_HEATING;
  mode3Running = true;
  Serial.print("Mode3 sequence ");
  Serial.print(code);
  Serial.print(" start, target=");
  Serial.println(mode3Target);
}

void runMode3() {
  if (!mode3Running) {
    stopAllHeaters();
    return;
  }

  for (int seqIdx = 0; seqIdx < activeSequenceLen; seqIdx++) {
    int ch = activeSequence[seqIdx];
    if (!sensorSafe(ch)) {
      stopAllHeaters();
      mode3Running = false;
      Serial.println("Mode3 stopped by safety.");
      return;
    }

    switch (flowState[seqIdx]) {
      case FLOW_NOT_STARTED:
        stopChannel(ch);
        break;

      case FLOW_HEATING:
        applyThreeStageControl(ch, mode3Target);
        if (currentTemp[ch] >= mode3Target - hysteresis) {
          flowState[seqIdx] = FLOW_HOLDING;
          flowHoldStart[seqIdx] = millis();
          if (seqIdx + 1 < activeSequenceLen && flowState[seqIdx + 1] == FLOW_NOT_STARTED) {
            flowState[seqIdx + 1] = FLOW_HEATING;
          }
          Serial.print("Mode3 CH");
          Serial.print(ch + 1);
          Serial.println(" reached target, next starts.");
        }
        break;

      case FLOW_HOLDING:
        applyThreeStageControl(ch, mode3Target);
        if (millis() - flowHoldStart[seqIdx] >= (unsigned long)mode3HoldTimeSec * 1000UL) {
          stopChannel(ch);
          flowState[seqIdx] = FLOW_DONE;
          Serial.print("Mode3 CH");
          Serial.print(ch + 1);
          Serial.println(" finished hold.");
        }
        break;

      case FLOW_DONE:
        stopChannel(ch);
        break;
    }
  }

  bool allDone = true;
  for (int i = 0; i < activeSequenceLen; i++) {
    if (flowState[i] != FLOW_DONE) { allDone = false; break; }
  }
  if (allDone) {
    mode3Running = false;
    stopAllHeaters();
    Serial.println("Mode3 sequence finished.");
  }
}

// =====================================================
// MODE 4 : CAT PETTING
// Stroke timing (relative to stroke start):
//   0ms   : CH2+CH3+CH4 ON
//   400ms : CH1+CH5 ON
//   1000ms: CH7+CH9 ON
//   1400ms: CH2+CH3+CH4 OFF
//   1600ms: CH6+CH8 ON
//   2000ms: CH1+CH5 OFF
//   2400ms: all OFF -> stroke done
// =====================================================
void startMode4Stroke() {
  stopAllHeaters();
  for (int i = 0; i < CHANNELS; i++) mode4ChActive[i] = false;
  mode4StrokeStart = millis();
  Serial.print("Mode4 stroke ");
  Serial.print(mode4RepeatCount + 1);
  Serial.println("/2 start.");
}

void runMode4() {
  if (!mode4Running) {
    stopAllHeaters();
    return;
  }

  // Safety check
  for (int i = 0; i < CHANNELS; i++) {
    if (!sensorSafe(i)) {
      stopAllHeaters();
      mode4Running = false;
      Serial.println("Mode4 stopped by safety.");
      return;
    }
  }

  unsigned long elapsed = millis() - mode4StrokeStart;

  // Wave ON events
  if (elapsed >= 0) {
    mode4ChActive[1] = true;  // CH2
    mode4ChActive[2] = true;  // CH3
    mode4ChActive[3] = true;  // CH4
  }
  if (elapsed >= 400) {
    mode4ChActive[0] = true;  // CH1
    mode4ChActive[4] = true;  // CH5
  }
  if (elapsed >= 1000) {
    mode4ChActive[6] = true;  // CH7
    mode4ChActive[8] = true;  // CH9
  }
  if (elapsed >= 1600) {
    mode4ChActive[5] = true;  // CH6
    mode4ChActive[7] = true;  // CH8
  }

  // Wave OFF events
  if (elapsed >= 1400) {
    mode4ChActive[1] = false; // CH2
    mode4ChActive[2] = false; // CH3
    mode4ChActive[3] = false; // CH4
  }
  if (elapsed >= 2000) {
    mode4ChActive[0] = false; // CH1
    mode4ChActive[4] = false; // CH5
  }

  // Apply control to each channel
  for (int i = 0; i < CHANNELS; i++) {
    if (mode4ChActive[i]) {
      applyThreeStageControl(i, mode4Target);
    } else {
      stopChannel(i);
    }
  }

  // Stroke complete at 2400ms
  if (elapsed >= 2400) {
    stopAllHeaters();
    for (int i = 0; i < CHANNELS; i++) mode4ChActive[i] = false;
    mode4RepeatCount++;
    Serial.print("Mode4 stroke ");
    Serial.print(mode4RepeatCount);
    Serial.println("/2 done.");

    if (mode4RepeatCount >= MODE4_TOTAL_REPEATS) {
      mode4Running = false;
      Serial.println("Mode4 petting sequence complete.");
    } else {
      // immediately start next stroke
      startMode4Stroke();
    }
  }
}

// =====================================================
// Command parsing
// =====================================================
void printHelp() {
  Serial.println("========== COMMAND HELP ==========");
  Serial.println("Format: mode,param");
  Serial.println("--- MODE 1: Base Temperature ---");
  Serial.println("  1,36        -> all 9 channels to 36C");
  Serial.println("--- MODE 2: Single Point ---");
  Serial.println("  2,38        -> set target 38C");
  Serial.println("  2,7         -> run point 7");
  Serial.println("--- MODE 3: Flow Sequence ---");
  Serial.println("  3,38        -> set target 38C");
  Serial.println("  3,A         -> fingertip forward CH1->5");
  Serial.println("  3,B         -> fingertip reverse CH5->1");
  Serial.println("  3,C         -> palm forward CH6->9");
  Serial.println("  3,D         -> palm reverse CH9->6");
  Serial.println("--- MODE 4: Cat Petting ---");
  Serial.println("  4,38        -> set target 38C");
  Serial.println("  4,start     -> start petting (x2)");
  Serial.println("--- Other ---");
  Serial.println("  stop / status / verbose on / verbose off / help");
  Serial.println("==================================");
}

void handleModeParamCommand(String cmd) {
  int commaPos = cmd.indexOf(',');
  if (commaPos < 0) {
    Serial.println("Invalid format. Use mode,param");
    return;
  }

  String modeStr = cmd.substring(0, commaPos);
  String paramStr = cmd.substring(commaPos + 1);
  modeStr.trim();
  paramStr.trim();

  int mode = modeStr.toInt();

  // ---- MODE 1 ----
  if (mode == 1) {
    float temp = paramStr.toFloat();
    if (temp < 30 || temp > 45) {
      Serial.println("Mode1 temperature out of range.");
      return;
    }
    stopAllHeaters();
    resetModeFlags();
    currentMode = MODE_BASE;
    mode1Target = temp;
    mode1Active = true;
    Serial.print("Mode1 target set to ");
    Serial.println(mode1Target);
    return;
  }

  // ---- MODE 2 ----
  if (mode == 2) {
    bool isNumeric = true;
    for (unsigned int i = 0; i < paramStr.length(); i++) {
      if (!isDigit(paramStr[i])) { isNumeric = false; break; }
    }

    if (!mode2TempSet || currentMode != MODE_SINGLE) {
      float temp = paramStr.toFloat();
      if (temp < 30 || temp > 45) {
        Serial.println("Mode2 temperature out of range.");
        return;
      }
      stopAllHeaters();
      resetModeFlags();
      currentMode = MODE_SINGLE;
      mode2Target = temp;
      mode2TempSet = true;
      Serial.print("Mode2 target set to ");
      Serial.println(mode2Target);
      Serial.println("Now use 2,1 ~ 2,9 to run point.");
    } else {
      if (!isNumeric) { Serial.println("Mode2 point must be 1~9."); return; }
      int point = paramStr.toInt();
      if (point < 1 || point > 9) { Serial.println("Mode2 point must be 1~9."); return; }
      startMode2Point(point);
    }
    return;
  }

  // ---- MODE 3 ----
  if (mode == 3) {
    if (!mode3TempSet || currentMode != MODE_FLOW) {
      float temp = paramStr.toFloat();
      if (temp < 30 || temp > 45) {
        Serial.println("Mode3 temperature out of range.");
        return;
      }
      stopAllHeaters();
      resetModeFlags();
      currentMode = MODE_FLOW;
      mode3Target = temp;
      mode3TempSet = true;
      Serial.print("Mode3 target set to ");
      Serial.println(mode3Target);
      Serial.println("Now use 3,A / 3,B / 3,C / 3,D");
    } else {
      if (paramStr.length() != 1) { Serial.println("Mode3 sequence must be A/B/C/D."); return; }
      char c = toupper(paramStr[0]);
      if (c != 'A' && c != 'B' && c != 'C' && c != 'D') {
        Serial.println("Mode3 sequence must be A/B/C/D.");
        return;
      }
      startMode3Sequence(c);
    }
    return;
  }

  // ---- MODE 4 ----
  if (mode == 4) {
    if (paramStr.equalsIgnoreCase("start")) {
      if (!mode4TempSet || currentMode != MODE_PET) {
        Serial.println("Set temperature first: 4,38");
        return;
      }
      stopAllHeaters();
      mode4Running = true;
      mode4RepeatCount = 0;
      startMode4Stroke();
    } else {
      float temp = paramStr.toFloat();
      if (temp < 30 || temp > 45) {
        Serial.println("Mode4 temperature out of range.");
        return;
      }
      stopAllHeaters();
      resetModeFlags();
      currentMode = MODE_PET;
      mode4Target = temp;
      mode4TempSet = true;
      Serial.print("Mode4 target set to ");
      Serial.println(mode4Target);
      Serial.println("Now use 4,start to begin petting.");
    }
    return;
  }

  Serial.println("Unknown mode. Use 1, 2, 3, or 4.");
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("help"))        { printHelp(); return; }
  if (cmd.equalsIgnoreCase("status"))      { printRealtimeStatus(); return; }
  if (cmd.equalsIgnoreCase("verbose on"))  { verboseMode = true;  Serial.println("Verbose ON");  return; }
  if (cmd.equalsIgnoreCase("verbose off")) { verboseMode = false; Serial.println("Verbose OFF"); return; }
  if (cmd.equalsIgnoreCase("stop")) {
    stopAllHeaters();
    resetModeFlags();
    currentMode = MODE_IDLE;
    Serial.println("System STOP");
    return;
  }
  if (cmd.indexOf(',') >= 0) { handleModeParamCommand(cmd); return; }
  Serial.println("Unknown command. Type help.");
}

// =====================================================
// setup / loop
// =====================================================
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  for (int i = 0; i < CHANNELS; i++) {
    pinMode(heaterPin[i], OUTPUT);
    analogWrite(heaterPin[i], 0);
  }

  Serial.println("9-Channel Thermal Experiment Controller v4 Ready");
  Serial.println("Heater pins: D2~D10 | 3-stage PWM | Mode4: Cat Petting");
  printHelp();
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }

  updateAllTemperatures();
  globalSafetyCheck();

  if (currentMode == MODE_BASE && mode1Active) runMode1();
  else if (currentMode == MODE_SINGLE)         runMode2();
  else if (currentMode == MODE_FLOW)           runMode3();
  else if (currentMode == MODE_PET)            runMode4();

  if (verboseMode && millis() - lastPrintTime > printInterval) {
    lastPrintTime = millis();
    printRealtimeStatus();
  }

  delay(50);
}
