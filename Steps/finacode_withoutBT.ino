// ===== Grid Solver + NVS Shortest-Path (NO BT/WiFi) =====
// - START button (D5→GND). Hold ~2s at boot to CLEAR saved path, tap to start run.
// - END box (>=7/8 white) → simplify + save to NVS, stop, LED on D2.
// - If a saved path exists on next power-up, REPLAY it.
// - Left/Right finish like U-turn (center capture). TURN_SPEED=100.

#include <Preferences.h>
Preferences prefs;

// --------- Motor pins ---------
const int LF = 13, LB = 12, RF = 14, RB = 27;

// --------- Sensor pins (left -> right). L3 on GPIO15 ---------
const int L4 = 4,  L3 = 15, L2 = 34, L1 = 35,
          R1 = 32, R2 = 33, R3 = 25, R4 = 26;

// --------- UI pins ---------
const int START_BTN = 5;   // momentary to GND (INPUT_PULLUP)
const int RUN_LED   = 2;   // HIGH = run over (END)

// Polarity from sanity: RAW==1 on black, RAW==0 on white
bool ACTIVE_LOW = false;

// ---- Follow tuning ----
int   baseSpeed = 100;     // 0..255
int   maxSpeed  = 120;     // 0..255
float Kp        = -180.0;  // negative per your setup

// ---- Timing ----
const unsigned long HOLD_MS     = 150;    // debounce (junction trigger)
const unsigned long NUDGE_MS    = 160;    // ~4–5 cm (rolling)
const unsigned long TURN_TO     = 900;    // 90°
const unsigned long UTURN_TO    = 2000;   // 180°
const unsigned long STRAIGHT_TO = 400;    // straight confirm
const unsigned long RECENTER_MS = 180;    // lockout
const unsigned long GAP_MS      = 250;    // min gap between junctions
const unsigned long MIN_SPIN_AFTER_GATE_MS = 150; // minimal spin after gate

// END box: treat as end if majority white
const int END_WHITE_MIN = 7;   // 7/8 or all-white → END

int TURN_SPEED = 100; // your request

// ---- NVS keys ----
namespace KV {
  const char* NS   = "storage";
  const char* PATH = "path";
}

// ---- Record/Replay state ----
String rawPath   = "";  // recorded during explore
String savedPath = "";  // read from NVS (shortest)
bool   replayMode = false; // true if we have a saved path
int    pathIndex  = 0;     // replay cursor

// ---- helpers ----
inline void motorsAllLow(){
  pinMode(LF, OUTPUT); digitalWrite(LF, LOW);
  pinMode(LB, OUTPUT); digitalWrite(LB, LOW);
  pinMode(RF, OUTPUT); digitalWrite(RF, LOW);
  pinMode(RB, OUTPUT); digitalWrite(RB, LOW);
}
inline int clamp255(int v){ return v<0?0:(v>255?255:v); }
inline void drive(int L, int R){
  L = clamp255(L); R = clamp255(R);
  analogWrite(LF, L); analogWrite(LB, 0);
  analogWrite(RF, R); analogWrite(RB, 0);
}
inline void forwardCruise(){ drive(baseSpeed, baseSpeed); }
inline void spinLeft(int pwm){
  pwm = clamp255(pwm);
  analogWrite(LF, 0);   analogWrite(LB, pwm);
  analogWrite(RF, pwm); analogWrite(RB, 0);
}
inline void spinRight(int pwm){
  pwm = clamp255(pwm);
  analogWrite(LF, pwm); analogWrite(LB, 0);
  analogWrite(RF, 0);   analogWrite(RB, pwm);
}
inline int onBlack(int pin){ int v=digitalRead(pin); return ACTIVE_LOW ? !v : v; } // 1=black, 0=white

// Globals: sensor bits (1=black)
int bL4,bL3,bL2,bL1,bR1,bR2,bR3,bR4;
void readSensors(){
  bL4=onBlack(L4); bL3=onBlack(L3); bL2=onBlack(L2); bL1=onBlack(L1);
  bR1=onBlack(R1); bR2=onBlack(R2); bR3=onBlack(R3); bR4=onBlack(R4);
}

void printSnap(const char* tag){
  char buf[128];
  snprintf(buf, sizeof(buf),
    "%s L4:%d L3:%d L2:%d L1:%d | R1:%d R2:%d R3:%d R4:%d",
    tag, bL4,bL3,bL2,bL1,bR1,bR2,bR3,bR4);
  Serial.println(buf);
}

// Predicates (black bg / white line)
inline bool allBlack(){ return (bL4&&bL3&&bL2&&bL1&&bR1&&bR2&&bR3&&bR4); }
inline bool allWhite(){ return (!bL4&&!bL3&&!bL2&&!bL1&&!bR1&&!bR2&&!bR3&&!bR4); }

// STRICT side detection (outer 2 need to be white)
inline bool leftSideWhiteAny()  { return (!bL3 && !bL4); }
inline bool rightSideWhiteAny() { return (!bR3 && !bR4); }
inline bool leftSideBlack()     { return (bL3||bL4); }
inline bool rightSideBlack()    { return (bR3||bR4); }

// Center present?
inline bool centerAnyWhite()    { return (!bL2 || !bL1 || !bR1 || !bR2); }

// Availability
inline bool leftAvail()     { return leftSideWhiteAny(); }
inline bool rightAvail()    { return rightSideWhiteAny(); }
inline bool straightAvail() { return centerAnyWhite(); }

// U-turn finish helper
inline bool centersBothL1R1White(){ return (!bL1 && !bR1); } // both L1 & R1 white

// Inner-4 P follower
void followInner4_P() {
  int wL2=!bL2, wL1=!bL1, wR1=!bR1, wR2=!bR2; // 1 if white
  if(!wL2 && !wL1 && !wR1 && !wR2){ drive(100,100); return; }
  int pos = (wL2?-3:0) + (wL1?-1:0) + (wR1?+1:0) + (wR2?+3:0);
  int corr = (int)(Kp * (float)pos / 10.0);
  int L = clamp255(baseSpeed - corr), R = clamp255(baseSpeed + corr);
  if(L>maxSpeed)L=maxSpeed; if(R>maxSpeed)R=maxSpeed;
  drive(L,R);
}

// --- Junction trigger (pre-nudge) — STRICT sides + center required, debounced ---
// returns: 0 none, 1 side(s)+center, 2 allBlack (dead-end)
int lastTrig=0, stableTrig=0; unsigned long seenAt=0;
int sidesTriggerNow(){
  if(allBlack()) return 2;
  bool L = leftAvail();
  bool R = rightAvail();
  if( (L || R) && centerAnyWhite() ) return 1;
  return 0;
}
bool junctionTriggered(){
  int now = sidesTriggerNow();
  if(now==0){ stableTrig=0; lastTrig=0; return false; }
  if(now!=lastTrig){ lastTrig=now; seenAt=millis(); return false; }
  if(stableTrig!=now && millis()-seenAt>=HOLD_MS) stableTrig=now;
  return (stableTrig!=0);
}

// --- State machine ---
enum { IDLE=-1, FOLLOW=0, NUDGE=1, TURN_L=2, TURN_R=3, UTURN=4, GO_STRAIGHT=6, RECENTER=7, HALT=9 };
int state=IDLE; unsigned long stateStart=0;

// Latching info around a junction (explore mode)
bool sawLeft=false, sawRight=false;  // BEFORE nudge
bool hasStraight=false;              // AFTER nudge (rolling)

// gap between junctions to avoid re-triggering
unsigned long lastDecisionMs = 0;

void enter(int s){ state = s; stateStart = millis(); }

// quick center sample (rolling)
bool sampleCenterPresence(unsigned samples=5, unsigned perDelayMs=3){
  unsigned yes=0;
  for(unsigned i=0;i<samples;i++){
    readSensors();
    if(straightAvail()) yes++;
    delay(perDelayMs);
  }
  return (yes*2 >= samples); // majority
}

// ----- Simplifier: cancels triples to shortest path -----
String simplifyPath(String path){
  bool changed = true;
  while(changed){
    changed = false;
    if(path.indexOf("LUL") != -1){ path.replace("LUL","S"); changed=true; }
    else if(path.indexOf("RUR") != -1){ path.replace("RUR","S"); changed=true; }
    else if(path.indexOf("LUR") != -1){ path.replace("LUR","U"); changed=true; }
    else if(path.indexOf("RUL") != -1){ path.replace("RUL","U"); changed=true; }
    else if(path.indexOf("LUS") != -1){ path.replace("LUS","R"); changed=true; }
    else if(path.indexOf("SUL") != -1){ path.replace("SUL","R"); changed=true; }
    else if(path.indexOf("RUS") != -1){ path.replace("RUS","L"); changed=true; }
    else if(path.indexOf("SUR") != -1){ path.replace("SUR","L"); changed=true; }
    else if(path.indexOf("SUS") != -1){ path.replace("SUS","U"); changed=true; }
  }
  return path;
}

// ----- NVS helpers -----
String readSavedPath(){
  prefs.begin(KV::NS, true);
  String p = prefs.getString(KV::PATH, "");
  prefs.end();
  return p;
}
void writeSavedPath(const String& p){
  prefs.begin(KV::NS, false);
  prefs.putString(KV::PATH, p);
  prefs.end();
}
void clearSavedPath(){
  prefs.begin(KV::NS, false);
  prefs.remove(KV::PATH);
  prefs.end();
  Serial.println("NVS cleared.");
}

// ----- Start/clear button -----
void waitForStartOrClear(){
  Serial.println("Hold START (D5) ~2s at boot to CLEAR path. Tap to start run.");
  unsigned long t0 = millis();
  // Detect long press within ~2.5s
  while(millis() - t0 < 2500){
    if(digitalRead(START_BTN)==LOW){
      unsigned long pressAt = millis();
      while(digitalRead(START_BTN)==LOW){
        if(millis() - pressAt >= 2000){
          clearSavedPath();
          while(digitalRead(START_BTN)==LOW) delay(5); // wait release
          break;
        }
        delay(5);
      }
      break;
    }
    delay(5);
  }
  // Now wait for normal press-release to start
  Serial.println("Press START to begin…");
  while(digitalRead(START_BTN)==HIGH) delay(5);
  delay(30);
  while(digitalRead(START_BTN)==LOW) delay(5);
  Serial.println("Starting run!");
}

void setup(){
  motorsAllLow(); delay(30);
  Serial.begin(115200);

  pinMode(L4,INPUT); pinMode(L3,INPUT); pinMode(L2,INPUT); pinMode(L1,INPUT);
  pinMode(R1,INPUT); pinMode(R2,INPUT); pinMode(R3,INPUT); pinMode(R4,INPUT);

  pinMode(START_BTN, INPUT_PULLUP);
  pinMode(RUN_LED, OUTPUT); digitalWrite(RUN_LED, LOW);

  // Load saved path (if any)
  savedPath = readSavedPath();
  replayMode = (savedPath.length() > 0);
  if(replayMode){
    Serial.printf("Saved shortest path found: %s\n", savedPath.c_str());
  } else {
    Serial.println("No saved path. This run will RECORD raw path.");
  }

  waitForStartOrClear(); // allow clearing before starting

  // Refresh replay flag in case it was cleared
  savedPath = readSavedPath();
  replayMode = (savedPath.length() > 0);
  pathIndex = 0;

  Serial.println(replayMode ? "Mode: REPLAY" : "Mode: EXPLORE");

  enter(FOLLOW);
}

// ===================== MAIN LOOP ======================
void loop(){
  // HALT: done + LED ON (END)
  if(state==HALT){
    motorsAllLow();
    digitalWrite(RUN_LED, HIGH);
    delay(50);
    return;
  }

  readSensors();

  // -------------------- REPLAY MODE --------------------
  if(replayMode){
    if(state==FOLLOW){
      if(junctionTriggered()){
        if(millis()-lastDecisionMs < GAP_MS){ followInner4_P(); return; }
        lastDecisionMs = millis();
        enter(NUDGE);
        forwardCruise();
        return;
      }
      followInner4_P();
      return;
    }

    if(state==NUDGE){
      forwardCruise();
      if(millis()-stateStart >= NUDGE_MS){
        // END box in replay
        readSensors();
        int whiteCount = (!bL4)+(!bL3)+(!bL2)+(!bL1)+(!bR1)+(!bR2)+(!bR3)+(!bR4);
        if (whiteCount >= END_WHITE_MIN || allWhite()){
          motorsAllLow();
          digitalWrite(RUN_LED, HIGH);
          printSnap("END_BOX (REPLAY)");
          Serial.println("END detected (REPLAY) → HALT");
          enter(HALT);
          return;
        }

        // Execute next move
        if(pathIndex >= (int)savedPath.length()){
          motorsAllLow();
          digitalWrite(RUN_LED, HIGH);
          Serial.println("Path finished — HALT.");
          enter(HALT);
          return;
        }

        char mv = savedPath[pathIndex++];
        Serial.printf("REPLAY move: %c\n", mv);
        if(mv=='L'){ enter(TURN_L); }
        else if(mv=='R'){ enter(TURN_R); }
        else if(mv=='S'){ enter(GO_STRAIGHT); }
        else if(mv=='U'){ enter(UTURN); }
        else { enter(GO_STRAIGHT); }
      }
      return;
    }
  }

  // -------------------- EXPLORE MODE --------------------
  if(!replayMode){
    if(state==FOLLOW){
      if(junctionTriggered()){
        if (millis() - lastDecisionMs < GAP_MS) { followInner4_P(); return; }
        sawLeft  = leftAvail();   // BEFORE nudge
        sawRight = rightAvail();
        lastDecisionMs = millis();
        enter(NUDGE);
        forwardCruise();
        return;
      }
      followInner4_P();
      return;
    }

    if(state==NUDGE){
      forwardCruise();
      if(millis()-stateStart >= NUDGE_MS){
        // END BOX? (majority white)
        readSensors();
        int whiteCount = (!bL4)+(!bL3)+(!bL2)+(!bL1)+(!bR1)+(!bR2)+(!bR3)+(!bR4);
        if (whiteCount >= END_WHITE_MIN || allWhite()){
          String shortest = simplifyPath(rawPath);
          prefs.begin(KV::NS, false);
          prefs.putString(KV::PATH, shortest);
          prefs.end();
          motorsAllLow();
          digitalWrite(RUN_LED, HIGH);
          printSnap("END_BOX");
          Serial.printf("END detected → stored shortest: %s\n", shortest.c_str());
          enter(HALT);
          return;
        }

        // Sample center presence (rolling)
        hasStraight = sampleCenterPresence(5, 3);

        // Decide (Left > Straight > Right > U)
        readSensors();
        printSnap("SNAP");
        if (sawLeft && hasStraight)       { rawPath += 'L'; Serial.println("Action: LEFT");     enter(TURN_L); }
        else if (!sawLeft && sawRight && hasStraight) { rawPath += 'S'; Serial.println("Action: STRAIGHT"); enter(GO_STRAIGHT); }
        else if (sawLeft && !hasStraight) { rawPath += 'L'; Serial.println("Action: LEFT");     enter(TURN_L); }
        else if (sawRight && !hasStraight){ rawPath += 'R'; Serial.println("Action: RIGHT");    enter(TURN_R); }
        else if (sawLeft && sawRight && !hasStraight){ rawPath += 'L'; Serial.println("Action: LEFT"); enter(TURN_L); }
        else if (!sawLeft && !sawRight && hasStraight){ rawPath += 'S'; Serial.println("Action: STRAIGHT"); enter(GO_STRAIGHT); }
        else { rawPath += 'U'; Serial.println("Action: U-TURN"); enter(UTURN); }
      }
      return;
    }
  }

  // -------------------- COMMON TURN / STRAIGHT / RECENTER --------------------

  if(state==GO_STRAIGHT){
    forwardCruise();
    bool sidesBlackOK = leftSideBlack() && rightSideBlack();
    if ((centerAnyWhite() && sidesBlackOK) || (millis()-stateStart >= STRAIGHT_TO)){
      enter(RECENTER);
    }
    return;
  }

  // LEFT: gate L4 white → finish when centered (L1&R1 white) + sides black + min time
  if(state==TURN_L){
    spinLeft(TURN_SPEED);

    static bool gateL=false;
    static unsigned long gateAt=0;
    if(millis()-stateStart < 20) { gateL=false; gateAt=0; }  // reset on entry

    if(!gateL && (bL4==0)){
      gateL = true; gateAt = millis();
      Serial.printf("LEFT Gate L4@%lums\n", gateAt%100000);
    }

    if(gateL){
      bool sidesBlackOK = leftSideBlack() && rightSideBlack();
      if( centersBothL1R1White() && sidesBlackOK
          && (millis()-gateAt) >= MIN_SPIN_AFTER_GATE_MS ){
        unsigned long now = millis();
        Serial.printf("LEFT Finish center aligned@%lums Δ=%lums\n",
                      now%100000, (now-gateAt));
        enter(RECENTER);
        return;
      }
    }

    if(millis()-stateStart >= TURN_TO){
      unsigned long now = millis();
      Serial.printf("LEFT Timeout@%lums\n", now%100000);
      enter(RECENTER);
    }
    return;
  }

  // RIGHT: gate R4 white → finish when centered (L1&R1 white) + sides black + min time
  if(state==TURN_R){
    spinRight(TURN_SPEED);

    static bool gateR=false;
    static unsigned long gateAt=0;
    if(millis()-stateStart < 20) { gateR=false; gateAt=0; }  // reset on entry

    if(!gateR && (bR4==0)){
      gateR = true; gateAt = millis();
      Serial.printf("RIGHT Gate R4@%lums\n", gateAt%100000);
    }

    if(gateR){
      bool sidesBlackOK = leftSideBlack() && rightSideBlack();
      if( centersBothL1R1White() && sidesBlackOK
          && (millis()-gateAt) >= MIN_SPIN_AFTER_GATE_MS ){
        unsigned long now = millis();
        Serial.printf("RIGHT Finish center aligned@%lums Δ=%lums\n",
                      now%100000, (now-gateAt));
        enter(RECENTER);
        return;
      }
    }

    if(millis()-stateStart >= TURN_TO){
      unsigned long now = millis();
      Serial.printf("RIGHT Timeout@%lums\n", now%100000);
      enter(RECENTER);
    }
    return;
  }

  // U-TURN: gate center white → finish when centered + sides black + min time
  if(state==UTURN){
    spinLeft(TURN_SPEED);

    static bool gateC=false; static unsigned long gateAt=0;
    if(millis()-stateStart < 20) { gateC=false; gateAt=0; }

    if(!gateC && centerAnyWhite()){
      gateC=true; gateAt=millis();
      Serial.printf("UTURN Gate center@%lums\n", gateAt%100000);
    }

    if(gateC){
      bool sidesBlackOK = leftSideBlack() && rightSideBlack();
      if(centersBothL1R1White() && sidesBlackOK && (millis()-gateAt)>=MIN_SPIN_AFTER_GATE_MS){
        unsigned long now = millis();
        Serial.printf("UTURN Finish center aligned@%lums Δ=%lums\n",
                      now%100000, (now-gateAt));
        enter(RECENTER);
        return;
      }
    }

    if(millis()-stateStart >= UTURN_TO){
      unsigned long now = millis();
      Serial.printf("UTURN Timeout@%lums\n", now%100000);
      enter(RECENTER);
    }
    return;
  }

  if(state==RECENTER){
    followInner4_P();                  // ignore junctions briefly
    if(millis()-stateStart >= RECENTER_MS){
      enter(FOLLOW);
    }
    return;
  }
}
