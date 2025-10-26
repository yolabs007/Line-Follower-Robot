// ===== Grid Solver — START btn (D5), END LED (D2), TURN_SPEED=100, L/R finish like U-turn =====
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;

// --------- Motor pins ---------
const int LF = 13, LB = 12, RF = 14, RB = 27;

// --------- Sensor pins (left -> right). L3 on GPIO15 ---------
const int L4 = 4,  L3 = 15, L2 = 34, L1 = 35,
          R1 = 32, R2 = 33, R3 = 25, R4 = 26;

// --------- UI pins ---------
const int START_BTN = 5;   // momentary to GND (INPUT_PULLUP)
const int RUN_LED   = 2;   // goes HIGH when END box detected

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
const unsigned long MIN_SPIN_AFTER_GATE_MS = 150; // must spin at least this after gate- increase if you feel turn is not full 

// END box: treat as end if majority white
const int END_WHITE_MIN = 7;   // 7/8 or all-white → END

int TURN_SPEED = 100; // your request

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

// Compact snapshot (1=black, 0=white)
void printSnap(const char* tag){
  char buf[128];
  snprintf(buf, sizeof(buf),
    "%s L4:%d L3:%d L2:%d L1:%d | R1:%d R2:%d R3:%d R4:%d",
    tag, bL4,bL3,bL2,bL1,bR1,bR2,bR3,bR4);
  Serial.println(buf);
  SerialBT.println(buf);
}

// Predicates (black bg / white line)
inline bool allBlack(){ return (bL4&&bL3&&bL2&&bL1&&bR1&&bR2&&bR3&&bR4); }
inline bool allWhite(){ return (!bL4&&!bL3&&!bL2&&!bL1&&!bR1&&!bR2&&!bR3&&!bR4); }

// STRICT side detection: BOTH outers must be white (reduce noise)
inline bool leftSideWhiteAny()  { return (!bL3 && !bL4); }  // L3 & L4 white
inline bool rightSideWhiteAny() { return (!bR3 && !bR4); }  // R3 & R4 white

inline bool leftSideBlack()     { return (bL3||bL4); }
inline bool rightSideBlack()    { return (bR3||bR4); }

// Center present?
inline bool centerAnyWhite()    { return (!bL2 || !bL1 || !bR1 || !bR2); }

// Availability (pre-nudge sides; post-nudge straight)
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
  if(allBlack()) return 2;                // dead-end → U-turn
  bool L = leftAvail();                   // both white (strict)
  bool R = rightAvail();
  if( (L || R) && centerAnyWhite() ) return 1;  // only if center present too
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

// Latching info around a junction
bool sawLeft=false, sawRight=false;  // BEFORE nudge
bool hasStraight=false;              // AFTER nudge (rolling)

// gap between junctions to avoid re-triggering
unsigned long lastDecisionMs = 0;

void enter(int s){ state = s; stateStart = millis(); }

// small rolling sample for center presence without stopping
bool sampleCenterPresence(unsigned samples=5, unsigned perDelayMs=3){
  unsigned yes=0;
  for(unsigned i=0;i<samples;i++){
    readSensors();
    if(straightAvail()) yes++;
    delay(perDelayMs);
  }
  return (yes*2 >= samples); // majority
}

void waitForStartPress(){
  Serial.println("Press START (D5→GND) to begin…");
  SerialBT.println("Press START (D5→GND) to begin…");
  // Require a LOW (pressed) then release to HIGH
  while (digitalRead(START_BTN) == HIGH) { delay(5); } // wait press
  delay(30); // debounce
  while (digitalRead(START_BTN) == LOW)  { delay(5); } // wait release
  Serial.println("Starting run!");
  SerialBT.println("Starting run!");
}

void setup(){
  motorsAllLow(); delay(30);
  Serial.begin(115200);
  SerialBT.begin("GRID-BOT");

  pinMode(L4,INPUT); pinMode(L3,INPUT); pinMode(L2,INPUT); pinMode(L1,INPUT);
  pinMode(R1,INPUT); pinMode(R2,INPUT); pinMode(R3,INPUT); pinMode(R4,INPUT);

  pinMode(START_BTN, INPUT_PULLUP);
  pinMode(RUN_LED, OUTPUT); digitalWrite(RUN_LED, LOW);

  Serial.println("Grid solver: START on D5; END LED on D2; TURN_SPEED=100; L/R finish like U-turn.");
  SerialBT.println("Grid solver ready.");
  waitForStartPress();
  enter(FOLLOW);
}

void loop(){
  // If we ever enter HALT, just stay stopped with LED ON
  if(state==HALT){
    motorsAllLow();
    digitalWrite(RUN_LED, HIGH);
    delay(50);
    return;
  }

  readSensors();

  if(state==FOLLOW){
    if(junctionTriggered()){
      if (millis() - lastDecisionMs < GAP_MS) {
        followInner4_P();
        return;
      }
      // capture sides BEFORE nudge
      sawLeft  = leftAvail();
      sawRight = rightAvail();
      lastDecisionMs = millis();
      enter(NUDGE);
      forwardCruise();
      return;
    }
    // normal corridor follow
    followInner4_P();
    return;
  }

  if(state==NUDGE){
    // keep moving forward; no stop
    forwardCruise();
    if(millis()-stateStart >= NUDGE_MS){
      // AFTER nudge (rolling): quick checks

      // 1) END BOX? (majority white)
      readSensors();
      int whiteCount = (!bL4)+(!bL3)+(!bL2)+(!bL1)+(!bR1)+(!bR2)+(!bR3)+(!bR4);
      if (whiteCount >= END_WHITE_MIN || allWhite()){
        motorsAllLow();
        digitalWrite(RUN_LED, HIGH);
        printSnap("END_BOX");
        Serial.println("END detected → HALT");
        SerialBT.println("END detected → HALT");
        enter(HALT);
        return;
      }

      // 2) Sample center presence (no stop)
      hasStraight = sampleCenterPresence(5, 3);  // ~15 ms majority

      // ---- One-time junction print (snapshot + summary) ----
      readSensors();
      printSnap("SNAP");
      char buf[96];
      snprintf(buf, sizeof(buf),
        "Junction: sawLeft=%d sawRight=%d hasStraight=%d",
        sawLeft, sawRight, hasStraight);
      Serial.println(buf); SerialBT.println(buf);

      // Decision (Left > Straight > Right > U)
      if (sawLeft && hasStraight) { Serial.println("Action: LEFT");  SerialBT.println("Action: LEFT");  enter(TURN_L); }
      else if (!sawLeft && sawRight && hasStraight) { Serial.println("Action: STRAIGHT"); SerialBT.println("Action: STRAIGHT"); enter(GO_STRAIGHT); }
      else if (sawLeft && !hasStraight) { Serial.println("Action: LEFT");  SerialBT.println("Action: LEFT");  enter(TURN_L); }
      else if (sawRight && !hasStraight) { Serial.println("Action: RIGHT"); SerialBT.println("Action: RIGHT"); enter(TURN_R); }
      else if (sawLeft && sawRight && !hasStraight) { Serial.println("Action: LEFT"); SerialBT.println("Action: LEFT"); enter(TURN_L); }
      else if (!sawLeft && !sawRight && hasStraight) { Serial.println("Action: STRAIGHT"); SerialBT.println("Action: STRAIGHT"); enter(GO_STRAIGHT); }
      else { Serial.println("Action: U-TURN"); SerialBT.println("Action: U-TURN"); enter(UTURN); }
    }
    return;
  }

  if(state==GO_STRAIGHT){
    forwardCruise();
    bool sidesBlackOK = leftSideBlack() && rightSideBlack();
    if ((centerAnyWhite() && sidesBlackOK) || (millis()-stateStart >= STRAIGHT_TO)){
      enter(RECENTER);
    }
    return;
  }

  // ===== LEFT TURN: Gate L4 white, finish like U-turn (center capture) =====
  if(state==TURN_L){
    spinLeft(TURN_SPEED);

    static bool gateL=false;
    static unsigned long gateAt=0;
    if(millis()-stateStart < 20) { gateL=false; gateAt=0; }  // reset on entry

    // Gate when outer-left sees white (we've entered the left lane)
    if(!gateL && (bL4==0)){
      gateL = true; gateAt = millis();
      char msg[64]; snprintf(msg,sizeof(msg),"LEFT Gate L4@%lums", gateAt%100000);
      Serial.println(msg); SerialBT.println(msg);
    }

    // After gate: keep rotating until line is centered like U-turn
    if(gateL){
      bool sidesBlackOK = leftSideBlack() && rightSideBlack();
      if( centersBothL1R1White() && sidesBlackOK
          && (millis()-gateAt) >= MIN_SPIN_AFTER_GATE_MS ){
        unsigned long now = millis();
        char msg[64]; snprintf(msg,sizeof(msg),
          "LEFT Finish center aligned@%lums Δ=%lums", now%100000, (now-gateAt));
        Serial.println(msg); SerialBT.println(msg);
        enter(RECENTER);
        return;
      }
    }

    // Safety timeout
    if(millis()-stateStart >= TURN_TO){
      unsigned long now = millis();
      char msg[64]; snprintf(msg,sizeof(msg),"LEFT Timeout@%lums", now%100000);
      Serial.println(msg); SerialBT.println(msg);
      enter(RECENTER);
    }
    return;
  }

  // ===== RIGHT TURN: Gate R4 white, finish like U-turn (center capture) =====
  if(state==TURN_R){
    spinRight(TURN_SPEED);

    static bool gateR=false;
    static unsigned long gateAt=0;
    if(millis()-stateStart < 20) { gateR=false; gateAt=0; }  // reset on entry

    // Gate when outer-right sees white (we've entered the right lane)
    if(!gateR && (bR4==0)){
      gateR = true; gateAt = millis();
      char msg[64]; snprintf(msg,sizeof(msg),"RIGHT Gate R4@%lums", gateAt%100000);
      Serial.println(msg); SerialBT.println(msg);
    }

    // After gate: keep rotating until line is centered like U-turn
    if(gateR){
      bool sidesBlackOK = leftSideBlack() && rightSideBlack();
      if( centersBothL1R1White() && sidesBlackOK
          && (millis()-gateAt) >= MIN_SPIN_AFTER_GATE_MS ){
        unsigned long now = millis();
        char msg[64]; snprintf(msg,sizeof(msg),
          "RIGHT Finish center aligned@%lums Δ=%lums", now%100000, (now-gateAt));
        Serial.println(msg); SerialBT.println(msg);
        enter(RECENTER);
        return;
      }
    }

    // Safety timeout
    if(millis()-stateStart >= TURN_TO){
      unsigned long now = millis();
      char msg[64]; snprintf(msg,sizeof(msg),"RIGHT Timeout@%lums", now%100000);
      Serial.println(msg); SerialBT.println(msg);
      enter(RECENTER);
    }
    return;
  }

  // ===== U-TURN: gate on any center white, finish on (L1&R1 both white) + sides black =====
  if(state==UTURN){
    spinLeft(TURN_SPEED);

    static bool gateC=false; static unsigned long gateAt=0;
    if(millis()-stateStart < 20) { gateC=false; gateAt=0; }

    if(!gateC && centerAnyWhite()){
      gateC=true; gateAt=millis();
      char msg[64]; snprintf(msg,sizeof(msg),"UTURN Gate center@%lums", gateAt%100000);
      Serial.println(msg); SerialBT.println(msg);
    }

    if(gateC){
      bool sidesBlackOK = leftSideBlack() && rightSideBlack();
      if(centersBothL1R1White() && sidesBlackOK && (millis()-gateAt)>=MIN_SPIN_AFTER_GATE_MS){
        unsigned long now = millis();
        char msg[64]; snprintf(msg,sizeof(msg),"UTURN Finish center aligned@%lums Δ=%lums",
                               now%100000, (now-gateAt));
        Serial.println(msg); SerialBT.println(msg);
        enter(RECENTER);
        return;
      }
    }

    if(millis()-stateStart >= UTURN_TO){
      unsigned long now = millis();
      char msg[64]; snprintf(msg,sizeof(msg),"UTURN Timeout@%lums", now%100000);
      Serial.println(msg); SerialBT.println(msg);
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
