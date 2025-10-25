// ===== STEP-3: Center-4 follow + 90° junctions (ACTIVE_LOW = false) =====
// Motors
const int LF = 25, LB = 26, RF = 27, RB = 14;

// Sensors (Right→Left, pins rise from 19 to 39)
const int R4 = 19, R3 = 23, R2 = 32, R1 = 33, L1 = 34, L2 = 35, L3 = 36, L4 = 39;

// ---- Tuning ----
int   baseSpeed   = 150;     // 0..255
int   maxSpeed    = 200;     // 0..255
float Kp          = 180.0;   // proportional gain (center-4)
bool  ACTIVE_LOW  = false;   // << your sensors are ACTIVE-HIGH: 1 on black

// ---- Helpers ----
inline int clamp255(int v){ return v < 0 ? 0 : (v > 255 ? 255 : v); }

void drive(int leftPWM, int rightPWM) {
  leftPWM  = clamp255(leftPWM);
  rightPWM = clamp255(rightPWM);
  analogWrite(LF, leftPWM);  analogWrite(LB, 0);
  analogWrite(RF, rightPWM); analogWrite(RB, 0);
}
void stopMotors(){ analogWrite(LF,0); analogWrite(LB,0); analogWrite(RF,0); analogWrite(RB,0); }

void spinLeft(int speed){
  speed = clamp255(speed);
  analogWrite(LF, 0); analogWrite(LB, speed);
  analogWrite(RF, speed); analogWrite(RB, 0);
}
void spinRight(int speed){
  speed = clamp255(speed);
  analogWrite(LF, speed); analogWrite(LB, 0);
  analogWrite(RF, 0); analogWrite(RB, speed);
}
void nudgeForward(int speed, int ms){ drive(speed, speed); delay(ms); stopMotors(); }

// Normalize to 1 = ON LINE (black), 0 = white
struct S8 { int L4,L3,L2,L1,R1,R2,R3,R4; };
S8 readS() {
  auto norm = [&](int pin){ int v = digitalRead(pin); return ACTIVE_LOW ? !v : v; };
  return { norm(L4), norm(L3), norm(L2), norm(L1),
           norm(R1), norm(R2), norm(R3), norm(R4) };
}

// Junction/Dead-end logic
bool leftBranch (const S8&s){ return (s.L3 || s.L4) && !(s.R3 || s.R4); }
bool rightBranch(const S8&s){ return (s.R3 || s.R4) && !(s.L3 || s.L4); }
bool crossJunction(const S8&s){ return (s.L3||s.L4) && (s.R3||s.R4); }
bool deadEnd(const S8&s){
  bool any = s.L4||s.L3||s.L2||s.L1||s.R1||s.R2||s.R3||s.R4;
  return !any;
}

// Center-4 P-controller (L2,L1,R1,R2)
void lineFollowCenter(const S8&s){
  int pos = (s.L2?-3:0) + (s.L1?-1:0) + (s.R1?+1:0) + (s.R2?+3:0);
  if (pos==0 && !(s.L2||s.L1||s.R1||s.R2)) { drive(80,80); return; }
  int correction = (int)(Kp * (float)pos / 10.0);
  int L = clamp255(baseSpeed - correction);
  int R = clamp255(baseSpeed + correction);
  if (L>maxSpeed) L=maxSpeed; if (R>maxSpeed) R=maxSpeed;
  drive(L,R);
}

// Turn helpers: spin until center picks the new line
void turnLeftToLine(int spinSpeed){
  nudgeForward(baseSpeed, 150);
  uint32_t ms = millis();
  while (millis() - ms < 2000) {
    spinLeft(spinSpeed);
    S8 s = readS();
    if (s.L1 || s.R1) break;
  }
  nudgeForward(baseSpeed, 150);
}
void turnRightToLine(int spinSpeed){
  nudgeForward(baseSpeed, 150);
  uint32_t ms = millis();
  while (millis() - ms < 2000) {
    spinRight(spinSpeed);
    S8 s = readS();
    if (s.L1 || s.R1) break;
  }
  nudgeForward(baseSpeed, 150);
}
void uTurnToLine(int spinSpeed){
  uint32_t ms = millis();
  while (millis() - ms < 2500) {
    spinLeft(spinSpeed);
    S8 s = readS();
    if (s.L1 || s.R1) break;
  }
  nudgeForward(baseSpeed, 150);
}

void setup() {
  Serial.begin(115200);
  pinMode(LF,OUTPUT); pinMode(LB,OUTPUT); pinMode(RF,OUTPUT); pinMode(RB,OUTPUT);

  pinMode(R4,INPUT); pinMode(R3,INPUT); pinMode(R2,INPUT); pinMode(R1,INPUT);
  pinMode(L1,INPUT); pinMode(L2,INPUT); pinMode(L3,INPUT); pinMode(L4,INPUT);

  stopMotors();
  Serial.println("STEP-3 (ACTIVE_HIGH sensors) ready.");
}

void loop() {
  S8 s = readS();

  if (deadEnd(s)) {
    Serial.println("Dead-end → U-turn");
    uTurnToLine(150);
    return;
  }
  if (leftBranch(s)) {
    Serial.println("Left junction → turn left");
    turnLeftToLine(150);
    return;
  }
  if (rightBranch(s)) {
    Serial.println("Right junction → turn right");
    turnRightToLine(150);
    return;
  }
  if (crossJunction(s)) {
    nudgeForward(baseSpeed, 120); // prefer straight for now
  }

  lineFollowCenter(s);

  static uint32_t t0=0;
  if (millis()-t0>150){
    t0=millis();
    Serial.printf("R4%d R3%d R2%d R1%d | L1%d L2%d L3%d L4%d\n",
      s.R4,s.R3,s.R2,s.R1,s.L1,s.L2,s.L3,s.L4);
  }
}
