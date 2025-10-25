// ===== STEP-2: Center-4 line following (digital sensors) =====
// Motors (two pins per motor: forward/back)
const int LF = 25, LB = 26, RF = 27, RB = 14;

// Sensors (we'll use only center-4 for now)
const int L2 = 35, L1 = 34, R1 = 33, R2 = 32;

// ---- Tuning ----
int   baseSpeed   = 150;     // 0..255
int   maxSpeed    = 200;     // 0..255
float Kp          = 180.0;   // proportional gain
bool  ACTIVE_LOW  = false;    // true if sensor reads 0 on black line

// ---- Helpers ----
inline int clamp255(int v){ return v < 0 ? 0 : (v > 255 ? 255 : v); }

void drive(int leftPWM, int rightPWM) {
  leftPWM  = clamp255(leftPWM);
  rightPWM = clamp255(rightPWM);

  // Left side
  analogWrite(LF, leftPWM);
  analogWrite(LB, 0);

  // Right side
  analogWrite(RF, rightPWM);
  analogWrite(RB, 0);
}

void stopMotors() {
  analogWrite(LF, 0); analogWrite(LB, 0);
  analogWrite(RF, 0); analogWrite(RB, 0);
}

void setup() {
  Serial.begin(115200);

  // Motors
  pinMode(LF, OUTPUT); pinMode(LB, OUTPUT);
  pinMode(RF, OUTPUT); pinMode(RB, OUTPUT);

  // Sensors (digital read)
  pinMode(L2, INPUT); pinMode(L1, INPUT);
  pinMode(R1, INPUT); pinMode(R2, INPUT);

  stopMotors();
  Serial.println("STEP-2: Center-4 line follower ready.");
}

void loop() {
  // Read sensors (normalize so '1' means "on line/active")
  int sL2raw = digitalRead(L2);
  int sL1raw = digitalRead(L1);
  int sR1raw = digitalRead(R1);
  int sR2raw = digitalRead(R2);

  int l2 = ACTIVE_LOW ? !sL2raw : sL2raw;
  int l1 = ACTIVE_LOW ? !sL1raw : sL1raw;
  int r1 = ACTIVE_LOW ? !sR1raw : sR1raw;
  int r2 = ACTIVE_LOW ? !sR2raw : sR2raw;

  // Weighted position: L2=-3, L1=-1, R1=+1, R2=+3 when active
  int pos = (l2 ? -3 : 0) + (l1 ? -1 : 0) + (r1 ? +1 : 0) + (r2 ? +3 : 0);

  // If nothing sees the line: gentle crawl forward (simple failsafe)
  if (!l2 && !l1 && !r1 && !r2) {
    drive(80, 80);
    debugPrint(l2,l1,r1,r2,pos,0,80,80);
    delay(10);
    return;
  }

  // P-control
  float error = (float)pos;              // negative -> line is left, positive -> right
  int correction = (int)(Kp * error / 10.0);

  int left  = clamp255(baseSpeed - correction);
  int right = clamp255(baseSpeed + correction);

  // Soft clamp to avoid sudden stops
  left  = left  > maxSpeed ? maxSpeed : left;
  right = right > maxSpeed ? maxSpeed : right;

  drive(left, right);

  debugPrint(l2,l1,r1,r2,pos,error,left,right);
  delay(10); // ~100 Hz loop
}

void debugPrint(int l2,int l1,int r1,int r2,int pos,float err,int L,int R){
  static uint32_t t0=0;
  if (millis()-t0 > 100) { // print at ~10 Hz
    t0 = millis();
    Serial.printf("L2=%d L1=%d R1=%d R2=%d | pos=%d err=%.1f | L=%d R=%d\n",
                  l2,l1,r1,r2,pos,err,L,R);
  }
}

