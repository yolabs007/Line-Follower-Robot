/*
ChatGPT said:
STEP-1: Motor Bring-Up (PWM) — Wiring + What to Expect

Goal: verify motor wiring and PWM control before adding sensors/IMU/junction logic.

Connections (ESP32 ↔ Motor Driver)

Driver type: L298N / TB6612 or similar (2 inputs per motor).

Pin map (ESP32 → driver inputs):

LF (25) → Left motor “forward” input

LB (26) → Left motor “backward” input

RF (27) → Right motor “forward” input

RB (14) → Right motor “backward” input

Power:

Motors V+: your battery (per motor spec) into driver VIN.

Logic VCC: as per driver (5V on L298N, 3.3–5V on TB6612).

Grounds: battery GND, driver GND, and ESP32 GND must be common.

Driver jumpers (if L298N):

Leave ENA/ENB jumpered (we’re PWM’ing IN pins). Remove jumpers only if you plan to PWM ENA/ENB later.

Polarity: If a wheel spins the wrong way, swap that motor’s leads (or swap LF/LB pins in code).

What the test code does

Forward (1.5 s) — both wheels forward at FAST speed

Stop (0.5 s)

Back (1.5 s) — both wheels backward at FAST speed

Stop (0.5 s)

Left turn in place (0.9 s) — left wheel back, right wheel forward

Stop (0.5 s)

Right turn in place (0.9 s) — left wheel forward, right wheel back

Stop (1.0 s) and repeats

Serial Monitor shows: Forward → Back → Left turn → Right turn for quick confirmation.

Safe test procedure

Lift the robot so wheels are off the ground for the first run.

Use a proper battery or bench supply rated for your motors.

Keep motor wires thick/short; verify grounds are common.

Expected results

Wheels spin smoothly in the listed sequence.

No overheating on driver; motions are symmetrical.

If one side is faster, that’s normal—will be corrected later with PID.

Quick troubleshooting

Nothing moves: check common ground, driver VCC/VIN, and that your board has analogWrite support (update ESP32 board package if needed).

Only one direction works: verify both inputs per motor are wired; ensure one input is PWM’d while the opposite is 0 (as in code).

Spins wrong way: swap that motor’s leads or swap LF↔LB (or RF↔RB) in wiring/code.

Brownouts/resets: battery sag—use a higher-current pack or add bulk caps on driver VIN.

Next (STEP-2): add 4 center line sensors and a simple P-controller to follow the line; then we’ll extend to 8 sensors, junction detect (90°), and U-turns at dead-ends.

*/






// ===== Basic 4-pin motor test using analogWrite() (ESP32) =====
// Motor driver inputs (two per motor: forward/back)
const int LF = 25;  // Left motor forward
const int LB = 26;  // Left motor backward
const int RF = 27;  // Right motor forward
const int RB = 14;  // Right motor backward

// Helper: clamp 0..255
inline int clamp255(int v){ return v < 0 ? 0 : (v > 255 ? 255 : v); }

// Stop/coast everything
void stopMotors() {
  analogWrite(LF, 0);
  analogWrite(LB, 0);
  analogWrite(RF, 0);
  analogWrite(RB, 0);
}

// Drive helpers (speed 0..255)
void forward(int speed) {
  speed = clamp255(speed);
  analogWrite(LF, speed);
  analogWrite(LB, 0);
  analogWrite(RF, speed);
  analogWrite(RB, 0);
}

void back(int speed) {
  speed = clamp255(speed);
  analogWrite(LF, 0);
  analogWrite(LB, speed);
  analogWrite(RF, 0);
  analogWrite(RB, speed);
}

void leftTurnInPlace(int speed) {   // spin CCW
  speed = clamp255(speed);
  analogWrite(LF, 0);      // left wheel backward
  analogWrite(LB, speed);
  analogWrite(RF, speed);  // right wheel forward
  analogWrite(RB, 0);
}

void rightTurnInPlace(int speed) {  // spin CW
  speed = clamp255(speed);
  analogWrite(LF, speed);  // left wheel forward
  analogWrite(LB, 0);
  analogWrite(RF, 0);      // right wheel backward
  analogWrite(RB, speed);
}

void setup() {
  Serial.begin(115200);

  pinMode(LF, OUTPUT);
  pinMode(LB, OUTPUT);
  pinMode(RF, OUTPUT);
  pinMode(RB, OUTPUT);

  stopMotors();
  Serial.println("Motor PWM test (analogWrite) ready.");
}

void loop() {
  const int SLOW = 120;
  const int FAST = 180;

  Serial.println("Forward");
  forward(FAST);  delay(1500);  stopMotors(); delay(500);

  Serial.println("Back");
  back(FAST);     delay(1500);  stopMotors(); delay(500);

  Serial.println("Left turn");
  leftTurnInPlace(SLOW); delay(900); stopMotors(); delay(500);

  Serial.println("Right turn");
  rightTurnInPlace(SLOW); delay(900); stopMotors(); delay(1000);
}
