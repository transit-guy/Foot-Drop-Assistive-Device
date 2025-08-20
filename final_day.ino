#include <Wire.h>
#include <MPU6050.h>

float targetPitch = 0.0f;
const int servoPin = 18;

#define MPU_ADDR 0x68
MPU6050 mpu(MPU_ADDR);

float axF=0, ayF=0, azF=0;
float pitch=0;

float gyroBiasX=0, gyroBiasY=0, gyroBiasZ=0;

const float LPF_ALPHA = 0.3f;
const float CF_ALPHA  = 0.95f;

const int MAX_CALIBRATION_SAMPLES = 2000;
float pitchBuffer[MAX_CALIBRATION_SAMPLES];
int sampleIndex = 0;
bool calibrationDone = false;
bool calibrationStarted = false;

int startPitchInt = 0;
int cycleCount = 0;
const int minSamplesBeforeCheck = 50;
const int desiredCycles = 2;
int count = 0;

float pitchMin = 0;
float pitchMax = 0;
float pitchStartThreshold = 0;
float pitchStopThreshold = 0;

enum MotorState {
  IDLE,
  ROTATE_FORWARD,
  ROTATE_BACKWARD
};

const int buttonForwardPin = 15;   // Button to force forward
const int buttonBackwardPin = 14;  // Button to force backward

void holdPosition(int pulseWidth) {
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(servoPin, LOW);
  delayMicroseconds(20000 - pulseWidth);
}

void setup() {
  Serial.begin(250000);
  pinMode(servoPin, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(buttonForwardPin, INPUT_PULLUP);
  pinMode(buttonBackwardPin, INPUT_PULLUP);
  digitalWrite(2, LOW);

  Wire.begin();

  mpu.initialize();
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

  if (!mpu.testConnection()) {
    Serial.println("MPU FAIL!");
    while(1);
  }
  digitalWrite(2, HIGH);
  delay(1000);
  Serial.println("Hold still: calibrating gyro bias...");
  delay(1000);

  long sumX=0, sumY=0, sumZ=0;
  const int N=200;
  for (int i=0; i<N; i++) {
    int16_t ax,ay,az,gx,gy,gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(2);
  }
  gyroBiasX = sumX/(float)N;
  gyroBiasY = sumY/(float)N;
  gyroBiasZ = sumZ/(float)N;

  Serial.println("Gyro calibration complete.");
  Serial.println("Please begin moving foot—calibration will detect 2 gait cycles automatically.");
  digitalWrite(2, LOW);
  delay(1000);
}

void loop() {
  // Check manual override buttons first
  if (digitalRead(buttonForwardPin) == LOW) {
    Serial.println("Manual override: FORWARD.");
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(servoPin, LOW);
    delayMicroseconds(19500);
    return;
  }
  if (digitalRead(buttonBackwardPin) == LOW) {
    Serial.println("Manual override: BACKWARD.");
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(2500);
    digitalWrite(servoPin, LOW);
    delayMicroseconds(17500);
    return;
  }

  static uint32_t tPrev = 0;
  if (millis() - tPrev < 10) return;
  float dt = (millis() - tPrev) / 1000.0f;
  tPrev = millis();

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  gx -= gyroBiasX;
  gy -= gyroBiasY;
  gz -= gyroBiasZ;

  axF = (1 - LPF_ALPHA) * axF + LPF_ALPHA * ax;
  ayF = (1 - LPF_ALPHA) * ayF + LPF_ALPHA * ay;
  azF = (1 - LPF_ALPHA) * azF + LPF_ALPHA * az;

  float accelPitch = atan2(axF, sqrt(ayF * ayF + azF * azF)) * 180 / PI;
  float gyroX = gx / 65.5f;
  pitch = CF_ALPHA * (pitch + gyroX * dt) + (1 - CF_ALPHA) * accelPitch;

  if (calibrationDone) {
    Serial.print("Pitch: "); Serial.print(pitch, 1);
    Serial.print("°  Thresholds[Start="); Serial.print(pitchStartThreshold, 1);
    Serial.print(", Stop="); Serial.print(pitchStopThreshold, 1);
    Serial.println("]");

    static MotorState motorState = IDLE;
    static unsigned long forwardStartTime = 0;
    static unsigned long forwardDuration = 0;
    static unsigned long backwardStartTime = 0;

    switch (motorState) {
      case IDLE:
        if (pitch <= pitchStartThreshold) {
          Serial.println("Starting servos in forward direction.");
          forwardStartTime = millis();
          motorState = ROTATE_FORWARD;
        }
        break;

      case ROTATE_FORWARD:
        if (pitch > targetPitch) {
          forwardDuration = millis() - forwardStartTime;
          Serial.print("Target reached. Rotated forward for ");
          Serial.print(forwardDuration);
          Serial.println(" ms. Reversing direction.");
          backwardStartTime = millis();
          motorState = ROTATE_BACKWARD;
        }
        break;

      case ROTATE_BACKWARD:
        if (millis() - backwardStartTime >= forwardDuration) {
          Serial.println("Reverse duration complete. Stopping.");
          motorState = IDLE;
        }
        break;
    }

    if (motorState == ROTATE_FORWARD) {
      digitalWrite(servoPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(servoPin, LOW);
      delayMicroseconds(19500);
    } else if (motorState == ROTATE_BACKWARD) {
      digitalWrite(servoPin, HIGH);
      delayMicroseconds(2500);
      digitalWrite(servoPin, LOW);
      delayMicroseconds(17500);
    } else {
      digitalWrite(servoPin, HIGH);
      delayMicroseconds(1500);
      digitalWrite(servoPin, LOW);
      delayMicroseconds(18500);
    }

    return;
  }

  // Calibration logic unchanged
  if (!calibrationStarted) {
    static float lastPitch = 0;
    if (count < 1) {
      lastPitch = pitch;
      targetPitch = pitch + 0.2;
      count++;
    }
    if (abs(pitch - lastPitch) > 1.5f) {
      startPitchInt = (int)(pitch + 0.5f);
      calibrationStarted = true;
      sampleIndex = 0;
      cycleCount = 0;
      Serial.print("Calibration started. Start pitch integer: ");
      Serial.println(startPitchInt);
      digitalWrite(2, HIGH);
    }
    lastPitch = pitch;
    return;
  }

  if (sampleIndex < MAX_CALIBRATION_SAMPLES) {
    pitchBuffer[sampleIndex++] = pitch;
  } else {
    Serial.println("Error: Buffer overflow.");
    calibrationDone = true;
    return;
  }

  if (sampleIndex > minSamplesBeforeCheck) {
    int currentPitchInt = (int)(pitch + 0.5f);
    if (currentPitchInt == startPitchInt) {
      cycleCount++;
      Serial.print("Detected return to start pitch (Cycle ");
      Serial.print(cycleCount);
      Serial.println(").");
      delay(300);
      if (cycleCount >= desiredCycles) {
        pitchMin = pitchBuffer[0];
        pitchMax = pitchBuffer[0];
        for (int i = 1; i < sampleIndex; i++) {
          if (pitchBuffer[i] < pitchMin) pitchMin = pitchBuffer[i];
          if (pitchBuffer[i] > pitchMax) pitchMax = pitchBuffer[i];
        }
        pitchStartThreshold = pitchMin + 0.4 * (pitchMax - pitchMin);
        pitchStopThreshold = pitchMin + 0.6 * (pitchMax - pitchMin);
        Serial.println("Calibration complete!");
        Serial.print("Pitch min: "); Serial.println(pitchMin, 1);
        Serial.print("Pitch max: "); Serial.println(pitchMax, 1);
        Serial.print("Start Threshold: "); Serial.println(pitchStartThreshold, 1);
        Serial.print("Stop Threshold: "); Serial.println(pitchStopThreshold, 1);
        calibrationDone = true;
        digitalWrite(2, LOW);
      }
    }
  }
}
