/*
  PrecisionDrive - Real-Time Motor Speed Control with PID

  What this code does:
  - Reads motor RPM from encoder pulses
  - Compares actual RPM with target RPM
  - Uses PID to adjust PWM
  - Keeps motor speed stable even when load is applied

  Hardware:
  - ESP32
  - DC geared motor with encoder
  - Motor driver such as TB6612FNG
*/

// =====================================================
// MOTOR DRIVER PINS
// =====================================================

// TB6612FNG motor driver pins
const int MOTOR_PWM_PIN = 25;   // PWM pin connected to PWMA
const int MOTOR_IN1_PIN = 26;   // Direction pin 1 connected to AIN1
const int MOTOR_IN2_PIN = 27;   // Direction pin 2 connected to AIN2
const int MOTOR_STBY_PIN = 14;  // Standby pin connected to STBY

// =====================================================
// ENCODER PINS
// =====================================================

const int ENCODER_A_PIN = 34;   // Encoder channel A
const int ENCODER_B_PIN = 35;   // Encoder channel B

// =====================================================
// PWM SETTINGS
// =====================================================

const int PWM_CHANNEL = 0;
const int PWM_FREQ = 20000;      // 20 kHz PWM
const int PWM_RESOLUTION = 8;    // 8-bit PWM, range 0-255

// =====================================================
// MOTOR / ENCODER SETTINGS
// =====================================================

// Change this based on your motor encoder.
// Example: if your encoder gives 11 pulses per motor shaft revolution,
// and gearbox ratio is 30:1,
// pulses per output shaft revolution = 11 * 30 = 330
const float PULSES_PER_REVOLUTION = 330.0;

// =====================================================
// PID SETTINGS
// =====================================================

// Target motor speed
float targetRPM = 100.0;

// PID constants
// You will tune these values during testing.
float Kp = 2.0;
float Ki = 0.5;
float Kd = 0.1;

// PID variables
float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;

float actualRPM = 0;
float pidOutput = 0;

// PWM output limits
const int MIN_PWM = 0;
const int MAX_PWM = 255;

// =====================================================
// ENCODER VARIABLES
// =====================================================

volatile long encoderPulseCount = 0;

long lastPulseCount = 0;
unsigned long lastRPMTime = 0;

// PID update interval
const unsigned long CONTROL_INTERVAL_MS = 100;

// =====================================================
// TASK HANDLES
// =====================================================

TaskHandle_t motorControlTaskHandle = NULL;
TaskHandle_t serialTaskHandle = NULL;

// =====================================================
// ENCODER INTERRUPT
// =====================================================

void IRAM_ATTR encoderISR() {
  /*
    This interrupt runs whenever encoder channel A changes.

    We read channel B to know direction.
    For only RPM control, direction is not extremely important,
    but this gives signed pulse counting.
  */

  int channelA = digitalRead(ENCODER_A_PIN);
  int channelB = digitalRead(ENCODER_B_PIN);

  if (channelA == channelB) {
    encoderPulseCount++;
  } else {
    encoderPulseCount--;
  }
}

// =====================================================
// MOTOR FUNCTIONS
// =====================================================

void setMotorForward() {
  digitalWrite(MOTOR_IN1_PIN, HIGH);
  digitalWrite(MOTOR_IN2_PIN, LOW);
}

void setMotorReverse() {
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, HIGH);
}

void stopMotor() {
  ledcWrite(PWM_CHANNEL, 0);
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
}

void setMotorPWM(int pwmValue) {
  pwmValue = constrain(pwmValue, MIN_PWM, MAX_PWM);
  ledcWrite(PWM_CHANNEL, pwmValue);
}

// =====================================================
// RPM CALCULATION
// =====================================================

float calculateRPM() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastRPMTime;

  if (elapsedTime == 0) {
    return actualRPM;
  }

  long currentPulseCount;

  noInterrupts();
  currentPulseCount = encoderPulseCount;
  interrupts();

  long pulseDifference = currentPulseCount - lastPulseCount;

  lastPulseCount = currentPulseCount;
  lastRPMTime = currentTime;

  float revolutions = abs(pulseDifference) / PULSES_PER_REVOLUTION;
  float minutes = elapsedTime / 60000.0;

  float rpm = revolutions / minutes;

  return rpm;
}

// =====================================================
// PID CONTROLLER
// =====================================================

float computePID(float target, float current, float deltaTimeSeconds) {
  error = target - current;

  integral += error * deltaTimeSeconds;

  // Prevent integral from growing too much
  integral = constrain(integral, -100, 100);

  derivative = (error - previousError) / deltaTimeSeconds;

  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  previousError = error;

  return output;
}

// =====================================================
// MOTOR CONTROL TASK
// =====================================================

void motorControlTask(void *parameter) {
  int basePWM = 80;

  lastRPMTime = millis();

  while (true) {
    unsigned long startTime = millis();

    actualRPM = calculateRPM();

    float deltaTimeSeconds = CONTROL_INTERVAL_MS / 1000.0;

    pidOutput = computePID(targetRPM, actualRPM, deltaTimeSeconds);

    int finalPWM = basePWM + pidOutput;

    finalPWM = constrain(finalPWM, MIN_PWM, MAX_PWM);

    setMotorForward();
    setMotorPWM(finalPWM);

    Serial.print("Target RPM: ");
    Serial.print(targetRPM);

    Serial.print(" | Actual RPM: ");
    Serial.print(actualRPM);

    Serial.print(" | Error: ");
    Serial.print(error);

    Serial.print(" | PWM: ");
    Serial.println(finalPWM);

    unsigned long elapsed = millis() - startTime;

    if (elapsed < CONTROL_INTERVAL_MS) {
      vTaskDelay(pdMS_TO_TICKS(CONTROL_INTERVAL_MS - elapsed));
    } else {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

// =====================================================
// SERIAL COMMAND TASK
// =====================================================

void serialTask(void *parameter) {
  while (true) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();

      if (input.startsWith("rpm")) {
        String rpmValue = input.substring(3);
        rpmValue.trim();

        float newRPM = rpmValue.toFloat();

        if (newRPM > 0) {
          targetRPM = newRPM;

          integral = 0;
          previousError = 0;

          Serial.print("New target RPM set to: ");
          Serial.println(targetRPM);
        } else {
          Serial.println("Invalid RPM value.");
        }
      }

      else if (input.startsWith("kp")) {
        String value = input.substring(2);
        value.trim();
        Kp = value.toFloat();

        Serial.print("Kp set to: ");
        Serial.println(Kp);
      }

      else if (input.startsWith("ki")) {
        String value = input.substring(2);
        value.trim();
        Ki = value.toFloat();

        Serial.print("Ki set to: ");
        Serial.println(Ki);
      }

      else if (input.startsWith("kd")) {
        String value = input.substring(2);
        value.trim();
        Kd = value.toFloat();

        Serial.print("Kd set to: ");
        Serial.println(Kd);
      }

      else if (input == "stop") {
        stopMotor();
        Serial.println("Motor stopped.");
      }

      else if (input == "help") {
        Serial.println("Commands:");
        Serial.println("rpm 100   -> set target RPM to 100");
        Serial.println("kp 2.0    -> set Kp");
        Serial.println("ki 0.5    -> set Ki");
        Serial.println("kd 0.1    -> set Kd");
        Serial.println("stop      -> stop motor");
      }

      else {
        Serial.println("Unknown command. Type 'help'.");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("PrecisionDrive Starting...");

  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  pinMode(MOTOR_STBY_PIN, OUTPUT);

  digitalWrite(MOTOR_STBY_PIN, HIGH);

  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, CHANGE);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_PWM_PIN, PWM_CHANNEL);

  setMotorForward();
  setMotorPWM(0);

  xTaskCreatePinnedToCore(
    motorControlTask,
    "Motor Control Task",
    4096,
    NULL,
    2,
    &motorControlTaskHandle,
    1
  );

  xTaskCreatePinnedToCore(
    serialTask,
    "Serial Command Task",
    4096,
    NULL,
    1,
    &serialTaskHandle,
    0
  );

  Serial.println("System ready.");
  Serial.println("Type 'help' for commands.");
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  // Empty because FreeRTOS tasks handle everything
}
