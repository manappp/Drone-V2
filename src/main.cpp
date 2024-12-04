#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>
#include <IBusBM.h>

void calibrateESCs();
// MPU6050
MPU6050 mpu;

// Motor control
Servo motor1, motor2, motor3, motor4;

// PID Variables
float roll, pitch, yaw, rollSetpoint = 0, pitchSetpoint = 0, yawSetpoint = 0;
float rollError, pitchError, yawError;
float rollOutput, pitchOutput, yawOutput;
float rollPrevError = 0, pitchPrevError = 0, yawPrevError = 0;
float rollIntegral = 0, pitchIntegral = 0, yawIntegral = 0;
float Kp = 1.5, Ki = 0.01, Kd = 0.2;

// Motor Speeds
int throttle = 1000; // Initial throttle
int motorSpeed1, motorSpeed2, motorSpeed3, motorSpeed4;

// Arming Variables
bool isArmed = false;
const int ARM_CHANNEL = 4;     // Channel 5 (index 4 in zero-based numbering)
const int DISARM_THRESHOLD = 1300;
const int ARM_THRESHOLD = 1700;

// iBus
IBusBM iBus;
int rollInput, pitchInput, throttleInput, yawInput, armInput;

// Time variables
unsigned long prevTime = 0;
float deltaTime;

void setup() {
  Serial.begin(115200);         // For debugging
  Serial1.begin(115200);        // iBus receiver connected to Serial1
  
  // Initialize iBus
  iBus.begin(Serial1);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  // Initialize motors
  motor1.attach(4);   // Motor 1
  motor2.attach(5);   // Motor 2
  motor3.attach(6);   // Motor 3
  motor4.attach(7);   // Motor 4

  // Initialize all motors to minimum throttle
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);

  // Calibrate ESCs
  calibrateESCs();
}

void loop() {
  // Read iBus data including arming channel
  if (iBus.readChannel(0)) { // Ensure data is valid
    rollInput = iBus.readChannel(0) - 1500;     // Roll channel
    pitchInput = iBus.readChannel(1) - 1500;    // Pitch channel
    throttleInput = iBus.readChannel(2);        // Throttle channel
    yawInput = iBus.readChannel(3) - 1500;      // Yaw channel
    armInput = iBus.readChannel(ARM_CHANNEL);   // Arm channel (Channel 5)
    
    // Check arming status
    if (armInput < DISARM_THRESHOLD) {
      isArmed = false;
      // Reset integrals when disarmed to prevent integral windup
      rollIntegral = 0;
      pitchIntegral = 0;
      yawIntegral = 0;
    } else if (armInput > ARM_THRESHOLD) {
      isArmed = true;
    }
  }

  // Only process flight controls if armed
  if (isArmed) {
    // Calculate deltaTime
    unsigned long currentTime = millis();
    deltaTime = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;

    // Read MPU6050 data
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    roll = gx / 131.0;  // Convert gyro data to roll angle
    pitch = gy / 131.0;
    yaw = gz / 131.0;   // Convert gyro data to yaw angle

    // Set setpoints based on iBus input
    rollSetpoint = map(rollInput, -500, 500, -30, 30);   // Scale roll input
    pitchSetpoint = map(pitchInput, -500, 500, -30, 30); // Scale pitch input
    yawSetpoint = map(yawInput, -500, 500, -30, 30);     // Scale yaw input
    throttle = constrain(throttleInput, 1000, 2000);      // Limit throttle

    // Calculate PID for roll
    rollError = rollSetpoint - roll;
    rollIntegral += rollError * deltaTime;
    float rollDerivative = (rollError - rollPrevError) / deltaTime;
    rollOutput = Kp * rollError + Ki * rollIntegral + Kd * rollDerivative;
    rollPrevError = rollError;

    // Calculate PID for pitch
    pitchError = pitchSetpoint - pitch;
    pitchIntegral += pitchError * deltaTime;
    float pitchDerivative = (pitchError - pitchPrevError) / deltaTime;
    pitchOutput = Kp * pitchError + Ki * pitchIntegral + Kd * pitchDerivative;
    pitchPrevError = pitchError;

    // Calculate PID for yaw
    yawError = yawSetpoint - yaw;
    yawIntegral += yawError * deltaTime;
    float yawDerivative = (yawError - yawPrevError) / deltaTime;
    yawOutput = Kp * yawError + Ki * yawIntegral + Kd * yawDerivative;
    yawPrevError = yawError;

    // Update motor speeds based on PID outputs
    motorSpeed1 = throttle + rollOutput + pitchOutput - yawOutput;  // Front-left motor
    motorSpeed2 = throttle - rollOutput + pitchOutput + yawOutput;  // Front-right motor
    motorSpeed3 = throttle + rollOutput - pitchOutput + yawOutput;  // Rear-left motor
    motorSpeed4 = throttle - rollOutput - pitchOutput - yawOutput;  // Rear-right motor

    // Constrain motor speeds
    motorSpeed1 = constrain(motorSpeed1, 1000, 2000);
    motorSpeed2 = constrain(motorSpeed2, 1000, 2000);
    motorSpeed3 = constrain(motorSpeed3, 1000, 2000);
    motorSpeed4 = constrain(motorSpeed4, 1000, 2000);
  } else {
    // If disarmed, set all motors to minimum throttle
    motorSpeed1 = 1000;
    motorSpeed2 = 1000;
    motorSpeed3 = 1000;
    motorSpeed4 = 1000;
  }

  // Write motor speeds
  motor1.writeMicroseconds(motorSpeed1);
  motor2.writeMicroseconds(motorSpeed2);
  motor3.writeMicroseconds(motorSpeed3);
  motor4.writeMicroseconds(motorSpeed4);

  // Debugging output
  Serial.print("Armed: "); Serial.print(isArmed);
  Serial.print(" Arm Input: "); Serial.print(armInput);
  Serial.print(" Roll: "); Serial.print(roll);
  Serial.print(" Pitch: "); Serial.print(pitch);
  Serial.print(" Yaw: "); Serial.print(yaw);
  Serial.print(" Motor1: "); Serial.print(motorSpeed1);
  Serial.print(" Motor2: "); Serial.print(motorSpeed2);
  Serial.print(" Motor3: "); Serial.print(motorSpeed3);
  Serial.print(" Motor4: "); Serial.println(motorSpeed4);
}

void calibrateESCs() {
  Serial.println("Calibrating ESCs...");
  
  // Send maximum throttle
  motor1.writeMicroseconds(2000);
  motor2.writeMicroseconds(2000);
  motor3.writeMicroseconds(2000);
  motor4.writeMicroseconds(2000);
  delay(2000); // Wait for ESCs to recognize max throttle
  
  // Send minimum throttle
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  delay(2000); // Wait for ESCs to recognize min throttle

  Serial.println("ESC Calibration Complete!");
}