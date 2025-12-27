#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <Servo.h>

MPU6050 mpu;

Servo servoYaw;
Servo servoPitch;
Servo servoRoll;

#define SERVO_PIN_YAW 10
#define SERVO_PIN_PITCH 5
#define SERVO_PIN_ROLL 4

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

const uint8_t defaultMpuI2cAddress = 0x68;
const uint8_t whoAmIRegisterAddress = 0x75;
const int originalWhoAmIResponse = 0x68;
const uint8_t mpuCloneWhoAmIResponse = 0x70;
const uint8_t mpuDeviceIsNotAvailable = 0xFF;

uint8_t getMpuWhoAmI() {
  Wire.beginTransmission(defaultMpuI2cAddress);
  Wire.write(whoAmIRegisterAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(originalWhoAmIResponse, 1, true);
  if (Wire.available()) {
    return Wire.read();
  }

  return mpuDeviceIsNotAvailable;
}

void verifyDevice() {
  uint8_t who = getMpuWhoAmI();
  Serial.print(F("MPU WHO_AM_I response= 0x"));
  Serial.println(who, HEX);

  if (who == mpuCloneWhoAmIResponse) {
    Serial.println(F("MPU6050 clone detected (WHO_AM_I=0x70), continuing..."));
  } else if (who == originalWhoAmIResponse) {
    Serial.println(F("Original MPU6050 detected (WHO_AM_I=0x68), continuing..."));
  } else if (who == mpuDeviceIsNotAvailable || who == 0x00) {
    Serial.println(F("No valid response from 0x68, check wiring!"));
  } else {
    Serial.println(F("Unknown I2C device on 0x68, trying to continue anyway..."));
  }
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);  //швидкістьт передачі даних з mpu
  Serial.begin(115200);
  while (!Serial) {
  }

  Serial.println("Start...");
  Serial.println("Initializing I2C devices...");
  mpu.initialize();
  verifyDevice();

  Serial.println("Testing device connections...");
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }

  Serial.println("Setting MPU offsets...");
  mpu.setXAccelOffset(3511);
  mpu.setYAccelOffset(3539);
  mpu.setZAccelOffset(11982);
  mpu.setXGyroOffset(-3);
  mpu.setYGyroOffset(-24);
  mpu.setZGyroOffset(-37);


  int retryCount = 0;
  const int maxRetries = 3;

  while (retryCount < maxRetries) {
    Serial.print("Attempting to initialize DMP. Attempt: ");
    Serial.println(retryCount + 1);

    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
      break;
    } else {

      Serial.print("DMP Initialization failed ");
      Serial.print(devStatus);
      Serial.println("");
      retryCount++;
      delay(1000);
    }
  }

  if (devStatus == 0) {
    Serial.println("Enabling DMP...");
    mpu.setDMPEnabled(true);
    mpu.resetFIFO();

    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    servoYaw.attach(SERVO_PIN_YAW);
    servoPitch.attach(SERVO_PIN_PITCH);
    servoRoll.attach(SERVO_PIN_ROLL);

  } else {
    Serial.println("DMP Initialization failed after 3 attempts.");
  }
}

void loop() {
  if (mpu.getFIFOCount() == 1024) {  //щоб запобігти переповненню
    mpu.resetFIFO();
    return;
  }

  if (!dmpReady) {
    return;
  }
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float yawDeg = ypr[0] * 180 / M_PI;
    float pitchDeg = ypr[1] * 180 / M_PI;
    float rollDeg = ypr[2] * 180 / M_PI;


    int yawValue = map(yawDeg, -180, 180, 0, 180);
    int pitchValue = map(pitchDeg, -90, 90, 0, 180);
    int rollValue = map(rollDeg, -90, 90, 0, 180);

    yawValue = constrain(yawValue, 0, 180);
    pitchValue = constrain(pitchValue, 0, 180);
    rollValue = constrain(rollValue, 0, 180);

    servoYaw.write(yawValue);
    servoPitch.write(pitchValue);
    servoRoll.write(rollValue);
  }
}