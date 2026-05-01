#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BNO08x.h>
#include <ESP32Servo.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SVC_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic* p_char = NULL;
bool is_connected = false;

struct TelemetryPacket {
  float p;
  float y;
} __attribute__((packed));
TelemetryPacket my_data;

class ServerHandlers: public BLEServerCallbacks {
    void onConnect(BLEServer* s) {
      is_connected = true;
      Serial.println("connected");
    };
    void onDisconnect(BLEServer* s) {
        is_connected = false;
        BLEDevice::startAdvertising();
    }
};

int sck = 36;
int miso = 37;
int mosi = 35;
int CS = 34;
int IMU_INT = 21; 
int rst = 7;

int BTN = 3; 
int YAW_PIN = 8;
int PITCH_PIN = 9;

volatile bool needs_cal = false; 
volatile unsigned long last_int = 0;

Servo yaw_serv;
Servo p_serv;

Adafruit_BNO08x bno(rst);
sh2_SensorValue_t sensorValue;

bool active = false;
float pitch_ref = 0.0f;
float yaw_ref = 0.0f;

void IRAM_ATTR btn_isr() {
  unsigned long t = millis();
  if (t - last_int > 250) {
    needs_cal = true;
    last_int = t;
  }
}

void fix_quat(float& w, float& x, float& y, float& z) {
  float n = sqrtf(w*w + x*x + y*y + z*z);
  if (n > 0.0f) {
    w /= n; x /= n; y /= n; z /= n;
  }
}

float wrap(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void get_p_y(float w, float x, float y, float z, float& pitch, float& yaw) {
    float sp = 2.0f * (w * y - z * x);
  if (fabsf(sp) >= 1.0f) pitch = copysignf(90.0f, sp);
  else pitch = asinf(sp) * 180.0f / PI;

  float sy = 2.0f * (w * z + x * y);
  float cy = 1.0f - 2.0f * (y * y + z * z);
  yaw = atan2f(sy, cy) * 180.0f / PI;
}

bool run_cal() {
  float p_sum = 0;
  float y_sum_x = 0, y_sum_y = 0;
  int hits = 0;

  Serial.println("calibrating...");
  unsigned long start_t = millis();
  while (millis() - start_t < 800) {
    if (bno.getSensorEvent(&sensorValue)) {
      float w = sensorValue.un.rotationVector.real;
      float x = sensorValue.un.rotationVector.i;
      float y = sensorValue.un.rotationVector.j;
      float z = sensorValue.un.rotationVector.k;
      float p, yw;
      get_p_y(w, x, y, z, p, yw);
      p_sum += p;
      y_sum_x += cosf(yw * PI / 180.0f);
      y_sum_y += sinf(yw * PI / 180.0f);
      hits++;
    }
    delay(5);
  }

  if (hits == 0) return false;
  pitch_ref = p_sum / hits;
  yaw_ref = atan2f(y_sum_y, y_sum_x) * 180.0f / PI;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN), btn_isr, FALLING);
  
  pinMode(IMU_INT, INPUT); 
  pinMode(rst, OUTPUT);
  digitalWrite(rst, HIGH);

  BLEDevice::init("UM_TinyS3_Gimbal");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerHandlers());
  BLEService *pSvc = pServer->createService(SVC_UUID);
  p_char = pSvc->createCharacteristic(CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  p_char->addDescriptor(new BLE2902());
  pSvc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->start();

  SPI.begin(36, 37, 35, 34);
  yaw_serv.setPeriodHertz(333);
  p_serv.setPeriodHertz(333);
  yaw_serv.attach(YAW_PIN, 1150, 1450);
  p_serv.attach(PITCH_PIN, 1150, 1450);
  
  // center
  yaw_serv.writeMicroseconds(1300);
  p_serv.writeMicroseconds(1300);

  if (!bno.begin_SPI(CS, IMU_INT)) {
    Serial.println("imu error");
    while(1);
  }
  bno.enableReport(SH2_ROTATION_VECTOR, 10000);
}

void loop() {
  if (needs_cal) {
      yaw_serv.writeMicroseconds(1300);
      p_serv.writeMicroseconds(1300);
      delay(200);
      if (run_cal()) active = true;
      needs_cal = false; 
  }

  if (!active) {
    delay(5);
    return;
  }

  if (bno.getSensorEvent(&sensorValue)) {
    float w = sensorValue.un.rotationVector.real;
    float x = sensorValue.un.rotationVector.i;
    float y = sensorValue.un.rotationVector.j;
    float z = sensorValue.un.rotationVector.k;
    fix_quat(w,x,y,z);

    float raw_p, raw_y;
    get_p_y(w, x, y, z, raw_p, raw_y);

    pitch_ref = pitch_ref + 0.03f * (raw_p - pitch_ref);
    float diff = wrap(raw_y - yaw_ref);
    yaw_ref = wrap(yaw_ref + 0.03f * diff);

    float pErr = raw_p - pitch_ref;
    float yErr = wrap(raw_y - yaw_ref);

    if (fabsf(pErr) < 0.4f) pErr = 0;
    if (fabsf(yErr) < 0.4f) yErr = 0;

    if (is_connected) {
      my_data.p = pErr;
      my_data.y = yErr;
      p_char->setValue((uint8_t*)&my_data, sizeof(my_data));
      p_char->notify();
    }

    // reverse for mechanics
    float outP = -pErr;
    float outY = -yErr;

    int pUs = 1300 + (int)((outP / 10.0f) * 150.0f);
    int yUs = 1300 + (int)((outY / 10.0f) * 150.0f);

    p_serv.writeMicroseconds(constrain(pUs, 1150, 1450));
    yaw_serv.writeMicroseconds(constrain(yUs, 1150, 1450));
  }
  delay(2);
}