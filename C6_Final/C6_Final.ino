#include <Arduino.h>
#include <HardwareSerial.h>
#include <BLEDevice.h>
#include <Goldelox_Serial_4DLib.h> 

int RX_ULCD = 16; 
int tx_pin = 17;
int reset_ulcd = 15;
#define BTN_PIN 1 
int baud_rate = 9600; 

volatile bool toggle_req = false;
volatile unsigned long last_press = 0;
int debounce = 300;

void IRAM_ATTR handleButton() {
  unsigned long now = millis();
  if (now - last_press > debounce) {
    toggle_req = true;
    last_press = now;
  }
}

struct TelemetryPacket {
    float pitch;
    float yaw;
};

class screen_stuff {
  private:
    HardwareSerial& _ser;
    Goldelox_Serial_4DLib display;
    int rst;
    bool ready;
    bool active; 

    int cur_x;
    int last_y;
    bool is_first;
    unsigned long timer;
    float avg_val;
    int count;

  public:
    screen_stuff(HardwareSerial& s, int r)
        : _ser(s), display(&s), rst(r) {
      active = false;
      cur_x = 5;
      is_first = true;
      timer = 0;
      avg_val = 0;
      count = 0;
    }

    void kill_screen() {
        digitalWrite(rst, LOW); 
        active = false;
        Serial.println("Display OFF");
    }

    void wake_up() {
      digitalWrite(rst, HIGH);
      delay(4000); 

      _ser.write(0x55); 
      delay(100);
      while(_ser.available()) _ser.read();

      display.TimeLimit4D = 30;
      
      // setup the ui
      display.gfx_Cls();
      display.txt_FGcolour(0x780F);
      display.txt_MoveCursor(0, 0);
      display.putstr((char*)"Tremor Intensity");
      display.gfx_Rectangle(4, 19, 123, 120, 0x780F);

      // clear area
      display.gfx_RectangleFilled(5, 20, 122, 119, 0x0000);
      display.gfx_Line(5, 119, 122, 119, 0x780F);
      cur_x = 5;
      last_y = 119;
        
      active = true;
      ready = true;
      is_first = true;
    }

    bool is_on() { return active; }

    void init(long b, int rx, int tx) {
        pinMode(rst, OUTPUT);
        _ser.begin(b, SERIAL_8N1, rx, tx);
        wake_up(); 
    }

    void refresh(float p, float y) {
      if (!active || !ready) return;

      float m = sqrt((p * p) + (y * y));
      avg_val += m;
      count++;

      if (millis() - timer < 50) return;
      timer = millis();

      float val = 0;
      if (count > 0) val = avg_val / count;
      avg_val = 0;
      count = 0;

      // map the Y
      int y_coord = 119 - (int)(val * 16.0f);
      if (y_coord < 20) y_coord = 20;
      if (y_coord > 119) y_coord = 119;

      if (is_first) {
        last_y = y_coord;
        is_first = false;
      } else {
        display.gfx_Line(cur_x - 1, last_y, cur_x, y_coord, 0xFD20);
        last_y = y_coord;
      }
      
      cur_x++;
      if (cur_x >= 123) {
          // clear and restart graph
          display.gfx_RectangleFilled(5, 20, 122, 119, 0x0000);
          display.gfx_Line(5, 119, 122, 119, 0x780F);
          cur_x = 5;
          last_y = 119;
          is_first = true;
      }
    }
};

static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static BLEUUID charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

bool connect_now = false;
bool is_connected = false;
BLERemoteCharacteristic* remote_char;
BLEAdvertisedDevice* target_device;
TelemetryPacket data_in = {0.0f, 0.0f};
volatile bool new_packet = false;

static void ble_callback(BLERemoteCharacteristic* p, uint8_t* data, size_t len, bool isNotify) {
  if (len == sizeof(TelemetryPacket)) {
    memcpy(&data_in, data, sizeof(TelemetryPacket));
    new_packet = true;
  }
}

class MyScanner: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) {
    if (dev.haveServiceUUID() && dev.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      target_device = new BLEAdvertisedDevice(dev);
      connect_now = true;
    }
  }
};

bool doConnect() {
    BLEClient* pClient = BLEDevice::createClient();
    pClient->connect(target_device);
    BLERemoteService* pSvc = pClient->getService(serviceUUID);
    if (pSvc == nullptr) return false;
    remote_char = pSvc->getCharacteristic(charUUID);
    if (remote_char == nullptr) return false;
    if(remote_char->canNotify()) remote_char->registerForNotify(ble_callback);
    is_connected = true;
    return true;
}

HardwareSerial MySerial(1);
screen_stuff my_disp(MySerial, reset_ulcd);

void stage_led(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
  }
  delay(400);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  delay(2000);

  stage_led(1);
  pinMode(BTN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_PIN), handleButton, FALLING);

  stage_led(2);
  my_disp.init(baud_rate, RX_ULCD, tx_pin);

  stage_led(3);
  BLEDevice::init("");
  BLEScan* scanner = BLEDevice::getScan();
  scanner->setAdvertisedDeviceCallbacks(new MyScanner());
  scanner->start(5, false);

  stage_led(4);
  Serial.println("System Ready...");
}

void loop() {
  if (toggle_req) {
    if (my_disp.is_on()) {
        my_disp.kill_screen();
    } else {
        my_disp.wake_up();
    }
    toggle_req = false;
  }

  if (connect_now) {
    if (doConnect()) Serial.println("Connected!");
    connect_now = false;
  }

  if (is_connected && new_packet) {
    noInterrupts();
    float p = data_in.pitch;
    float y = data_in.yaw;
    new_packet = false;
    interrupts();

    my_disp.refresh(p, y);

    static unsigned long log_t = 0;
    if (millis() - log_t > 500) {
      log_t = millis();
      Serial.print("Pitch: "); Serial.print(p);
      Serial.print(" | Yaw: "); Serial.println(y);
    }
  } else if (!is_connected) {
    if (millis() % 5000 < 20) {
        BLEDevice::getScan()->start(0);
    }
  }
  delay(1); 
}