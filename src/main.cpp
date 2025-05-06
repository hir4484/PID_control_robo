///////// ESP32-C3 SuperMINI inverted Pendulum ////////
/////////////////// PID Control Robo //////////////////
////////////////// 2025/05/03 by hir. /////////////////

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <Wire.h>

// LED_PIN
#define LED1_PIN 1

//MPU6050 I2C PIN
#define SDA 8 // on board LED
#define SCL 9 // GPIO9を0にして電源ONにするとDownloadBootモードで起動

////////// MPU6050 処理関連 //////////
#define MPU6050_ADDR         0x68
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_GYRO_LSB     65.5
#define MPU6050_ACCEL_LSB    8192.0

// Motor PWM is attached to PIN 16 and 17
#define Phase_A   5
#define Phase_B   6
#define driv_stby 7
// battery ADC
#define Battery_PIN 0

////////// mpu6050 計算用係数 //////////
float y_rad, y_last;
double offsetX = 0, offsetY = 0, offsetZ = 0;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float angleX, angleY, angleZ;
float interval, preInterval;
float acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
float gx, gy, gz, dpsX, dpsY, dpsZ;
bool dmpReady = false;

// movement status管理 => 0:kick start, 1:balance,.., 9:stop,
int status = 1; 
bool do_balance = false;

////////// 倒立係数 //////////
float score = 0, rad=0, delta=0, length=0, base_line = 0;
int MT_min = 50;  // 50 倒立制御時のﾓｰﾀ下限値
int MT_max = 255;

float radK =      8.10;    // 15.10 7.60 8.20
float deltaK =    0.11;    // 0.14 0.19
float lengthK =  56.20;    // 84.10 71.0 56.20

float adjustWUp  =  90.0;  // 80 20
float adjustZero =  0.40;  // 0.23
float adjustBase = 120.0;  // 150

float y_base = 0, y_base_orig, 
      y_base_delta = 34.4, //38.20, // 40.0, 
      y_base_kick  = 24.0; //29.20; // 41.0;
uint8_t pwm_duty = 0;      // 8bit max 255, 16bit max 65535

// 1Cell Lipo battery voltage
float batt_v = 0;

////////// BLE 処理関連 //////////
#define DEVICENAME "ESP32C3_name"

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
boolean isrequested = false;
bool led_bool = false;

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    //Serial.println("** device connected");
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    //Serial.println("** device disconnected");
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0) {
      // //Display on serial monitor for debug
      // Serial.println("*********");
      // Serial.print("Received Value: ");
      // //rxValue.trim();
      // Serial.println(rxValue.c_str());
      // Serial.println("*********");
      // //Reply as is
      pTxCharacteristic->setValue(rxValue.c_str());
      pTxCharacteristic->notify();
      delay(10);

      ////////////////////////////////////////////////////////////
      ////////////// Analyzing the content received //////////////
      if (rxValue.find("start") == 0) {       // BLE start
        isrequested = true;
      } else if (rxValue.find("quit") == 0) { // BLE finished
        isrequested = false;
      } else if (rxValue.find("move") == 0) { // kick start moving
        do_balance = true;
        status = 0;
        y_base_orig = y_rad;
        //y_base = y_rad + y_base_kick;
      } else if (rxValue.find("stop") == 0) { // stop moving
        y_base = 90.0;   // Forced backward tilt angle (着座角度 68 deg)
        status = 9;                           // motor stop
      }
    }
  }
};

////////// timer intrrupt 関連 //////////
hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
u_int32_t isrCounter = 0; // timer内処理完結なので vola要りません
volatile bool first_timer = false, second_timer = false, third_timer = false;
bool timer_enable = false; // mpu6050 の初期化待ち flag

////////// Timer 実行処理 //////////
void ARDUINO_ISR_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);

  // Depending on the count, you can safely set a flag here.
  if (isrCounter %    5 == 0){ // 5msc timer
    first_timer = true;
  }
  if (isrCounter %  200 == 0){ // 200msc timer
    second_timer = true;
  }
  if (isrCounter % 1000 == 0){ // 1000msc timer
    third_timer = true;
  }
}

////////// LED 処理 //////////
void led_blink(u8_t pin){
  digitalWrite(pin, !digitalRead(pin));
}

////////// I2C 処理 //////////
// i2c write
void writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// i2C read
byte readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(true);
  Wire.requestFrom(MPU6050_ADDR, 1/*length*/); 
  byte data =  Wire.read();
  return data;
}

////////// MPU6050 初期設定 //////////
void setup_mpu6050(){
  Wire.begin(SDA, SCL);
  Wire.setClock(400000);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(50);

  ///// set Register
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x01); // 1kHz/(1+(1))
  writeMPU6050(MPU6050_CONFIG,     0x01); // DLPF No1
  uint8_t data = readMPU6050(MPU6050_GYRO_CONFIG);
  data = ( data & 0b11100000 );
  data = ( data | 0b00001000 );           // 500deg/s (5ms loop => 2.5deg/loop)
  writeMPU6050(MPU6050_GYRO_CONFIG, data);
  data = readMPU6050(MPU6050_ACCEL_CONFIG);
  data = ( data & 0b11100000 );
  data = ( data | 0b00001000 );           // 4g
  writeMPU6050(MPU6050_ACCEL_CONFIG, data);
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x00);

  ///// set Calibration
  //Serial.print("Calculate Calibration");
  for(int i = 0; i < 3000; i++){
    int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z ;
    
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14);
  
    raw_acc_x = Wire.read() << 8 | Wire.read();
    raw_acc_y = Wire.read() << 8 | Wire.read();
    raw_acc_z = Wire.read() << 8 | Wire.read();
    raw_t = Wire.read() << 8 | Wire.read();
    raw_gyro_x = Wire.read() << 8 | Wire.read();
    raw_gyro_y = Wire.read() << 8 | Wire.read();
    raw_gyro_z = Wire.read() << 8 | Wire.read();
    dpsX = ((float)raw_gyro_x) / MPU6050_GYRO_LSB;
    dpsY = ((float)raw_gyro_y) / MPU6050_GYRO_LSB;
    dpsZ = ((float)raw_gyro_z) / MPU6050_GYRO_LSB;
    offsetX += dpsX;
    offsetY += dpsY;
    offsetZ += dpsZ;
    delay(1);
 
  }
  offsetX /= 3000;
  offsetY /= 3000;
  offsetZ /= 3000;
  
  dmpReady = true;
}

////////// MPU6050 data読み込み, 相補ﾌｨﾙﾀｰ //////////
void calcRotation(){
  int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z ;
  
  // Register Address 0x3Bから、14byte data 出力
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14);
  
  // Read data and bit shift operations
  raw_acc_x = Wire.read() << 8 | Wire.read();
  raw_acc_y = Wire.read() << 8 | Wire.read();
  raw_acc_z = Wire.read() << 8 | Wire.read();
  raw_t = Wire.read() << 8 | Wire.read();
  raw_gyro_x = Wire.read() << 8 | Wire.read();
  raw_gyro_y = Wire.read() << 8 | Wire.read();
  raw_gyro_z = Wire.read() << 8 | Wire.read();
  
  // Convert units to [g]
  acc_x = ((float)raw_acc_x) / MPU6050_ACCEL_LSB;
  acc_y = ((float)raw_acc_y) / MPU6050_ACCEL_LSB;
  acc_z = ((float)raw_acc_z) / MPU6050_ACCEL_LSB;
  
  // Calculate angle from acceleration sensor
  acc_angle_y = atan2(acc_x, acc_z + abs(acc_y)) * 360 / -2.0 / PI;
  acc_angle_x = atan2(acc_y, acc_z + abs(acc_x)) * 360 /  2.0 / PI;

  // Convert units to [d/s]
  dpsX = ((float)raw_gyro_x) / MPU6050_GYRO_LSB;
  dpsY = ((float)raw_gyro_y) / MPU6050_GYRO_LSB;
  dpsZ = ((float)raw_gyro_z) / MPU6050_GYRO_LSB;
  
  // Calculate the elapsed time
  interval = micros() - preInterval; // millis => micros
  preInterval = micros();
  
  // Numerical Integration
  gyro_angle_x += (dpsX - offsetX) * (interval * 0.000001);
  gyro_angle_y += (dpsY - offsetY) * (interval * 0.000001);
  gyro_angle_z += (dpsZ - offsetZ) * (interval * 0.000001);
  
  ////////////////////////////////////////////////////////
  
  ///// Complementary Filter, original k = 0.996, 0.004
  angleX = (0.992 * gyro_angle_x) + (0.008 * acc_angle_x);
  angleY = (0.992 * gyro_angle_y) + (0.008 * acc_angle_y);
  angleZ = gyro_angle_z;

  gyro_angle_x = angleX;
  gyro_angle_y = angleY;
  gyro_angle_z = angleZ;

  // Update y-axis angle
  y_last = y_rad;
  y_rad = angleY;
}

///// forward rotation /////
void forward( int pwm ){
  ledcWrite( 1, pwm );
  ledcWrite( 2, 0   );
}

///// forward rotation /////
void backward( int pwm ){
  ledcWrite( 1, 0   );
  ledcWrite( 2, pwm );
}

///// y-axis PID calcuration /////
void pid_calc_set(){  
  rad = y_rad - y_base;
  delta = (y_rad - y_last) / 0.005;

  // 偏差の時間積分を全て積分し、初期位置からのﾄﾞﾘﾌﾄ量を予測
  base_line += length;
  // 前後ﾄﾞﾘﾌﾄ移動に対し、次回の目標角度変更(ﾍﾞｰｽ角)で処理
  if (base_line > adjustBase){
    y_base -= adjustZero;
    base_line = 0;
  } else if (base_line < -1*adjustBase){
    y_base += adjustZero;
    base_line = 0;
  }

  ///// Anti-windup
  if ( abs(length) > adjustWUp ) {
    length = 0;
  } else {
    length += rad * 0.005;
  }

  score = rad*radK + delta*deltaK + length*lengthK;
  pwm_duty = map ( constrain( abs( score ), 0, 210 ), 0, 210, MT_min, MT_max );
  // score は 0-210 でｸﾘｯﾌﾟ
  // 0-210 の範囲を min-max に比例分配

  if (pwm_duty < MT_min ){
    pwm_duty = MT_min;
  } else if (pwm_duty > MT_max) {
    pwm_duty = MT_max;
  }

  if ( score < 0 ) {
    forward( pwm_duty );
  } else if ( score > 0) {
    backward( pwm_duty );
  } else {
    forward( 0 );
  }

}

///// Lipo Battery Voltage check /////
float batt_volt(){
  ///// check the battery Voltage!
  int ad_temp = 0;
  for ( int i=0; i<3; i++ ){
    ad_temp += analogRead(Battery_PIN);
  }
  // battery電圧は 1/5 に分圧
  // 電圧変換は、実測値から線形近似式
  float ad_volt = 0.0013866*((float)ad_temp)/3 + 0.087105; 
  return ad_volt;
}

void setup() {
  // LED pin setting
  pinMode(LED1_PIN, OUTPUT);

  // MPU 6050 connect & initialize
  setup_mpu6050();

  // Create the BLE Device
  BLEDevice::init(DEVICENAME); //BLE Device Name scaned and found by clients

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY );
                    
  // pTxCharacteristic に BLE2902 を追加。BLE Client の Notify (Indicate) を有効化
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_RX,
                    BLECharacteristic::PROPERTY_WRITE );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  //Serial.println("start advertising");
  //Serial.println("Waiting a client connection to notify...");

  //The ESP32S3 bluetoth 5.0 requires security settings.
  //Without it, an error will occur when trying to pair with other devices.
  //Using a 6-digit PIN as the authentication method seems to work.
  //This PIN allows the device to be paired with an Client device.
  //Client device users will be prompted to key in a 6-digit PIN, '123456'.
  BLESecurity *pSecurity = new BLESecurity();
  //pSecurity->setStaticPIN(123456);
  //Setting ESP_LE_AUTH_REQ_SC_ONLY instead of the PIN setting eliminates the need for PIN input during pairing.
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_ONLY);

  ////////// timer count setting //////////
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();
  timer = timerBegin(0, 80, true);    // prescaler (1usec, increment => 80)
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true); // 1000usec timer
  delay(50);
  //timerAlarmEnable(timer);            // timer start!! ==> mpu の初期化後に起動

  ////////// PWM setting //////////
  pinMode(Phase_A, OUTPUT);
  pinMode(Phase_B, OUTPUT);
  pinMode(driv_stby, OUTPUT);

  ledcSetup(1, 1000, 8);      //ch_0、1kHz、8bit_range
  ledcAttachPin(Phase_A, 1);
  ledcSetup(2, 1000, 8);      //ch_1、1kHz、8bit_range
  ledcAttachPin(Phase_B, 2);
  digitalWrite(driv_stby, HIGH);

  // battery monitor = 1/5 value, 4.2V/5 = 0.84V (-2.5db => max 1.05V)
  // ADC1 = GPIO 0-4
  pinMode(Battery_PIN, ANALOG);
  analogSetAttenuation(ADC_2_5db);  //ATT -2.5dB
}

void loop() {
  // If Timer has fired, do the ballance moving!
  if(timer_enable == false){
    if(dmpReady == true){
      // After the MPU6050 starts, start the timer.
      timerAlarmEnable(timer);
      timer_enable = true;
    }
  }

  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    portEXIT_CRITICAL(&timerMux);
  }

  ////////////////////////////////////////////////////
  if ( first_timer == true ){
      ///// Angle calculation is always performed /////
      calcRotation();  // y軸角度の更新 相補ﾌｨﾙﾀｰ
      ///// Switch operation mode according to the status value /////
      switch( status ){
        case 0: // kick start mode
        int static kick_cnt = 0;
        if ( kick_cnt < 12 ){                 // 5ms x 12 = 60ms
          y_base = y_base_orig + y_base_kick; // kick start angle
          pid_calc_set(); // Motor rotation
          kick_cnt ++;

        } else {
          status = 1;
          kick_cnt = 0;
          y_base = y_base_orig + y_base_delta; // balance angle
        }
        break;

        case 1: // Normal balance mode
        if ( do_balance == true ){
          if ( abs(rad) < 60 ){ // 60
            pid_calc_set(); // Motor rotation

          } else {
            do_balance = false; // 60°より傾いた場合は強制停止
            forward( 0 );
          }
          
        }
        break;

        case 9: // stop mode
        int static stop_cnt = 0;
        if ( stop_cnt < 20 ){ // 5ms x 20 = 100ms Tilt backwards!
          pid_calc_set();
          stop_cnt ++;
        } else {
          do_balance = false; // Stopped working,
          forward( 0 );       // Motor stopped
          status = 1;
          stop_cnt = 0;
        }
        break;
      }
    
    first_timer = false;
  }
  ////////////////////////////////////////////////////
  if ( second_timer == true ){
    batt_v  = batt_volt();

    if (deviceConnected) {
      if (isrequested) {
        char strings[64]; // up to 256
        // Write the transmission strings
        sprintf(strings, "Ydeg= %.2f, base= %.2f, V= %.2f\r\n", angleY, y_base, batt_v);
        pTxCharacteristic->setValue(strings);
        pTxCharacteristic->notify();
        //pTxCharacteristic->indicate();
      }
    } else {
      isrequested = false;
    }

    second_timer = false;
  }
  ////////////////////////////////////////////////////
  if ( third_timer == true ){
    led_blink(LED1_PIN);
    third_timer = false;
  }
}