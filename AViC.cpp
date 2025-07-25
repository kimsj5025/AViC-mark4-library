#include "AViC.h"
#include "ICM45686.h"
// --- Public Methods ---

#define ICM_ADDRESS_LSB 0
#define MS5607_ADDRESS 0x76  // MS5607 기압계 모듈의 I2C 주소
#define MS5607_CMD_PROM_READ_BASE  0xA0 // PROM 읽기 시작 주소
unsigned int C[8]; //ms5607용 보정계수
ICM456xx imu(Wire, ICM_ADDRESS_LSB); // 기본 Wire 객체를 IMU에 전달

float acceleXYZ[4];
float gyroXYZ[4];
double TEMP;
double P;


/**
 * @brief AViC 보드의 핀들을 초기화 합니다.
 * 반드시 setup() 함수 안쪽에서 실행하세요
 */

// MS5607의 PROM에서 C1~C6 보정 계수를 읽어오는 함수
bool readPROMCoefficients() {
  Serial.println("Reading MS5607 PROM Coefficients C1-C6...");
  // 데이터시트에 따라 PROM의 1번부터 6번 워드(C1~C6)를 읽는다.
  for (int i = 1; i <= 6; i++) {
    Wire.beginTransmission(MS5607_ADDRESS);
    Wire.write(MS5607_CMD_PROM_READ_BASE + (i * 2)); // 0xA2, 0xA4, ..., 0xAC
    if (Wire.endTransmission(false) != 0) {
      Serial.println("endTransmission failed");
      return false; // 통신 오류 시 false 반환
    }

    Wire.requestFrom(MS5607_ADDRESS, 2); // 2바이트(16비트) 데이터 요청
    if (Wire.available() >= 2) {
      C[i] = (unsigned int)Wire.read() << 8 | Wire.read(); // MSB, LSB 순으로 읽어 C[i]에 저장
    } else {
      Serial.println("requestFrom failed");
      return false; // 데이터 수신 실패 시 false 반환
    }
    Serial.print("C"); Serial.print(i); Serial.print(": "); Serial.println(C[i]);
  }
  return true;
}


void AViC::initialize(bool beep) {

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000); // I2C 속도를 400kHz로 설정 (센서들이 지원하는 일반적인 속도)


  if(beep == true){
    playTone(NOTE_C, 200);
    playTone(NOTE_E, 200);
    playTone(NOTE_G, 200);
  }
  
  // MS5607 PROM에서 보정 계수 읽기
  if (!readPROMCoefficients()) {
    Serial.println("Failed to read PROM coefficients from MS5607. Halting.");
    while (1);
  } else {
    Serial.println("Successfully read MS5607 PROM coefficients.");
  }

  // IMU 초기화
  if (imu.begin() != 0) {
    Serial.println("IMU initialization failed. Halting.");
    while (1);
  }

  // 가속도계 시작 (ODR=100Hz, FSR=16g)
  if (imu.startAccel(100, 16) != 0) {
    Serial.println("Failed to start accelerometer.");
  }

  // 자이로스코프 시작 (ODR=100Hz, FSR=2000dps)
  if (imu.startGyro(100, 2000) != 0) {
    Serial.println("Failed to start gyroscope.");
  }

  Serial.println("ICM45686 and MS5607 initialized. Starting measurements...");

  pinMode(IGNITE_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(IGNITE_PIN, LOW); // Default to off
  digitalWrite(LED_PIN, HIGH);   // Default to on
  isLedOn = true;
}

/**
 * @brief LED 상태를 바꿉니다.
 * 만약 LED가 켜져있다면 끄고, 껴져있다면 켭니다.
 * 
 * @param brightness - LED의 밝기를 조절합니다. (단위 : %)
 */
void AViC::ledSW(int brightness) {
    isLedOn = !isLedOn;
    //digitalWrite(LED_PIN, isLedOn ? HIGH : LOW);
    analogWrite(LED_PIN,isLedOn ? (brightness/100.0) * 256 : LOW);
}

/**
 * @brief Toggles the state of the LED.
 * If the LED is on, it turns it off. If it's off, it turns it on.
 * @param freq - 재생할 음의 주파수 입니다.
 * @param duration - 재생 시간 입니다.
 */
void AViC::playTone(int freq, int duration) {
  ledcSetup(BUZZER_CHANNEL, freq, BUZZER_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
  ledcWrite(BUZZER_CHANNEL, 128); // 50% duty
  delay(duration);
  ledcWrite(BUZZER_CHANNEL, 0);   // 소리 끔
  delay(50); // 음 사이 간격
}


/**
 * @brief   솔(G) 음의 바프음을 냅니다. (0.5초 사이클)
 * 
 * @param times - 반복 수
 */
void AViC::Waring(int times){
  for(int i = 0; i<times; i++){
    ledcSetup(BUZZER_CHANNEL, NOTE_G, BUZZER_RESOLUTION);
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
    ledcWrite(BUZZER_CHANNEL, 128); // 50% duty
    delay(50);
    ledcWrite(BUZZER_CHANNEL, 0);   // 소리 끔
    delay(450); // 음 사이 간격
  }
}

/**
 * @brief Pyro체널을 켭니다.
 * 
 * @param num - Pyro 체널의 번호
 */
void AViC::PyroON(int num){
  num -= 6;
  digitalWrite(num, HIGH);
}

/**
 * @brief Pyro체널을 끕니다.
 * 
 * @param num - Pyro 체널의 번호
 */
void AViC::PyroOff(int num){
  num -= 6;
  digitalWrite(num, LOW);
}


void AViC::readAcceleData(){
  
    inv_imu_sensor_data_t imu_data;
    imu.getDataFromRegisters(imu_data);

    acceleXYZ[0] = imu_data.accel_data[0];
    acceleXYZ[1] = imu_data.accel_data[1];
    acceleXYZ[2] = imu_data.accel_data[2];
    gyroXYZ[0] = imu_data.gyro_data[0]/16.4;
    gyroXYZ[1] = imu_data.gyro_data[1]/16.4;
    gyroXYZ[2] = imu_data.gyro_data[2]/16.4;
    acceleXYZ[3] = (imu_data.temp_data / 128.0) + 25;
    gyroXYZ[3] = (imu_data.temp_data / 128.0) + 25;

}


void AViC::readBaroData(){
  // 1. 압력 변환 요청 (D1)
  Wire.beginTransmission(MS5607_ADDRESS);
  Wire.write(0x48);  // OSR=4096 압력 변환 명령
  Wire.endTransmission();
  delay(10); // 데이터시트상 변환 시간은 약 9.04ms

  // 2. 압력 값 읽기
  Wire.beginTransmission(MS5607_ADDRESS);
  Wire.write(0x00);  // ADC 값 읽기 명령
  Wire.endTransmission(false);

  Wire.requestFrom(MS5607_ADDRESS, 3);
  uint32_t D1 = 0;
  if (Wire.available() >= 3) {
    D1 = (uint32_t)Wire.read() << 16 | (uint32_t)Wire.read() << 8 | (uint32_t)Wire.read();
  }

  // 3. 온도 변환 요청 (D2)
  Wire.beginTransmission(MS5607_ADDRESS);
  Wire.write(0x58);  // OSR=4096 온도 변환 명령
  Wire.endTransmission();
  delay(10); // 데이터시트상 변환 시간은 약 9.04ms

  // 4. 온도 값 읽기
  Wire.beginTransmission(MS5607_ADDRESS);
  Wire.write(0x00);  // ADC 값 읽기 명령
  Wire.endTransmission(false);

  Wire.requestFrom(MS5607_ADDRESS, 3);
  uint32_t D2 = 0;
  if (Wire.available() >= 3) {
    D2 = (uint32_t)Wire.read() << 16 | (uint32_t)Wire.read() << 8 | (uint32_t)Wire.read();
  }
  
  // 5. 1차 보정된 압력 및 온도 계산
  int32_t dT = D2 - (uint32_t)C[5] * 256;
  TEMP = 2000 + dT * C[6] / 8388608.0;

  int64_t OFF = (int64_t)C[2] * 131072 + (((int64_t)C[4] * dT) / 64.0);
  int64_t SENS = (int64_t)C[1] * 65536 + (((int64_t)C[3] * dT) / 128.0);
  
  // ★★★★★ 2차 온도 보정 (온도가 20°C 미만일 때 정확도 향상) ★★★★★
  int32_t T2 = 0;
  int64_t OFF2 = 0;
  int64_t SENS2 = 0;

  if (TEMP < 2000) {
    T2 = ((int64_t)dT * dT) / 2147483648; // dT^2 / 2^31
    OFF2 = 61 * (TEMP - 2000) *(TEMP - 2000)  / 16;
    SENS2 = 2 * ((TEMP - 2000) * (TEMP - 2000));
    
    if (TEMP < -1500) { // 온도가 -15°C 미만일 때 추가 보정
      OFF2 = OFF2 + 15 * ((TEMP + 1500) * (TEMP + 1500));
      SENS2 = SENS2 + 8 * ((TEMP + 1500) * (TEMP + 1500));
    }

    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;
  }
  // ★★★★★ 보정 완료 ★★★★★

  P = (((int64_t)D1 * SENS) / 2097152.0 - OFF) / 32768.0;
};

/**
 * @brief 가속도 데이터를 가져옵니다.
 * 
 * @param aixs 가속도 백터의 방향을 결정합니다
 * 0 : x
 * 1 : y
 * 2 : z
 * 3 : 온도
 */
float AViC::getAcceleData(int aixs){
  return (acceleXYZ[aixs]/2048.0)*9.81;
}

/**
 * @brief 자이로 데이터를 가져옵니다.
 * 
 * @param aixs 자이로 백터의 방향을 결정합니다
 * 0 : x
 * 1 : y
 * 2 : z
 * 3 : 온도
 */
float AViC::getGyroData(int aixs){
  return gyroXYZ[aixs];
}

/**
 * @brief 기압계의 데이터를 가져옵니다.
 * 
 * @param PorT 
 */
float AViC::getBaroData(int PorT){
  if(PorT == 1){
    return P/100.0;
  }else{
    return TEMP/100.0;
  }
}

float AViC::getMagnetData(int aixs){
  return 0.0;
}

void AViC::printSensorData(){
  Serial.println("가속도 X Y Z");
  Serial.println(getAcceleData(0));
  Serial.println(getAcceleData(1));
  Serial.println(getAcceleData(2));
  Serial.println();
  Serial.println("자이로 X Y Z");
  Serial.println(getGyroData(0));
  Serial.println(getGyroData(1));
  Serial.println(getGyroData(2));
  Serial.println();
  Serial.println("가속도계 온도");
  Serial.println(getGyroData(3));
  Serial.println();
  Serial.println("기압 , 온도");
  Serial.println(getBaroData(0));
  Serial.println(getBaroData(1));
  Serial.println("-------------");
}
/*
추가할 기능들

센서 데이터 읽기
GPS 데이터 읽기
SD카드 쓰기 / 읽기
자이로(오일러각) 계산하기
필터 사용하기
데이터 한번에 프린트
데이터 한번에 SD저장
*/
