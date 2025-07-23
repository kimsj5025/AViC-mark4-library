#include "AViC.h"

// --- Public Methods ---

/**
 * @brief AViC 보드의 핀들을 초기화 합니다.
 * 반드시 setup() 함수 안쪽에서 실행하세요
 */
void AViC::initialize(bool beep = false) {
  
    pinMode(IGNITE_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);

    digitalWrite(IGNITE_PIN, LOW); // Default to off
    digitalWrite(LED_PIN, HIGH);   // Default to on
    isLedOn = true;

    if(beep == true){
        playTone(NOTE_C, 200);
      playTone(NOTE_E, 200);
      playTone(NOTE_G, 200);
    }

    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.begin(115200); // UART0 
    HardwareSerial GPS_Serial(1); // UART1 객체 생성 (번호 1)
    GPS_Serial.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX); // UART1 
}

/**
 * @brief LED 상태를 바꿉니다.
 * 만약 LED가 켜져있다면 끄고, 껴져있다면 켭니다.
 */
void AViC::ledSW() {
    isLedOn = !isLedOn; // Toggle the state
    digitalWrite(LED_PIN, isLedOn ? HIGH : LOW);
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

void AViC::getAcceleData(){

};

void AViC::getMagnetData(){

};

void AViC::getBaroData(){

};

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
