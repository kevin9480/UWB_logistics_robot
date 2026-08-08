//최종 수신부
#include <DW1000Jang.hpp>
#include <DW1000JangUtils.hpp>
#include <DW1000JangTime.hpp>
#include <DW1000JangConstants.hpp>
#include <DW1000JangRanging.hpp>
#include <DW1000JangRTLS.hpp>
#include <Arduino.h>

#define VELOCITY 130 // 모터 속도 정의
#define SCALE 0.3    // 좌표 스케일 조정
#define THRESHOLD 0.1 // 좌표 오차 허용 범위

int LEFT_FRONT_PWM = 5; 
int LEFT_FRONT_DIR = 4;  

int RIGHT_FRONT_PWM = 6;  
int RIGHT_FRONT_DIR = A1;  

int RIGHT_BACK_PWM = 9;  
int RIGHT_BACK_DIR = 8;  

int LEFT_BACK_PWM = 3; 
int LEFT_BACK_DIR = A2; 

#define MAX_PATH_SIZE 100 // 최대 경로 크기
#define BUFFER_SIZE 512   // 버퍼 크기

int pathCoordinates[MAX_PATH_SIZE][2]; // 경로 좌표 배열
int pathSize = 0; // 경로 크기
char inputBuffer[BUFFER_SIZE]; // 입력 버퍼
int bufferIndex = 0; // 버퍼 인덱스 (데이터 일시적 저장, 조건 만족시 한 번에 처)

// millis함수 쓰기 위해서 설
unsigned long lastMotorUpdateTime = 0; // 모터가 마지막으로 업데이트된 시간
const unsigned long motorUpdateInterval = 50; // 모터 업데이트 간격 (밀리초)
unsigned long lastPositionUpdateTime = 0; // 위치가 마지막으로 업데이트된 시간
const unsigned long positionUpdateInterval = 10; // 위치 업데이트 간격 (밀리초)

#if defined(ESP8266)
const uint8_t PIN_SS = 15;
#else
const uint8_t PIN_SS = 10; // SPI 선택 핀
const uint8_t PIN_RST = 7; // 리셋 핀
#endif

double range_A;
double range_B;
double range_C;

typedef struct Position {
    double x;
    double y;
} Position;

Position position_A = {0, 0};
Position position_B = {2.5, 0};
Position position_C = {2.5, 2.5};

device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

frame_filtering_configuration_t TAG_FRAME_FILTER_CONFIG = {
    false,
    false,
    true,
    false,
    false,
    false,
    false,
    false
};

void setup() {
    Serial.begin(115200);
    pinMode(LEFT_FRONT_DIR, OUTPUT);
    pinMode(RIGHT_FRONT_DIR, OUTPUT);
    pinMode(RIGHT_BACK_DIR, OUTPUT);
    pinMode(LEFT_BACK_DIR, OUTPUT);

    pinMode(LEFT_FRONT_PWM, OUTPUT);
    pinMode(RIGHT_FRONT_PWM, OUTPUT);
    pinMode(RIGHT_BACK_PWM, OUTPUT);
    pinMode(LEFT_BACK_PWM, OUTPUT);

    while (!Serial) {
        ; 
    }
    Serial.println("Receiver ready");

    Serial.println(F("### DW1000Jang-arduino-ranging-tag ###"));
    #if defined(ESP8266)
    DW1000Jang::initializeNoInterrupt(PIN_SS);
    #else
    DW1000Jang::initializeNoInterrupt(PIN_SS, PIN_RST);
    #endif
    Serial.println("DW1000Jang initialized ...");

    DW1000Jang::applyConfiguration(DEFAULT_CONFIG);
    DW1000Jang::enableFrameFiltering(TAG_FRAME_FILTER_CONFIG);

    DW1000Jang::setDeviceAddress(4);
    DW1000Jang::setNetworkId(9);
    DW1000Jang::setAntennaDelay(16436);
    DW1000Jang::setPreambleDetectionTimeout(64);
    DW1000Jang::setSfdDetectionTimeout(273);
    DW1000Jang::setReceiveFrameWaitTimeoutPeriod(5000);

    Serial.println(F("Committed configuration ..."));
    char msg[128];
    DW1000Jang::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: ");
    Serial.println(msg);
    DW1000Jang::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: ");
    Serial.println(msg);
    DW1000Jang::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: ");
    Serial.println(msg);
    DW1000Jang::getPrintableDeviceMode(msg);
    Serial.print("Device mode: ");
    Serial.println(msg);
}

// 위치 계산 함수
void calculatePosition(double &x, double &y) {
    double A = ((-2 * position_A.x) + (2 * position_B.x));
    double B = ((-2 * position_A.y) + (2 * position_B.y));
    double C = ((range_A * range_A) - (range_B * range_B) - (position_A.x * position_A.x) + (position_B.x * position_B.x) - (position_A.y * position_A.y) + (position_B.y * position_B.y));
    double D = ((-2 * position_B.x) + (2 * position_C.x));
    double E = ((-2 * position_B.y) + (2 * position_C.y));
    double F = ((range_B * range_B) - (range_C * range_C) - (position_B.x * position_B.x) + (position_C.x * position_C.x) - (position_B.y * position_B.y) + (position_C.y * position_C.y));

    x = (C * E - F * B) / (E * A - B * D);
    y = (C * D - A * F) / (B * D - A * E);
}

void North() {
    analogWrite(LEFT_FRONT_PWM, VELOCITY);
    digitalWrite(LEFT_FRONT_DIR, HIGH);

    analogWrite(RIGHT_FRONT_PWM, VELOCITY);
    digitalWrite(RIGHT_FRONT_DIR, LOW);

    analogWrite(RIGHT_BACK_PWM, VELOCITY);
    digitalWrite(RIGHT_BACK_DIR, HIGH);

    analogWrite(LEFT_BACK_PWM, VELOCITY);
    digitalWrite(LEFT_BACK_DIR, LOW);
}

void South() {
    analogWrite(LEFT_FRONT_PWM, VELOCITY);
    digitalWrite(LEFT_FRONT_DIR, LOW);

    analogWrite(RIGHT_FRONT_PWM, VELOCITY);
    digitalWrite(RIGHT_FRONT_DIR, HIGH);

    analogWrite(RIGHT_BACK_PWM, VELOCITY);
    digitalWrite(RIGHT_BACK_DIR, LOW);

    analogWrite(LEFT_BACK_PWM, VELOCITY);
    digitalWrite(LEFT_BACK_DIR, HIGH);
}

void West() {
    analogWrite(LEFT_FRONT_PWM, VELOCITY);
    digitalWrite(LEFT_FRONT_DIR, LOW);

    analogWrite(RIGHT_FRONT_PWM, VELOCITY);
    digitalWrite(RIGHT_FRONT_DIR, LOW);

    analogWrite(RIGHT_BACK_PWM, VELOCITY);
    digitalWrite(RIGHT_BACK_DIR, LOW);

    analogWrite(LEFT_BACK_PWM, VELOCITY);
    digitalWrite(LEFT_BACK_DIR, LOW);
}

void East() {
    analogWrite(LEFT_FRONT_PWM, VELOCITY);
    digitalWrite(LEFT_FRONT_DIR, HIGH);

    analogWrite(RIGHT_FRONT_PWM, VELOCITY);
    digitalWrite(RIGHT_FRONT_DIR, HIGH);

    analogWrite(RIGHT_BACK_PWM, VELOCITY);
    digitalWrite(RIGHT_BACK_DIR, HIGH);

    analogWrite(LEFT_BACK_PWM, VELOCITY);
    digitalWrite(LEFT_BACK_DIR, HIGH);
}

void stopMotors() {
    analogWrite(LEFT_FRONT_PWM, 0);
    analogWrite(RIGHT_FRONT_PWM, 0);
    analogWrite(RIGHT_BACK_PWM, 0);
    analogWrite(LEFT_BACK_PWM, 0);
}

// 앵커들로부터 거리를 측정하여 위치를 업데이트 하는 함
void updatePosition(double &x, double &y) {
    New_structure result_A_Anchor = DW1000JangRTLS::Tag_Distance_Request(1, 1500);
    if (result_A_Anchor.success) {
        range_A = result_A_Anchor.distance;

        New_structure result_B_Anchor = DW1000JangRTLS::Tag_Distance_Request(2, 1500);
        if (result_B_Anchor.success) {
            range_B = result_B_Anchor.distance;

            New_structure result_C_Anchor = DW1000JangRTLS::Tag_Distance_Request(3, 1500);
            if (result_C_Anchor.success) {
                range_C = result_C_Anchor.distance;

                calculatePosition(x, y);

                Serial.println("------------------------------");
                Serial.print("x : ");
                Serial.println(x);

                Serial.print("y : ");
                Serial.println(y);
                Serial.println("------------------------------");
            }
        }
    }
}

// 목표 좌표로 이동하는 함수
void moveToCoordinates(double targetX, double targetY, double &x, double &y) {
    double deltaX = targetX - x;
    double deltaY = targetY - y;

    if (abs(deltaX) < abs(THRESHOLD) && abs(deltaY) < abs(THRESHOLD)) {
        stopMotors(); // 목표 위치에 도달하면 모터 정지
    } else {
        if (abs(deltaX) > abs(deltaY)) {
            if (deltaX > THRESHOLD) {
                East();
            } else if(deltaX < -THRESHOLD){
                West();
            }
        } else {
            if (deltaY > THRESHOLD) {
                North();
            } else if(deltaY < -THRESHOLD){
                South();
            }
        }
    }
} // UWB의 10cm의 오차가 있어서 감안하기 위해 threshold 설정 

void loop() {
    static int currentTargetIndex = 0;
    static double x = 0.0, y = 0.0;
    static unsigned long lastMotorUpdateTime = 0;
    static unsigned long lastPositionUpdateTime = 0;

    while (Serial.available() > 0) {
        char receivedChar = Serial.read();
        if (receivedChar == '\n' || bufferIndex >= sizeof(inputBuffer) - 1) {
            inputBuffer[bufferIndex] = '\0';
            bufferIndex = 0;

            if (strncmp(inputBuffer, "Path: ", 6) == 0) {
                pathSize = 0;
                char *token = strtok(inputBuffer + 6, ";");
                while (token != NULL && pathSize < MAX_PATH_SIZE) {
                    sscanf(token, "%d,%d", &pathCoordinates[pathSize][0], &pathCoordinates[pathSize][1]);
                    pathSize++;
                    token = strtok(NULL, ";");
                }
                Serial.println("Path coordinates received:");
                for (int i = 0; i < pathSize; i++) {
                    Serial.print("(");
                    Serial.print(pathCoordinates[i][0] * SCALE, 1);
                    Serial.print(", ");
                    Serial.print(pathCoordinates[i][1] * SCALE, 1);
                    Serial.println(")");
                }

                // 첫 번째 목표로 초기화
                currentTargetIndex = 0;
            }
        } else {
            inputBuffer[bufferIndex++] = receivedChar;
        }
    }

    if (millis() - lastMotorUpdateTime >= motorUpdateInterval) {
        lastMotorUpdateTime = millis();

        if (currentTargetIndex < pathSize) {
            double targetX = pathCoordinates[currentTargetIndex][0] * SCALE;
            double targetY = pathCoordinates[currentTargetIndex][1] * SCALE;
            moveToCoordinates(targetX, targetY, x, y);

            // 목표에 도달했는지 확인
            if (abs(targetX - x) < abs(THRESHOLD) && abs(targetY - y) < abs(THRESHOLD)) {
                currentTargetIndex++;
            }
        } else {
            stopMotors(); // 목표에 도달하면 정
        }
    }

    // 위치 업데이
    if (millis() - lastPositionUpdateTime >= positionUpdateInterval) {
        lastPositionUpdateTime = millis();
        updatePosition(x, y);
    }
}
