// =====================================================================
// 【ESP32 + L298N 五路黑線自走車】防晃＋加強轉彎版 v3
// 修正：上一版變得不太會轉彎
//
// 目標：
// 1. 直線不要瘋狂左右晃
// 2. L2 / R2 外側感測器一偵測到黑線，就進入鎖定轉彎
// 3. 轉彎不是原地轉，而是內輪慢速前進、外輪大力前進
// 4. 轉到中間 M 重新偵測到黑線後離開轉彎
// =====================================================================

#include <Arduino.h>

// ---------------- 啟動按鈕與 RGB LED ----------------
const int VLS_PIN = 4;

const int LED_R = 5;
const int LED_G = 15;
const int LED_B = 2;

// ---------------- 五路感測器腳位：左到右 ----------------
const int S_L2 = 25;
const int S_L1 = 26;
const int S_M  = 27;
const int S_R1 = 14;
const int S_R2 = 12;

// 黑線 = 1，白底 = 0
const int BLACK = 1;
const int WHITE = 0;

// ---------------- L298N 馬達控制腳位 ----------------
// 這組是你們目前確認可正常直走、左右判斷正確的腳位
const int M_L1 = 19;
const int M_L2 = 18;

const int M_R1 = 17;
const int M_R2 = 16;

bool INVERT_LEFT  = false;
bool INVERT_RIGHT = false;

// ---------------- PWM 設定 ----------------
const int PWM_FREQ = 20000;
const int PWM_BITS = 8;
const int PWM_MAX  = 255;

const int CH_L1 = 0;
const int CH_L2 = 1;
const int CH_R1 = 2;
const int CH_R2 = 3;

// =====================================================================
// 速度設定
// =====================================================================

// 直線速度
int BASE_SPEED     = 140;
int STRAIGHT_SPEED = 150;

// 偏差大時降速
int CORNER_SPEED = 120;

// 最大速度
int MAX_SPEED = 185;

// L298N 低 PWM 容易推不動
int MIN_PWM = 90;

// PD 修正最大值：越小越不晃，但太小會轉不動
int MAX_CORRECTION = 55;

// ---------------- 轉彎速度 ----------------
// 不原地轉：內輪慢速前進、外輪大力前進
int TURN_OUTER_SPEED = 215;
int TURN_INNER_SPEED = 75;

// 掉線找線
int RECOVER_OUTER_SPEED = 165;
int RECOVER_INNER_SPEED = 85;

// ---------------- 轉彎控制 ----------------
// 轉彎至少持續時間，避免剛進彎就誤判 M 黑線
const int TURN_MIN_TIME = 170;

// 若 M 一直沒有重新看到黑線，最多轉這麼久
const int TURN_MAX_TIME = 1400;

// 出彎後冷卻時間：縮短，避免下一個彎抓不到
const int TURN_COOLDOWN_TIME = 160;

// 出彎後往前推一小段
const int AFTER_TURN_FORWARD_TIME = 70;

// 全白時先維持上一拍
const int LOST_HOLD_TIME = 60;

// ---------------- PD 參數 ----------------
// 不用 Ki，避免累積誤差造成後段越晃越大
float Kp = 0.030;
float Kd = 0.080;

// 誤差死區：小偏差不修正，減少蛇行
const int ERROR_DEADBAND = 250;

// ---------------- 狀態記錄 ----------------
bool isRunning = false;
bool DEBUG_PRINT = false;

int weight[5] = {-2000, -1000, 0, 1000, 2000};

int lastError = 0;
float filteredError = 0;

int lastDirection = 0;
// -1 = 最後線在左邊
//  1 = 最後線在右邊
//  0 = 中間

int lastLeftPWM = 0;
int lastRightPWM = 0;

unsigned long lastSeenTime = 0;
unsigned long lastPrintTime = 0;
unsigned long turnCooldownUntil = 0;

// =====================================================================
// 函式宣告
// =====================================================================

void setupPWMOne(int pin, int channel);
void writePWMOne(int pin, int channel, int duty);

int stableReadPin(int pin);
void readSensors(int s[5]);
int getLinePosition(int s[5], int &activeCount);

void followLine();
void lockedTurnLeft();
void lockedTurnRight();
void recoverLine();

void setMotor(int left, int right);
int applyMinPWM(int pwm);

void setASL(int r, int g, int b);
void countdownStart();
void printSensor(int s[5], const char* action, int leftPWM, int rightPWM);

// =====================================================================
// ESP32 PWM 相容處理
// =====================================================================

void setupPWMOne(int pin, int channel) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(channel, PWM_FREQ, PWM_BITS);
  ledcAttachPin(pin, channel);
#endif
}

void writePWMOne(int pin, int channel, int duty) {
  duty = constrain(duty, 0, PWM_MAX);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

// =====================================================================
// setup
// =====================================================================

void setup() {
  Serial.begin(115200);

  pinMode(VLS_PIN, INPUT_PULLUP);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  pinMode(S_L2, INPUT);
  pinMode(S_L1, INPUT);
  pinMode(S_M,  INPUT);
  pinMode(S_R1, INPUT);
  pinMode(S_R2, INPUT);

  setupPWMOne(M_L1, CH_L1);
  setupPWMOne(M_L2, CH_L2);
  setupPWMOne(M_R1, CH_R1);
  setupPWMOne(M_R2, CH_R2);

  setMotor(0, 0);

  setASL(LOW, HIGH, LOW);

  Serial.println("ESP32 + L298N 五路黑線車：防晃＋加強轉彎版 v3");
}

// =====================================================================
// loop
// =====================================================================

void loop() {
  if (!isRunning) {
    if (digitalRead(VLS_PIN) == LOW) {
      delay(50);

      if (digitalRead(VLS_PIN) == LOW) {
        countdownStart();

        lastError = 0;
        filteredError = 0;
        lastDirection = 0;
        lastSeenTime = millis();
        turnCooldownUntil = 0;

        isRunning = true;
        setASL(HIGH, LOW, LOW);
      }
    }

    return;
  }

  followLine();
}

// =====================================================================
// 主循跡邏輯
// =====================================================================

void followLine() {
  int s[5];
  readSensors(s);

  int activeCount = 0;
  int error = getLinePosition(s, activeCount);

  if (activeCount == 0) {
    recoverLine();
    return;
  }

  lastSeenTime = millis();

  bool inCooldown = millis() < turnCooldownUntil;

  int l2 = s[0];
  int l1 = s[1];
  int m  = s[2];
  int r1 = s[3];
  int r2 = s[4];

  // ---------------------------------------------------------
  // 1. 加強轉彎觸發
  // 外側 L2 / R2 優先判斷，避免上一版太保守導致不轉彎
  // ---------------------------------------------------------

  bool leftTurnCandidate =
    !inCooldown &&
    (
      // 左外側碰到黑線，通常就是左彎入口
      (l2 == BLACK && r2 == WHITE) ||

      // 左側 L1 明顯黑、右側都白，也視為左彎
      (l1 == BLACK && m == WHITE && r1 == WHITE && r2 == WHITE) ||

      // L2 + L1 同時黑，也視為左彎
      (l2 == BLACK && l1 == BLACK)
    );

  bool rightTurnCandidate =
    !inCooldown &&
    (
      // 右外側碰到黑線，通常就是右彎入口
      (r2 == BLACK && l2 == WHITE) ||

      // 右側 R1 明顯黑、左側都白，也視為右彎
      (r1 == BLACK && m == WHITE && l1 == WHITE && l2 == WHITE) ||

      // R2 + R1 同時黑，也視為右彎
      (r2 == BLACK && r1 == BLACK)
    );

  // 左右同時觸發時，不進入強轉，避免寬黑線或交會點誤判
  if (leftTurnCandidate && !rightTurnCandidate) {
    lastDirection = -1;
    printSensor(s, "鎖定左轉", 0, 0);
    lockedTurnLeft();
    return;
  }

  if (rightTurnCandidate && !leftTurnCandidate) {
    lastDirection = 1;
    printSensor(s, "鎖定右轉", 0, 0);
    lockedTurnRight();
    return;
  }

  // ---------------------------------------------------------
  // 2. PD 線循跡
  // ---------------------------------------------------------

  // 低通濾波，避免誤差瞬間跳太大
  filteredError = filteredError * 0.65 + error * 0.35;

  int useError = (int)filteredError;

  // deadband：接近中間就不修正，減少左右晃
  if (abs(useError) < ERROR_DEADBAND) {
    useError = 0;
  }

  int derivative = useError - lastError;

  int correction = (int)(Kp * useError + Kd * derivative);
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  int base = BASE_SPEED;

  // 線很置中才加速
  if (useError == 0 && m == BLACK) {
    base = STRAIGHT_SPEED;
  }

  // 偏差大就降速
  if (abs(useError) > 1000) {
    base = CORNER_SPEED;
  }

  int leftPWM  = base + correction;
  int rightPWM = base - correction;

  leftPWM  = constrain(leftPWM,  MIN_PWM, MAX_SPEED);
  rightPWM = constrain(rightPWM, MIN_PWM, MAX_SPEED);

  setMotor(leftPWM, rightPWM);

  if (useError < 0) {
    lastDirection = -1;
  }
  else if (useError > 0) {
    lastDirection = 1;
  }
  else {
    lastDirection = 0;
  }

  lastError = useError;

  printSensor(s, "PD循線", leftPWM, rightPWM);
}

// =====================================================================
// 左轉鎖定：左內輪慢、右外輪快，直到 M 重新看到黑線
// =====================================================================

void lockedTurnLeft() {
  unsigned long startTime = millis();
  bool centerWasWhite = false;

  while (millis() - startTime < TURN_MAX_TIME) {
    int s[5];
    readSensors(s);

    // 左轉：左側慢速前進，右側快速前進
    setMotor(TURN_INNER_SPEED, TURN_OUTER_SPEED);

    unsigned long elapsed = millis() - startTime;

    if (elapsed > 50 && s[2] == WHITE) {
      centerWasWhite = true;
    }

    // 避免一開始 M 還在舊線上就退出
    if (elapsed > TURN_MIN_TIME && s[2] == BLACK && (centerWasWhite || elapsed > 360)) {
      break;
    }

    delay(5);
  }

  setMotor(0, 0);
  delay(20);

  setMotor(BASE_SPEED, BASE_SPEED);
  delay(AFTER_TURN_FORWARD_TIME);

  lastError = 0;
  filteredError = 0;
  lastDirection = 0;

  turnCooldownUntil = millis() + TURN_COOLDOWN_TIME;
}

// =====================================================================
// 右轉鎖定：左外輪快、右內輪慢，直到 M 重新看到黑線
// =====================================================================

void lockedTurnRight() {
  unsigned long startTime = millis();
  bool centerWasWhite = false;

  while (millis() - startTime < TURN_MAX_TIME) {
    int s[5];
    readSensors(s);

    // 右轉：左側快速前進，右側慢速前進
    setMotor(TURN_OUTER_SPEED, TURN_INNER_SPEED);

    unsigned long elapsed = millis() - startTime;

    if (elapsed > 50 && s[2] == WHITE) {
      centerWasWhite = true;
    }

    // 避免一開始 M 還在舊線上就退出
    if (elapsed > TURN_MIN_TIME && s[2] == BLACK && (centerWasWhite || elapsed > 360)) {
      break;
    }

    delay(5);
  }

  setMotor(0, 0);
  delay(20);

  setMotor(BASE_SPEED, BASE_SPEED);
  delay(AFTER_TURN_FORWARD_TIME);

  lastError = 0;
  filteredError = 0;
  lastDirection = 0;

  turnCooldownUntil = millis() + TURN_COOLDOWN_TIME;
}

// =====================================================================
// 掉線找線
// =====================================================================

void recoverLine() {
  unsigned long lostTime = millis() - lastSeenTime;

  if (lostTime < LOST_HOLD_TIME) {
    setMotor(lastLeftPWM, lastRightPWM);
    return;
  }

  if (lastDirection < 0) {
    setMotor(RECOVER_INNER_SPEED, RECOVER_OUTER_SPEED);
  }
  else if (lastDirection > 0) {
    setMotor(RECOVER_OUTER_SPEED, RECOVER_INNER_SPEED);
  }
  else {
    setMotor(120, 120);
  }
}

// =====================================================================
// 感測器讀取：3 次多數決
// =====================================================================

int stableReadPin(int pin) {
  int sum = 0;

  for (int i = 0; i < 3; i++) {
    sum += digitalRead(pin);
    delayMicroseconds(250);
  }

  return (sum >= 2) ? 1 : 0;
}

void readSensors(int s[5]) {
  s[0] = stableReadPin(S_L2);
  s[1] = stableReadPin(S_L1);
  s[2] = stableReadPin(S_M);
  s[3] = stableReadPin(S_R1);
  s[4] = stableReadPin(S_R2);
}

// =====================================================================
// 計算線的位置
// 回傳：-2000 ~ 2000
// 負值代表線在左，正值代表線在右
// =====================================================================

int getLinePosition(int s[5], int &activeCount) {
  long weightedSum = 0;
  activeCount = 0;

  for (int i = 0; i < 5; i++) {
    if (s[i] == BLACK) {
      activeCount++;
      weightedSum += weight[i];
    }
  }

  if (activeCount == 0) {
    return lastError;
  }

  return weightedSum / activeCount;
}

// =====================================================================
// 馬達控制
// left / right：正值前進，負值後退
// =====================================================================

void setMotor(int left, int right) {
  if (INVERT_LEFT) {
    left = -left;
  }

  if (INVERT_RIGHT) {
    right = -right;
  }

  left  = constrain(left,  -PWM_MAX, PWM_MAX);
  right = constrain(right, -PWM_MAX, PWM_MAX);

  left  = applyMinPWM(left);
  right = applyMinPWM(right);

  lastLeftPWM = left;
  lastRightPWM = right;

  // 左側馬達
  if (left > 0) {
    writePWMOne(M_L1, CH_L1, left);
    writePWMOne(M_L2, CH_L2, 0);
  }
  else if (left < 0) {
    writePWMOne(M_L1, CH_L1, 0);
    writePWMOne(M_L2, CH_L2, -left);
  }
  else {
    writePWMOne(M_L1, CH_L1, 0);
    writePWMOne(M_L2, CH_L2, 0);
  }

  // 右側馬達
  if (right > 0) {
    writePWMOne(M_R1, CH_R1, right);
    writePWMOne(M_R2, CH_R2, 0);
  }
  else if (right < 0) {
    writePWMOne(M_R1, CH_R1, 0);
    writePWMOne(M_R2, CH_R2, -right);
  }
  else {
    writePWMOne(M_R1, CH_R1, 0);
    writePWMOne(M_R2, CH_R2, 0);
  }
}

int applyMinPWM(int pwm) {
  if (pwm == 0) {
    return 0;
  }

  if (pwm > 0 && pwm < MIN_PWM) {
    return MIN_PWM;
  }

  if (pwm < 0 && pwm > -MIN_PWM) {
    return -MIN_PWM;
  }

  return pwm;
}

// =====================================================================
// LED 與倒數
// =====================================================================

void setASL(int r, int g, int b) {
  digitalWrite(LED_R, r);
  digitalWrite(LED_G, g);
  digitalWrite(LED_B, b);
}

void countdownStart() {
  for (int i = 0; i < 3; i++) {
    setASL(LOW, LOW, HIGH);
    delay(200);

    setASL(LOW, LOW, LOW);
    delay(200);
  }
}

// =====================================================================
// Serial 輸出
// =====================================================================

void printSensor(int s[5], const char* action, int leftPWM, int rightPWM) {
  if (!DEBUG_PRINT) {
    return;
  }

  if (millis() - lastPrintTime > 150) {
    lastPrintTime = millis();

    Serial.print("S=");
    Serial.print(s[0]);
    Serial.print(s[1]);
    Serial.print(s[2]);
    Serial.print(s[3]);
    Serial.print(s[4]);

    Serial.print(" ");
    Serial.print(action);

    Serial.print(" L=");
    Serial.print(leftPWM);

    Serial.print(" R=");
    Serial.println(rightPWM);
  }
}
