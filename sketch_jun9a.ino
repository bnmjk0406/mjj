// =====================================================================
// 【ESP32 + L298N 五路黑線自走車】最終統整版
// 適用：48 mm 黑線、L298N、一顆按鈕起跑/急停、ASL 指示燈
//
// 功能：
// 1. 按鈕 VLS：按一下起跑，倒數後進入自主導航
// 2. 按鈕 EBS：行駛中再按一下，立即停止
// 3. ASL：
//    綠燈 = 安全狀態 / 停止
//    紅燈 = 自主導航 / 車子正在跑
//    藍燈 = 倒數或其他狀態
// 4. 強化 48 mm 寬黑線轉彎能力
// 5. 直角彎會鎖定轉彎一小段時間，避免太早放掉
//
// 接線前提：
// 1. ESP32
// 2. L298N ENA、ENB 跳帽插上
// 3. OUT1/OUT2 接左側兩顆馬達
// 4. OUT3/OUT4 接右側兩顆馬達
// 5. 感測器黑線 = 1，白色 = 0
// =====================================================================

#include <Arduino.h>

// =====================================================================
// 1. 按鈕與 ASL 指示燈
// =====================================================================

const int VLS_PIN = 4;   // 起跑/急停按鈕，按鈕另一端接 GND

const int LED_R = 2;
const int LED_G = 15;
const int LED_B = 5;

// 如果你們 RGB LED 是「高電位亮」，維持 true
// 如果燈號顏色怪怪的，改成 false
const bool LED_ACTIVE_HIGH = true;

const int LED_ON  = LED_ACTIVE_HIGH ? HIGH : LOW;
const int LED_OFF = LED_ACTIVE_HIGH ? LOW  : HIGH;

// =====================================================================
// 2. 五路循跡感測器腳位
// 左到右：L2、L1、M、R1、R2
// =====================================================================

const int S_L2 = 25;
const int S_L1 = 26;
const int S_M  = 27;
const int S_R1 = 14;
const int S_R2 = 12;

// 你們目前設定：黑線 = 1，白色 = 0
// 如果實測相反，改成 BLACK = 0, WHITE = 1
const int BLACK = 1;
const int WHITE = 0;

// 如果感測器左右裝反，改成 true
const bool SWAP_SENSOR_SIDE = false;

// =====================================================================
// 3. L298N 馬達腳位
// 這組是前面依照你們車子狀況修正後的版本
// 若 M 黑線時四輪會往前，就不要改這四行
// =====================================================================

const int M_L1 = 19;   // 左側馬達方向 1
const int M_L2 = 18;   // 左側馬達方向 2

const int M_R1 = 17;   // 右側馬達方向 1
const int M_R2 = 16;   // 右側馬達方向 2

// 如果直走正確，但左轉右轉相反，改成 true
const bool SWAP_TURN_DIRECTION = false;

// 如果左側整組前後反了，改成 true
const bool INVERT_LEFT = false;

// 如果右側整組前後反了，改成 true
const bool INVERT_RIGHT = false;

// 如果左右馬達通道整組反了，改成 true
const bool SWAP_MOTOR_SIDE = false;

// =====================================================================
// 4. PWM 設定
// L298N 的 ENA/ENB 跳帽插著時，可以用 IN 腳 PWM 控速
// =====================================================================

const int PWM_FREQ = 20000;
const int PWM_BITS = 8;
const int PWM_MAX  = 255;

const int CH_L1 = 0;
const int CH_L2 = 1;
const int CH_R1 = 2;
const int CH_R2 = 3;

// =====================================================================
// 5. 速度參數
// 如果轉彎轉不動，優先調 SHARP_FAST、SHARP_SLOW、MIN_TURN_TIME
// =====================================================================

// 一般速度
int BASE_SPEED = 185;

// 中線穩定時直線速度
int FAST_SPEED = 205;

// 小彎速度
int MINOR_FAST = 200;
int MINOR_SLOW = 75;

// 直角彎速度
// 外側輪全速前進，內側輪全速後退
int SHARP_FAST = 255;
int SHARP_SLOW = -255;

// 掉線找線速度
int LOST_SPEED = 150;

// 直角彎最短鎖定時間
// 轉不動：130 改 160 或 180
// 轉過頭：130 改 100 或 90
const unsigned long MIN_TURN_TIME = 150;

// 直角彎最長鎖定時間
const unsigned long MAX_TURN_TIME = 550;

// 按鈕防彈跳時間
const unsigned long BUTTON_DEBOUNCE = 350;

// =====================================================================
// 6. 狀態變數
// =====================================================================

bool isRunning = false;

int lastDirection = 0;    // -1 = 左，1 = 右，0 = 中間
int turnMode = 0;         // -1 = 左急轉，1 = 右急轉，0 = 無

unsigned long turnStartTime = 0;
unsigned long lastButtonTime = 0;
unsigned long lastPrintTime = 0;

// =====================================================================
// 7. ESP32 PWM 相容處理
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
// 8. 初始化
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

  // 開機待命：安全狀態，綠燈
  setASL_GREEN();

  Serial.println("========================================");
  Serial.println("ESP32 + L298N 五路黑線車 最終統整版");
  Serial.println("按一下按鈕：起跑");
  Serial.println("行駛中再按一下：急停");
  Serial.println("綠燈：安全狀態");
  Serial.println("紅燈：自主導航狀態");
  Serial.println("藍燈：倒數 / 其他狀態");
  Serial.println("========================================");
}

// =====================================================================
// 9. 主迴圈
// =====================================================================

void loop() {
  handleButton();

  if (!isRunning) {
    setMotor(0, 0);
    setASL_GREEN();   // 停止時保持綠燈
    return;
  }

  setASL_RED();       // 車子跑的時候保持紅燈
  followLine();
}

// =====================================================================
// 10. 按鈕控制：按一下起跑，再按一下急停
// =====================================================================

void handleButton() {
  if (digitalRead(VLS_PIN) == LOW && millis() - lastButtonTime > BUTTON_DEBOUNCE) {
    lastButtonTime = millis();

    if (!isRunning) {
      Serial.println(">>> VLS 觸發：準備起跑 <<<");

      setMotor(0, 0);
      turnMode = 0;
      lastDirection = 0;

      // 倒數期間：藍燈
      Serial.println("3");
      blinkBlue();

      Serial.println("2");
      blinkBlue();

      Serial.println("1");
      blinkBlue();

      Serial.println("GO!");

      isRunning = true;
      setASL_RED();   // 自主導航：紅燈
    }
    else {
      Serial.println("!!! EBS 急停：進入安全狀態 !!!");

      isRunning = false;
      turnMode = 0;

      setMotor(0, 0);
      setASL_GREEN(); // 安全狀態：綠燈
    }
  }
}

void blinkBlue() {
  setASL_BLUE();
  delay(300);

  setASL_OFF();
  delay(300);
}

// =====================================================================
// 11. 尋線主邏輯
// =====================================================================

void followLine() {
  int l2 = digitalRead(S_L2);
  int l1 = digitalRead(S_L1);
  int m  = digitalRead(S_M);
  int r1 = digitalRead(S_R1);
  int r2 = digitalRead(S_R2);

  // 若感測器左右裝反，可用 SWAP_SENSOR_SIDE 一鍵交換
  if (SWAP_SENSOR_SIDE) {
    int temp;

    temp = l2;
    l2 = r2;
    r2 = temp;

    temp = l1;
    l1 = r1;
    r1 = temp;
  }

  bool L2 = (l2 == BLACK);
  bool L1 = (l1 == BLACK);
  bool M  = (m  == BLACK);
  bool R1 = (r1 == BLACK);
  bool R2 = (r2 == BLACK);

  int activeCount = 0;
  if (L2) activeCount++;
  if (L1) activeCount++;
  if (M)  activeCount++;
  if (R1) activeCount++;
  if (R2) activeCount++;

  // ---------------------------------------------------------
  // A. 直角彎鎖定模式
  // 進入直角彎後，不要太早因為看到中間線就放掉
  // ---------------------------------------------------------
  if (turnMode != 0) {
    unsigned long t = millis() - turnStartTime;

    if (turnMode == -1) {
      hardLeft();
    }
    else if (turnMode == 1) {
      hardRight();
    }

    // 至少轉 MIN_TURN_TIME 後，才允許解除
    if (t > MIN_TURN_TIME) {
      if (M || activeCount >= 3 || t > MAX_TURN_TIME) {
        turnMode = 0;
      }
    }

    printSensor(l2, l1, m, r1, r2, "LOCK_TURN");
    return;
  }

  // ---------------------------------------------------------
  // B. 全白：掉線
  // 根據最後一次方向找線
  // ---------------------------------------------------------
  if (activeCount == 0) {
    if (lastDirection < 0) {
      hardLeft();
      printSensor(l2, l1, m, r1, r2, "LOST_LEFT");
    }
    else if (lastDirection > 0) {
      hardRight();
      printSensor(l2, l1, m, r1, r2, "LOST_RIGHT");
    }
    else {
      setMotor(120, 120);
      printSensor(l2, l1, m, r1, r2, "LOST_FORWARD");
    }

    return;
  }

  // ---------------------------------------------------------
  // C. 最外側感測器優先：直角彎
  // 48 mm 黑線很寬，所以外側感測器要最高優先權
  // ---------------------------------------------------------
  if (L2 && !R2) {
    turnMode = -1;
    turnStartTime = millis();
    lastDirection = -1;

    hardLeft();
    printSensor(l2, l1, m, r1, r2, "HARD_LEFT");
    return;
  }

  if (R2 && !L2) {
    turnMode = 1;
    turnStartTime = millis();
    lastDirection = 1;

    hardRight();
    printSensor(l2, l1, m, r1, r2, "HARD_RIGHT");
    return;
  }

  // ---------------------------------------------------------
  // D. 左側偏移：左轉修正
  // ---------------------------------------------------------
  if ((L1 || L2) && !R1 && !R2) {
    lastDirection = -1;

    softLeft();
    printSensor(l2, l1, m, r1, r2, "SOFT_LEFT");
    return;
  }

  // ---------------------------------------------------------
  // E. 右側偏移：右轉修正
  // ---------------------------------------------------------
  if ((R1 || R2) && !L1 && !L2) {
    lastDirection = 1;

    softRight();
    printSensor(l2, l1, m, r1, r2, "SOFT_RIGHT");
    return;
  }

  // ---------------------------------------------------------
  // F. 中間黑線：直走
  // ---------------------------------------------------------
  if (M) {
    lastDirection = 0;

    if (activeCount >= 3) {
      // 寬線或交會處，不要太暴衝
      setMotor(BASE_SPEED, BASE_SPEED);
      printSensor(l2, l1, m, r1, r2, "WIDE_FORWARD");
    }
    else {
      setMotor(FAST_SPEED, FAST_SPEED);
      printSensor(l2, l1, m, r1, r2, "FAST_FORWARD");
    }

    return;
  }

  // ---------------------------------------------------------
  // G. 左右都有黑，可能是寬線或膠帶重疊，慢速直走
  // ---------------------------------------------------------
  if (activeCount >= 3) {
    setMotor(BASE_SPEED, BASE_SPEED);
    printSensor(l2, l1, m, r1, r2, "WIDE_LINE");
    return;
  }

  // ---------------------------------------------------------
  // H. 其他不明狀況：照最後方向修正
  // ---------------------------------------------------------
  if (lastDirection < 0) {
    softLeft();
    printSensor(l2, l1, m, r1, r2, "UNKNOWN_LEFT");
  }
  else if (lastDirection > 0) {
    softRight();
    printSensor(l2, l1, m, r1, r2, "UNKNOWN_RIGHT");
  }
  else {
    setMotor(BASE_SPEED, BASE_SPEED);
    printSensor(l2, l1, m, r1, r2, "UNKNOWN_FORWARD");
  }
}

// =====================================================================
// 12. 動作函式
// =====================================================================

void softLeft() {
  if (!SWAP_TURN_DIRECTION) {
    // 左轉：左側慢，右側快
    setMotor(MINOR_SLOW, MINOR_FAST);
  }
  else {
    setMotor(MINOR_FAST, MINOR_SLOW);
  }
}

void softRight() {
  if (!SWAP_TURN_DIRECTION) {
    // 右轉：左側快，右側慢
    setMotor(MINOR_FAST, MINOR_SLOW);
  }
  else {
    setMotor(MINOR_SLOW, MINOR_FAST);
  }
}

void hardLeft() {
  if (!SWAP_TURN_DIRECTION) {
    // 急左轉：左側後退，右側前進
    setMotor(SHARP_SLOW, SHARP_FAST);
  }
  else {
    setMotor(SHARP_FAST, SHARP_SLOW);
  }
}

void hardRight() {
  if (!SWAP_TURN_DIRECTION) {
    // 急右轉：左側前進，右側後退
    setMotor(SHARP_FAST, SHARP_SLOW);
  }
  else {
    setMotor(SHARP_SLOW, SHARP_FAST);
  }
}

// =====================================================================
// 13. 馬達控制
// left / right：
// 正值 = 前進
// 負值 = 後退
// 0 = 停止
// =====================================================================

void setMotor(int left, int right) {
  if (SWAP_MOTOR_SIDE) {
    int temp = left;
    left = right;
    right = temp;
  }

  if (INVERT_LEFT) {
    left = -left;
  }

  if (INVERT_RIGHT) {
    right = -right;
  }

  left  = constrain(left,  -PWM_MAX, PWM_MAX);
  right = constrain(right, -PWM_MAX, PWM_MAX);

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

// =====================================================================
// 14. ASL 燈號控制
// =====================================================================

void setASL_RED() {
  digitalWrite(LED_R, LED_ON);
  digitalWrite(LED_G, LED_OFF);
  digitalWrite(LED_B, LED_OFF);
}

void setASL_GREEN() {
  digitalWrite(LED_R, LED_OFF);
  digitalWrite(LED_G, LED_ON);
  digitalWrite(LED_B, LED_OFF);
}

void setASL_BLUE() {
  digitalWrite(LED_R, LED_OFF);
  digitalWrite(LED_G, LED_OFF);
  digitalWrite(LED_B, LED_ON);
}

void setASL_OFF() {
  digitalWrite(LED_R, LED_OFF);
  digitalWrite(LED_G, LED_OFF);
  digitalWrite(LED_B, LED_OFF);
}

// =====================================================================
// 15. Serial 除錯輸出
// =====================================================================

void printSensor(int l2, int l1, int m, int r1, int r2, const char* action) {
  if (millis() - lastPrintTime < 80) {
    return;
  }

  lastPrintTime = millis();

  Serial.print("S=");
  Serial.print(l2);
  Serial.print(l1);
  Serial.print(m);
  Serial.print(r1);
  Serial.print(r2);

  Serial.print("  ACTION=");
  Serial.println(action);
}
