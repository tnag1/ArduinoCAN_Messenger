// demo: CAN-BUS Shield, receive data with check mode
// send data coming to fast, such as less than 10ms, you can use this way
// loovee, 2014-6-13

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/
#include <SPI.h>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/
#define CAN_2515
// For Arduino MCP2515 Hat:
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10 = minima
// pin　:　ピン番号、デジタルピンは0~13、アナログピンはA0~A5のように宣言する。
const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
const int SMR_B_LED_PIN = 4;//3;
const int smrG_ledPin = 6;//5;
const int smrP_ledPin = 7;//6;
const int BG_ON_btnPin = A0;    //b
const int G_ON_btnPin = A1;     //G
const int BGP_OFF_btnPin = A2;  //off
const int P_ON_btnPin = A3;     //p
const unsigned long BatEcuSendId = 0x3a4;
const unsigned long BatEcuReceiveId = 0x22e;
const int longPressThreshold = 2000;  //長押し判定時間(ms)
#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN);  // Set CS pin
#endif
unsigned long c_time;
unsigned long merc;        // フレーム管理時計用
const int SYCLE_TIME = 5;  //最小周期時間
unsigned char stmp[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned char buf[8];
unsigned char len = 0;
unsigned long canId = 0;
unsigned char smrState;
unsigned char smrB_StateLed;
unsigned char smrG_StateLed;
unsigned char smrP_StateLed;
bool blinking = true;
unsigned long btCount_B = 0;
unsigned long btCount_G = 0;
unsigned long btCount_P = 0;
unsigned long btCount_OFF = 0;
unsigned int ReadErrCounter = 0;
bool readState;
bool maskState = true;

/**************************************************************************************
 * Send CAN Messages
 **************************************************************************************/
const unsigned char B_ON[8] = { 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f };
const unsigned char G_ON[8] = { 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xcf };
const unsigned char P_ON[8] = { 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xef };
const unsigned char BG_ON[8] = { 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4f };
const unsigned char BP_ON[8] = { 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6f };
const unsigned char BGP_ON[8] = { 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f };
const unsigned char BGP_OFF[8] = { 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xaf };  //★送信内容　要確認
// int debgMsg = true;

/**************************************************************************************
* SETUP/LOOP
**************************************************************************************/

void setup() {
  // Serial.println(SPI_CS_PIN);  //del
  Serial.begin(9600);

  while (CAN_OK != CAN.begin(CAN_500KBPS)) {  // init can bus : baudrate = 500k
    Serial.println("CAN init fail, retry...");
    delay(100);
  }
  //--入力ボタン,出力ボタンLED 初期設定--//
  pinMode(SMR_B_LED_PIN, OUTPUT);
  pinMode(smrG_ledPin, OUTPUT);
  pinMode(smrP_ledPin, OUTPUT);
  pinMode(G_ON_btnPin, INPUT_PULLUP);
  pinMode(BG_ON_btnPin, INPUT_PULLUP);
  pinMode(BGP_OFF_btnPin, INPUT_PULLUP);
  pinMode(P_ON_btnPin, INPUT_PULLUP);

  //--CAN ID マスク、フィルタ処理 --//
  Serial.println("CAN init ok!");
  //全範囲マスクし、フィルタチャンネルのみ通過
  CAN.init_Mask(0, 0, 0x7FF);  // there are 2 mask in mcp2515, you need to set both of them
  CAN.init_Mask(1, 0, 0x7FF);
  CAN.init_Filt(0, 0, BatEcuReceiveId);
  // CAN.init_Filt(1, 0, 0x05);                          // there are 6 filter in mcp2515
  // CAN.init_Filt(2, 0, 0x06);                          // there are 6 filter in mcp2515
  // CAN.init_Filt(3, 0, 0x07);                          // there are 6 filter in mcp2515
  // CAN.init_Filt(4, 0, 0x08);                          // there are 6 filter in mcp2515
  // CAN.init_Filt(5, 0, 0x09);                          // there are 6 filter in mcp2515
}

void loop() {
  //  5ms周期処理 /////////////////////////////////
  if (c_time % (5 / SYCLE_TIME) < 1) {

    //CANを読み込み。ID:22eをRead→1バイト目の値をグローバル変数へ代入
    readState = getSmrState(BatEcuReceiveId, 0);

    //CAN通信エラー時間を蓄積
    if (readState == false) {
      ReadErrCounter++;
    };

    //--ボタン入力を確認し、入力条件に従いCANメッセージ送信
    checkBotton(5);
  }

  //  200ms周期処理 /////////////////////////////////
  if (c_time % (200 / SYCLE_TIME) < 1) {
    //--LED-ON処理--　変数が変化したらLEDの点灯、点滅処理
    toggleLED();

    //通信エラーカウントが"500"を超えたらフラグをリセット
    if (ReadErrCounter > 500) {
      smrB_StateLed = 0;
      smrG_StateLed = 0;
      smrP_StateLed = 0;
      ReadErrCounter = 0;
    }
  }

  //  2000ms周期処理 ////////////////////////////////
  if (c_time % (2000 / SYCLE_TIME) < 1) {
    //処理なし
  }
  delay(SYCLE_TIME);
  c_time = c_time + 1;
}

/**************************************************************************************
* Functions
**************************************************************************************/
void toggleLED() {
  // 機能：SMRの変数(グローバル)の状態によりLEDの点灯処理を行う。(0:消灯, 1:点滅, 2:点灯)

  if (smrB_StateLed == 2) {
    digitalWrite(SMR_B_LED_PIN, HIGH);
  } else if (smrB_StateLed == 1) {
    blinking ? digitalWrite(SMR_B_LED_PIN, HIGH) : digitalWrite(SMR_B_LED_PIN, LOW);
    blinking = !blinking;
  } else if (smrB_StateLed == 0) {
    digitalWrite(SMR_B_LED_PIN, LOW);
  }
  if (smrG_StateLed == 2) {
    digitalWrite(smrG_ledPin, HIGH);
  } else if (smrG_StateLed == 1) {
    blinking ? digitalWrite(smrG_ledPin, HIGH) : digitalWrite(smrG_ledPin, LOW);
    blinking = !blinking;
  } else if (smrG_StateLed == 0) {
    digitalWrite(smrG_ledPin, LOW);
  }
  if (smrP_StateLed == 2) {
    digitalWrite(smrP_ledPin, HIGH);
  } else if (smrP_StateLed == 1) {
    blinking ? digitalWrite(smrP_ledPin, HIGH) : digitalWrite(smrP_ledPin, LOW);
    blinking = !blinking;
  } else if (smrP_StateLed == 0) {
    digitalWrite(smrP_ledPin, LOW);
  }
}

bool getSmrState(unsigned long canIdValue, int targetBytePosition) {
  // 引数:canIdValue = CAN ID
  //      targetBytePosition = 返り値用のバイト位置”0～7”
  // 機能:CAN値 読み込み。指定のCAN-IDを得たら1バイト目の値(SMR状態)をSMR状態の変数にセット
  bool l_receiveState;

  if (CAN_MSGAVAIL == CAN.checkReceive()) {  // check if data coming
    CAN.readMsgBuf(&len, buf);               // read data,  len: data length, buf: data buf
    canId = CAN.getCanId();

    //--debug用↓↓--
    Serial.print("CAN-ID:");
    Serial.print(canId, HEX);  //　CANメッセージ受信　→　変数に状態を書き込み
    Serial.print("  値:");
    for (int i = 0; i < len; i++) {
      Serial.print(buf[i], HEX);
      Serial.print(",");
    }
    Serial.println();
    //--debug用 ↑↑--

    //指定ID(0x22e)なら以下を実行
    if (canId == canIdValue) {
      smrState = buf[targetBytePosition];  //1バイト目

      setSmrState(smrState);  //SMR状態を変数へ代入する
      l_receiveState = true;
    }
  } else {
    l_receiveState = false;
  }
  return l_receiveState;
}

void checkBotton(int cTime) {
  //引数:cTime = 長押し判定用の時間。長押し判定に使用 "> longPressThreshold(ms)"
  //機能:ボタンの状態をチェックし、押されたら処理を実施
  int prevCounter_B;    //前回のカウント値
  int prevCounter_G;    //前回のカウント値
  int prevCounter_P;    //前回のカウント値
  int prevCounter_OFF;  //前回のカウント値

  // 【1】BG-ONボタン処理(シーケンス処理)
  prevCounter_B = btCount_B;
  if (digitalRead(BG_ON_btnPin) == 0) {
    btCount_B += cTime;  //押している間カウントアップする
  }
  if (btCount_B == longPressThreshold) {  //カウントが”長押し判定時間”を超えたら処理
    Serial.println("マスク処理変更");
    if (maskState == true) {
      CAN.init_Mask(0, 0, 0);  // there are 2 mask in mcp2515, you need to set both of them
      CAN.init_Mask(1, 0, 0);
      CAN.init_Filt(0, 0, 0);
      maskState = !maskState;
    }
    else if (maskState == false) {
      // 処理の割り当てなし
      CAN.init_Mask(0, 0, 0x7FF);  // there are 2 mask in mcp2515, you need to set both of them
      CAN.init_Mask(1, 0, 0x7FF);
      CAN.init_Filt(0, 0, BatEcuReceiveId);
      maskState = !maskState;
    }
  }
  if ((prevCounter_B == btCount_B) && (btCount_B > 0)) {  //短時間でボタンを離すと"BG-ON"処理
    if (btCount_B < longPressThreshold) {
      Serial.println("-- BG_ON シーケンス処理開始 --");
      sendMessage(BatEcuSendId, "BG_ON");
    }
    btCount_B = 0;
  }

  // 【2】Gボタン処理--
  prevCounter_G = btCount_G;
  if (digitalRead(G_ON_btnPin) == 0) {
    btCount_G += cTime;  //押している間カウントアップする
  }
  if (btCount_G == longPressThreshold) {  //カウントが”長押し判定時間”を超えると"P-ON"処理
    Serial.println("P_ONメッセージ送信!");
    sendMessage(BatEcuSendId, "P_ON");
  }
  if (prevCounter_G == btCount_G && btCount_G > 0) {  //短時間でボタンを離すと'G-ON'処理
    if (btCount_G < longPressThreshold) {
      Serial.println("G_ONメッセージ送信!");
      sendMessage(BatEcuSendId, "G_ON");
    }
    btCount_G = 0;
  }

  // 【3】OFFボタン処理
  prevCounter_OFF = btCount_OFF;
  if (digitalRead(BGP_OFF_btnPin) == 0) {
    btCount_OFF += cTime;  //押している間カウントアップする
  }
  if (btCount_OFF == longPressThreshold) {  //カウントが”長押し判定時間”を超えると"LEDチェック処理"
    Serial.println("LED動作チェック");
    digitalWrite(SMR_B_LED_PIN, HIGH);
    delay(100);
    digitalWrite(SMR_B_LED_PIN, LOW);
    delay(100);
    digitalWrite(smrG_ledPin, HIGH);
    delay(100);
    digitalWrite(smrG_ledPin, LOW);
    delay(100);
    digitalWrite(smrP_ledPin, HIGH);
    delay(100);
    digitalWrite(smrP_ledPin, LOW);
  }
  if (prevCounter_OFF == btCount_OFF && btCount_OFF > 0) {  //短時間でボタンを離すと'BGP-OFF'処理
    if (btCount_OFF < longPressThreshold) {
      Serial.println("BGP_OFFメッセージ送信!");
      sendMessage(BatEcuSendId, "BGP_OFF");
    }
    btCount_OFF = 0;
  }
}
void setSmrState(unsigned char smrState) {
  // 引数:smrState = 2バイトのSMR状態変数
  // 処理:SMR状態値をビット処理し、変数へ代入する
  // 0:消灯  1:点滅    2:点灯

  //SMR-B点灯の確認:消灯/点灯/点滅
  if ((smrState & B01000000) != 0) {  //マスク0x40
    // Serial.print(" B点灯:");
    smrB_StateLed = 2;
  } else if ((smrState & B00001000) != 0) {  //マスク0x08
    // Serial.println(" B点滅");
    smrB_StateLed = 1;
  } else if ((smrState & B01001000) == 0) {  //マスク0x48
    // Serial.println(" B消灯");
    smrB_StateLed = 0;
  }

  //SMR-G点灯の確認:消灯/点灯/点滅
  if ((smrState & B00100000) != 0) {  //マスク0x20　
    // Serial.print(" G点灯:");
    smrG_StateLed = 2;
  } else if ((smrState & B00000100) != 0) {  //マスク0x04
    // Serial.println(" G点滅");
    smrG_StateLed = 1;
  } else if ((smrState & B00100100) == 0) {  //マスク0x24
    // Serial.println(" G消灯");
    smrG_StateLed = 0;
  }

  //SMR-P点灯の確認:消灯/点灯/点滅
  if ((smrState & B10000000) != 0) {  //マスク0x80　
    // Serial.print(" P点灯:");
    smrP_StateLed = 2;
  } else if ((smrState & B00010000) != 0) {  //マスク0x10
    // Serial.println(" P点滅");
    smrP_StateLed = 1;
  } else if ((smrState & B10010000) == 0) {  //マスク0x90
    // Serial.println(" P消灯");
    smrP_StateLed = 0;
  }
}

void sendMessage(unsigned long cId, String smrMsg) {
  //引数:cId=電ECUのCAN-ID, smrMsg = CAN送信するメッセージ(SMR-GB, SMR-G, SMR-P, SMR-OFF)
  //機能:smrMsgに基づき、bt-ECUへSMR操作のCANメッセージ送信

  if (smrMsg == "B_ON") {
    memcpy(stmp, B_ON, sizeof(B_ON));  //G-ONのみ
    CAN.sendMsgBuf(cId, 0, 8, stmp);
  } else if (smrMsg == "G_ON") {
    memcpy(stmp, G_ON, sizeof(G_ON));  //G-ONのみ
    CAN.sendMsgBuf(cId, 0, 8, stmp);
  } else if (smrMsg == "P_ON") {
    memcpy(stmp, P_ON, sizeof(P_ON));  //G-ONのみ
    CAN.sendMsgBuf(cId, 0, 8, stmp);
  } else if (smrMsg == "BGP_OFF") {
    memcpy(stmp, BGP_OFF, sizeof(BGP_OFF));  //OFFのみ
    CAN.sendMsgBuf(cId, 0, 8, stmp);
  } else if (smrMsg == "B_OFF") {
    // memcpy(stmp, B_OFF, sizeof(B_OFF));  //OFFのみ
    CAN.sendMsgBuf(cId, 0, 8, stmp);
  } else if (smrMsg == "G_OFF") {
    // memcpy(stmp, G_OFF, sizeof(G_OFF));  //OFFのみ
    CAN.sendMsgBuf(cId, 0, 8, stmp);
  } else if (smrMsg == "P_OFF") {
    // memcpy(stmp, P_OFF, sizeof(P_OFF));  //OFFのみ
    CAN.sendMsgBuf(cId, 0, 8, stmp);
  }

  else if (smrMsg == "BG_ON") {  //B→BP→BPG→BG シーケンス処理
    int i = 0;
    bool l_readStatus;
    bool l_err = false;
    int errTry = 50;  //リトライ回数
    int waitTime = 10; //データチェックリトライの待ち時間

    //-- 1. B-ON処理 --
    do {
      i++;
      if (i == 1) {  //1回だけメッセージ送信
        memcpy(stmp, B_ON, sizeof(B_ON));
        CAN.sendMsgBuf(cId, 0, 8, stmp);
      }
      // 状態変化するまで繰り返し
      l_readStatus = getSmrState(BatEcuReceiveId, 0);  //読み取り処理,変数代入処理
      toggleLED();                                     //LED表示処理
      if (i >= errTry) l_err = true;
      delay(waitTime);

      //--debug用--
      Serial.println("●");
      Serial.print("B-ONループ(回)→");
      Serial.println(i);
      Serial.print("SMR変数値B,G,P(2=ON)→");
      Serial.print(smrB_StateLed);
      Serial.print(smrG_StateLed);
      Serial.println(smrP_StateLed);
      Serial.print("doループ判定値→");
      Serial.println((smrB_StateLed != 2) && i < errTry);
    } while ((smrB_StateLed != 2) && i < errTry);
    Serial.println();

    //-- 2. BP-ON処理 --
    i = 0;
    if (l_err == false) {
      do {
        i++;
        if (i == 1) {  //1回だけメッセージ送信
          memcpy(stmp, BP_ON, sizeof(BP_ON));
          CAN.sendMsgBuf(cId, 0, 8, stmp);
        }
        // 状態変化するまで繰り返し
        l_readStatus = getSmrState(BatEcuReceiveId, 0);  //読み取り処理,変数代入処理
        toggleLED();                                     //LED表示処理
        if (i >= errTry) l_err = true;
        delay(waitTime);

        //--debug用--
        Serial.println("▲");
        Serial.print("BP-ONループ(回)→");
        Serial.println(i);
        Serial.print("SMR変数値P,G,B(2=ON)→");
        Serial.print(smrB_StateLed);
        Serial.print(smrG_StateLed);
        Serial.println(smrP_StateLed);
        Serial.print("doループ判定値→");
        Serial.println(!(smrB_StateLed == 2 && smrP_StateLed == 2) && i < errTry);
      } while (!(smrB_StateLed == 2 && smrP_StateLed == 2) && i < errTry);
    }
    delay(1000);
    Serial.print("BP err→");
    Serial.println(l_err);

    //-- 3. BGP-ON処理 --
    i = 0;
    if (l_err != true) {
      do {
        i++;
        if (i == 1) {  //1回だけメッセージ送信
          memcpy(stmp, BGP_ON, sizeof(BGP_ON));
          CAN.sendMsgBuf(cId, 0, 8, stmp);
        }
        // 状態変化するまで繰り返し
        l_readStatus = getSmrState(BatEcuReceiveId, 0);  //読み取り処理,変数代入処理
        toggleLED();                                     //LED表示処理
        if (i >= errTry) l_err = true;
        delay(waitTime);

        //--debug用--
        Serial.println("■");
        Serial.print("BGP-ONループ(回)→");
        Serial.println(i);
        Serial.print("SMR変数値P,G,B→");
        Serial.print(smrB_StateLed);
        Serial.print(smrG_StateLed);
        Serial.println(smrP_StateLed);
        Serial.print("doループ判定値→");
        Serial.println(!(smrB_StateLed == 2 && smrG_StateLed == 2 && smrP_StateLed == 2) && (i < errTry));
      } while (!(smrB_StateLed == 2 && smrG_StateLed == 2 && smrP_StateLed == 2) && (i < errTry));
    }
    Serial.print("BGP err→");
    Serial.println(l_err);

    //-- 4. BG-ON処理 --
    i = 0;
    if (l_err != true) {
      do {  //1回だけメッセージ送信
        i++;
        if (i == 1) {
          memcpy(stmp, BG_ON, sizeof(BG_ON));  //B→BP→BGP→BG
          CAN.sendMsgBuf(cId, 0, 8, stmp);     //読み取り処理,変数代入処理
        }
        // 状態変化するまで繰り返し
        l_readStatus = getSmrState(BatEcuReceiveId, 0);
        toggleLED();  //LED表示処理
        if (i >= errTry) l_err = true;
        delay(waitTime);

        //--debug用--
        Serial.println("★");
        Serial.print("BG-ONループ(回)→");
        Serial.println(i);
        Serial.print("SMR変数値P,G,B→");
        Serial.print(smrB_StateLed);
        Serial.print(smrG_StateLed);
        Serial.println(smrP_StateLed);
        Serial.print("3.BG ループ判定値→");
        Serial.println(!(smrB_StateLed == 2 && smrG_StateLed == 2 && smrP_StateLed == 0) && (i < errTry));
      } while (!(smrB_StateLed == 2 && smrG_StateLed == 2 && smrP_StateLed == 0) && (i < errTry));  //(smrB_StateLed == 2) && (smrG_StateLed == 2) && (smrP_StateLed == 2) &&
    }
    Serial.print("BG err→");
    Serial.println(l_err);
  }
  // void timeMeasurement() {
  //   unsigned long time;    //「time」をunsigned longで変数宣言
  //   time = millis();       //プログラム実行からの経過時間(ms)をtimeに返す
  //   Serial.print(time);    //経過時間(ms)を送信
  //   Serial.println("ms");  //文字列「ms」を送信、改行
}

/*********************************************************************************************************
    END FILE
*********************************************************************************************************/
