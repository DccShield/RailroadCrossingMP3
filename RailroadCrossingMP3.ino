//
// servoライブラリとSoftwareserialの相性が悪い。
// ソフトウエアシリアルで文字を送信するとPWMのON幅が変わってしまう。
//
// dftobotDFPlayerminiのドライバをやめてみた。
// けどサーボに影響あり。
//
// 最終バージョン。
//
#include  <arduino.h>
#include <SoftwareSerial.h>
#include "servoSeq.h"
#include "led.h"
#include "cds.h"


//各種設定、宣言

#define PIN_F0_F  3     // D3 PD3,PWM
#define PIN_F0_R  4     // D4 PD4
#define PIN_AUX1  5     // D5 PD5
#define PIN_AUX2  6     // D6 PD6
#define PIN_AUX3  7     // D7 PD7
#define PIN_AUX4  8     // D8 PB0
#define PIN_AUX5  9     // D9 PB1,DIGITAL TR,PWM
#define PIN_AUX6  10    // D10 PB2,DIGITAL TR,PWM
#define PIN_AUX7  11    // D11 PB3,DIGITAL TR,PWM

#define MAX_SERVO 2     // サーボの最大数2個
#define PIN_SERVO1 5    // D5 PD5
#define PIN_SERVO2 4    // D4 PD4
#define PIN_LED_DIV1 6  // D6 PD6 SERVO1用DIV(分岐) LED
#define PIN_LED_STR1 7  // D7 PD7 SERVO1用STR(直進) LED
#define PIN_LED_DIV2 8  // D8 PB0 SERVO2用DIV(分岐) LED
#define PIN_LED_STR2 9  // D9 PB1 SERVO2用STR(直進) LED

#define ANGLE_MIN_A 800    // DM-S0025 400-2100  0-180deg
#define ANGLE_MAX_A 2200     // DM-S0037 1000-2000 0-90deg 
                             // DM-S0037 670-2600 0-180deg
                             
#define ANGLE_MIN_B 500     // 1000 -90deg
#define ANGLE_MAX_B 2000    // 2000 +90deg


#define TrainWait 10      // Cdsセンサ踏んでからカンカンしだす待ち時間(100ms)
#define RCwait 20         // カンカンしてから経過後遮断機動作までのウエイト(10ms)

unsigned long PreviosTimeMainState = 0;
unsigned long PreviosTimeServo = 0;

enum{
  ST_RCIDLE = 0,
  ST_RCINIT,
  ST_RCRUN,
  ST_RCEND,
};

int RCstate = ST_RCIDLE;  

unsigned int ptn1[10][3]={//ms   ,deg
                          { 0    ,75 },
                          { 200  ,70 }, // 200:2000ms
                          { 350  ,10 }, // 350:3500ms
                          { 200  , 0 },
                          { 600  , 0 },
                          { 150  ,10 },
                          { 200  ,70 },
                          { 150  ,75 },                          
                          { 0    ,999 }};

unsigned int ptn2[10][3]={//ms   ,deg
                          { 0    ,75 },
                          { 400  ,75 },
                          { 200 ,70 }, // 200:2000ms
                          { 350  ,10 }, // 350:3500ms
                          { 200  , 0 },
                          { 200  , 0 },
                          { 150  ,10 },
                          { 200  ,70 },
                          { 150  ,75 },                          
                          { 0    ,999 }};
                 
unsigned int ptn3[10][3]={//ms   ,deg
                          { 0    ,90 },
                          { 30   ,0  },
                          { 30   ,45 },
                          { 30   ,0  },
                          { 30   ,90 },
                          { 30   ,0  },
                          { 30   ,45 },
                          { 30   ,0  },
                          { 30   ,90 },                          
                          { 0    ,999 }};

WarningLights WarnLight(500);
Cds TrainSensor(0,1,2,200,500);    // CDSセンサのセットアップ
SoftwareSerial mySoftwareSerial(A5, A4); // RX, TX

void setup() {
  Serial.begin(115200);

Serial.println("RailRoad Crossing"); 
delay(1000);   
  // ソフトウエアシリアル通信レートセット:
  mySoftwareSerial.begin(9600);

  pinMode(A3, INPUT);  //Mp3 Busy pin 設定
  digitalWrite(A3, HIGH); //Internal pull up enabled
 
//  myDFPlayer.volume(5);  //Set volume value. From 0 to 30
  DFPuart(0x06,0x10); 

  WarnLight.FlashOnOff(0);  // 赤ランプ初期化

  PreviosTimeMainState = millis();
  PreviosTimeServo = PreviosTimeMainState;
}

void loop() {
  static int ret = 0;
   
  if( (millis() - PreviosTimeMainState ) >= 100 ){  // 100msec
    PreviosTimeMainState = millis();
    ret = MainState();     // メインステートマシン
    
    if(ret & 0x01)         // カンカンステートマシン
      KanKanState();
      
    if(ret & 0x01)         // 赤ランプステートマシン
      WarnLight.StateCheck();

  }

  if( (millis() - PreviosTimeServo ) >= 10 ){  // 100msec
    PreviosTimeServo = millis();
    if( ret & 0x02 )       // 遮断機ステートマシン 
      RailloadCrossingState();
  }
}

//--------------------------------------------------------------------------------
// MainState
// メインステートマシン
//--------------------------------------------------------------------------------
int MainState(void){
enum {
  ST_INIT = 0,
  ST_TRSENSING,
  ST_STTRAINWAIT,
  ST_KANKAN,
  ST_RCWAIT,
  ST_RCRUN,
  ST_ENDWAIT,
};

  int re;
  static int state = ST_INIT;
  static int retn = 0;
  static int wait = 0;
  
  switch(state){
    case ST_INIT:
Serial.println("MainState:ST_INIT");    
                  state = ST_TRSENSING;
                  retn = 0;
                  break;
    case ST_TRSENSING:      // 車両侵入チェック
                   re = TrainSensor.statechk(LOW); // 明るさ閾値LOWでチェック
                  if( re == 0 )
                    break;
                  else {        
                    TrainSensor.Reset();
Serial.println("MainState:ST_TRSENSING");
                    wait = TrainWait;
                    state = ST_STTRAINWAIT;
                  }
                  break;                
    case ST_STTRAINWAIT:
                  wait--;
Serial.println(wait);
                  if(wait<=0){
Serial.println("MainState:ST_STTRAINWAIT");
                    state = ST_KANKAN;
                  }
                  break;   
    case ST_KANKAN:
Serial.println("MainState:ST_KANKAN");
                  WarnLight.FlashOnOff(1);  // 赤ランプ初期化
                  retn = 0x01; // 警報機ライト点滅、カンカン鳴動 開始
                  wait = RCwait;
                  state = ST_RCWAIT;
                  break;
    case ST_RCWAIT:
                  wait--;
Serial.println(wait);
                  if(wait<=0){
Serial.println("MainState:ST_RCWAIT");
                    state = ST_RCRUN;
                  }
                  break;                     
    case ST_RCRUN:
Serial.println("MainState:ST_RCRUN");
                  retn = retn + 0x02; // 遮断機動作開始
                  state = ST_ENDWAIT;
                  break;
    case ST_ENDWAIT:
                  if(RCstate == ST_RCEND){
Serial.println("MainState:ST_ENDWAIT");
                    WarnLight.FlashOnOff(0);  // 赤ランプ初期化
                    WarnLight.StateCheck();
                    retn = 0;  
                    RCstate = ST_RCIDLE;
                    state = ST_INIT;
                  }
                  break;
    default:
                  break;
  }
  return retn;
}


//--------------------------------------------------------------------------------
// RailloadCrossingState
// 遮断機ステートマシン
//--------------------------------------------------------------------------------
void RailloadCrossingState(void){
static ServoSequence ServoA(0,PIN_SERVO1,ptn1,670,2600);
  static ServoSequence ServoB(1,PIN_SERVO2,ptn2,400,2100);

  switch(RCstate){
      case ST_RCIDLE:
            ServoA.stateReset();
            ServoB.stateReset();
            RCstate = ST_RCINIT;
            break;
      case ST_RCINIT:
            PreviosTimeServo = millis();
            RCstate = ST_RCRUN;
            break;
      case ST_RCRUN:
              ServoB.stateCheck();
              ServoA.stateCheck();
            if(ServoA.nowState() == 5 ){
              RCstate = ST_RCEND;
            }
            break;
      case ST_RCEND:
            break;
      default:
            break;
  }
}


//--------------------------------------------------------------------------------
// KanKanState
// カンカン音ステートマシン
//--------------------------------------------------------------------------------
void KanKanState(void){
  static byte SoundCnt = 0;

  if(SoundCnt >= 5){
    digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
    DFPuart(0x03,0x01);  
    SoundCnt = 0;
  }
  SoundCnt ++;
}

void DFPuart(char cmd, char par){
  char i;
  
  char DFPcmd[10]={0x7e,0xff,0x06,0x00,0x00,0x00,0x00,0xef};
  DFPcmd[3]=cmd;
  DFPcmd[6]=par;
  for(i=0;i<=7;i++){
     mySoftwareSerial.write((uint8_t)DFPcmd[i]);
  }
}
