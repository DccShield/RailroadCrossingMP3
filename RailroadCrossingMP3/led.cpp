#include <arduino.h>
#include "led.h"

#define buttonPin 4

// コンストラクタ（初期化処理）
WarningLights::WarningLights( int FT )
{
  pinMode(PIN_LED_DIV1, OUTPUT);    
  pinMode(PIN_LED_STR1, OUTPUT);
  pinMode(PIN_LED_DIV2, OUTPUT);    
  pinMode(PIN_LED_STR2, OUTPUT);
  m_flashtimer = FT;
  state = ST_INIT;
}

void WarningLights::FlashOnOff( int flg )
{
  switch(flg){
      case 0:
              state = ST_OFF;
              break;
      case 1: state = ST_INIT;
              break;
      default:
              break;
  }
}

void WarningLights::StateCheck()
{
    switch(state){
      case ST_INIT:
                    state = ST_DIVON;
                    break;
      case ST_IDLE:
                    break;
      case ST_DIVON:
                    digitalWrite(PIN_LED_DIV1, HIGH);
                    digitalWrite(PIN_LED_STR1, LOW);
                    digitalWrite(PIN_LED_DIV2, HIGH);                 
                    digitalWrite(PIN_LED_STR2, LOW);
                    PreviosTimer = millis();
                    state = ST_DIVTIMER;
                    break;
      case ST_DIVTIMER:
                    if(( millis() - PreviosTimer ) >= m_flashtimer){
                      state = ST_STRON;
                    }
                    break;      
      case ST_STRON:
                    digitalWrite(PIN_LED_DIV1, LOW);
                    digitalWrite(PIN_LED_STR1, HIGH);
                    digitalWrite(PIN_LED_DIV2, LOW);                 
                    digitalWrite(PIN_LED_STR2, HIGH);
                    PreviosTimer = millis();
                    state = ST_STRTIMER;
                    break;
      case ST_STRTIMER:
                    if(( millis() - PreviosTimer ) >= m_flashtimer){
                      state = ST_DIVON;
                    }
                    break;
      case ST_OFF:
                    digitalWrite(PIN_LED_DIV1, LOW);
                    digitalWrite(PIN_LED_STR1, LOW);
                    digitalWrite(PIN_LED_DIV2, LOW);                 
                    digitalWrite(PIN_LED_STR2, LOW);
                    state = ST_IDLE;
                    break;
                                                  
      default:
                    break;
    }
}
