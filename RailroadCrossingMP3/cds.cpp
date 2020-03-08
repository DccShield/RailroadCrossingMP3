#include  <arduino.h>
#include "cds.h"

Cds::Cds(char ch1 , char ch2 , char ct, int LTh, int HTh){
  port1 = ch1;
  port2 = ch2;
  cntup = ct;
  LThreshold = LTh;
  HThreshold = HTh;
}

void Cds::Reset( void ){
  state = ST_INIT;
}

char Cds::statechk( char range ){
  static unsigned long PreviosTimerSp;  // 速度監視用変数
  static unsigned long TimTemp;
    
  switch(state){
    case ST_INIT:
Serial.println("cds:ST_INIT");
              state = ST_MEAS1;
              cnt = 0;
              TimTemp = 0;
              break;
    case ST_MEAS1:
              if(cnt > cntup ){
                state = ST_DETECTION;
                cnt = 0;
                PreviosTimerSp = millis();
                break;
              }
              Ain = analogRead( port1 );
              if(range == LOW){
                if(Ain <=  LThreshold){
                  cnt++;
                } else
                  cnt = 0;
              }
              if(range == HIGH){
                if(Ain <=  HThreshold){
                  cnt++;
                } else
                  cnt = 0;
              }
              break;
     
    case ST_DETECTION:
              return 1;
              break;

    default:
              break;
  }
  return 0;
}
