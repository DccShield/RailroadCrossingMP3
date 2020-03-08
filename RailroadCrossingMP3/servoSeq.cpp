//------------------------------------------------------------------------------
// ServoSequenceクラス
//------------------------------------------------------------------------------
#include <arduino.h>
#include "servoSeq.h"
//#include "ServoTimer2.h"
// コンストラクタ
ServoSequence::ServoSequence(char ch,unsigned char port, unsigned int pn[10][3],int MinAngle, int MaxAngle)
{
  char i;
  pinMode(port, OUTPUT);
  digitalWrite(port, HIGH);

  if(ch == 0){
//    ServoA.attach(port, MinAngle, MaxAngle);
//    ServoA.attach(port);
  }
  if(ch == 1){
//    ServoB.attach(port, MinAngle, MaxAngle);
//    ServoB.attach(port);
  }
  svch = ch;
  state = ST_INIT;

  for(i=0 ; i <= 10 ; i++){
    if(pn[i][1] == 999){
      ptn[i][1] = 999;
      break;
    }

    ptn[i][0] = pn[i][0];
    ptn[i][1] = pn[i][1];
    ptn[i][2] = (int)map(pn[i][1],0,180,MinAngle,MaxAngle);  // 角度→PwmRefに変換
  }
}

int ServoSequence::nowState()
{
  return state; 
}

void ServoSequence::stateReset()
{
  state = ST_INIT;
}

void ServoSequence::servoABwite(char ch, int ref)
{
  if(ch == 0){
ServoCont(5,ref,0);
  }
  if(ch == 1){
ServoCont(4,ref,0);
  }
}

// ServoSequence ステートマシン（状態遷移）
int ServoSequence::stateCheck()
{
  switch(state){
    case ST_INIT:
                  adr = 0;
                  deltPwm = 0;
                  state = ST_FARST;
                  break;
    case ST_FARST:
                  PwmRef = ptn[adr][2];
                  servoABwite(svch, (int)PwmRef);
                  adr++;
                  nowPwm = ptn[adr-1][2];
                  nextPwm = ptn[adr][2];
                  deltPwm = (nextPwm - nowPwm) / (float)(ptn[adr][0]);
                  if(nextPwm - nowPwm == 0){
                    updownFlg = STY;
                    styTime = ptn[adr][0];
                    state = ST_STAY;
                    break;
                  }
                  else if(nextPwm - nowPwm < 0)
                    updownFlg = DOWN;
                  else
                    updownFlg = UP;
                  state = ST_RUN;
                  break;
    case ST_STAY:
                  styTime --;
                  if(styTime <= 0)
                    state = ST_NEXT;
                  break;
    case ST_RUN:
                  PwmRef = PwmRef + deltPwm;
                  if((updownFlg == DOWN) && (PwmRef <= nextPwm)){
                    servoABwite(svch, (int)nextPwm);
                    state = ST_NEXT;
                  } else if((updownFlg == UP) && (PwmRef >= nextPwm)){
                    servoABwite(svch, (int)nextPwm);
                    state = ST_NEXT;
                  } else {
                    servoABwite(svch, (int)PwmRef);
                  }
                  break;
    case ST_NEXT:
                  adr++;
                  if(ptn[adr][1]==999){
                    state = ST_END;
                    break;
                  }
                  
                  nowPwm = ptn[adr-1][2];
                  nextPwm = ptn[adr][2];
                  deltPwm = (nextPwm - nowPwm) / (float)(ptn[adr][0]);
                  if(nextPwm - nowPwm == 0){
                    updownFlg = STY;
                    styTime = ptn[adr][0];
                    state = ST_STAY;
                    break;
                  }
                  else if(nextPwm - nowPwm < 0)
                    updownFlg = DOWN;
                  else
                    updownFlg = UP;
                  state = ST_RUN;
                  break;
    case ST_END:  // 5
                  break;

    default:
                  break;
  }
  return state; 
}
void ServoSequence::ServoCont(char port, int ms, char cnt){
char lp;
int wt;

  cli();  // 割り込み禁止
  for(lp=0;lp<=cnt;lp++){   
//    digitalWrite(port, HIGH); // HIGH-LOWまで24.3us
    PORTD |= _BV(port);
    for(wt=0;wt<ms;wt++){
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
    }
//    digitalWrite(port, LOW);
    PORTD &= ~_BV(port);
  }
  sei();  // 割り込み許可  
}
