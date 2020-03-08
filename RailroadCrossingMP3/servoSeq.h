#ifndef SERVO_H_
#define SERVO_H_

// 状態基底クラス
class ServoSequence
{
public:
  ServoSequence(char ch,unsigned char port, unsigned int pn[10][3],int MinAngle, int MaxAngle);
  int stateCheck();
  void servoABwite(char ch, int ref);
  int nowState();
  void stateReset();

  
private:
  char state = ST_INIT;
  char updownFlg;
  char svch;
  
  unsigned char port;
	unsigned char adr;
  unsigned int ptn[10][3];

  float PwmRef;
  float deltPwm;            // 10msあたりのpwm増加量
  
  int nowPwm;
  int nextPwm;
  int styTime;

//  Servo   ServoA;  // create servo object to control a servo
//  Servo   ServoB;  // create servo object to control a servo
//  ServoTimer2   ServoA;  // create servo object to control a servo
//  ServoTimer2   ServoB;  // create servo object to control a servo

void ServoCont(char port,int ms, char cnt);


  enum{
    ST_INIT = 0,
    ST_FARST,
    ST_RUN,
    ST_STAY,
    ST_NEXT,
    ST_END,
  };

  enum{
    DOWN = 0,
    STY,
    UP,
  };
  


};

#endif
