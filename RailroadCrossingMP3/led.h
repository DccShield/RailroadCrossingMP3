//　ステートマシンクラス
#define PIN_LED_DIV1 6  // D6 PD6 SERVO1用DIV(分岐) LED
#define PIN_LED_STR1 7  // D7 PD7 SERVO1用STR(直進) LED
#define PIN_LED_DIV2 8  // D8 PB0 SERVO2用DIV(分岐) LED
#define PIN_LED_STR2 9  // D9 PB1 SERVO2用STR(直進) LED

//　クラスの定義
class WarningLights{
  enum {
    ST_INIT = 0,
    ST_IDLE,
    ST_DIVON,
    ST_DIVTIMER,
    ST_STRON,
    ST_STRTIMER,
    ST_OFF,
  } state = ST_IDLE;
  
private:
  int m_state;      // クラス内で使用するメンバ変数　（m_***)
  int m_flashtimer;
  unsigned long PreviosTimer = 0;
  
public:
  WarningLights(int);
  void StateCheck(void);
  void FlashOnOff(int);
};
