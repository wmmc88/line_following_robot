#include "mte220.c"

#define ENABLE_LINE_FOLLOWING 1
#define ENABLE_MAGNET_DETECTION 1

#define TRUE 1
#define FALSE 0

#define VOLTS_TO_HEX(X) ((uns16)(X * 1024 / 5) >> 2)
#define SECS_TO_LONG_DELAY_COUNTS(X) (uns16)(X * 8)

/**Exponential Moving Average
 * Newest value is weighted by Alpha, where Alpha is 1/N
 * Previous average weighted by 1-Alpha
 **/
#define HALL_EFFECT_INV_ALPHA (1 << 2)
#define IR_INV_ALPHA (1 << 2)

#define TURN_RIGHT_THRESHOLD (VOLTS_TO_HEX(2.2))
#define TURN_LEFT_THRESHOLD (VOLTS_TO_HEX(2.8))

#define MAGNET_BLINK_THRESHOLD (VOLTS_TO_HEX(2)) //1.35
#define MAGNET_BLINK_RESET_THRESHOLD (VOLTS_TO_HEX(2.35))

#define MAGNET_SOLID_THRESHOLD (VOLTS_TO_HEX(3.2)) //3
#define MAGNET_SOLID_RESET_THRESHOLD (VOLTS_TO_HEX(2.65))

#define MAGNET_BLINK_FREQUENCY 4

#define SPEED_CHANGE_WAIT 100

typedef enum{
  STOPPED, FORWARD, LEFT, RIGHT
} robot_state_t;


void rampForward(){
  uns16 speed_increment = 1;
  uns16 speed_right = SERVO_RIGHT_STOP;
  uns16 speed_left = SERVO_LEFT_STOP;

  while(speed_right != SERVO_2MS && speed_left != SERVO_1MS) {
    uns16 new_speed = speed_right + speed_increment;
    if (new_speed > SERVO_2MS) {
      speed_right = SERVO_2MS;
    } else {
      speed_right = new_speed;
    }

    new_speed = speed_left - speed_increment;
    if (new_speed < SERVO_1MS) {
      speed_left = SERVO_1MS;
    } else {
      speed_left = new_speed;
    }

    SetRight(speed_right);
    SetLeft(speed_left);
    Delay(SPEED_CHANGE_WAIT);

    speed_increment *= 2;
  }
}

void rampStop(){
  uns16 speed_increment = 1;
  uns16 speed_right = SERVO_2MS;
  uns16 speed_left = SERVO_1MS;

  while(speed_right != SERVO_RIGHT_STOP && speed_left != SERVO_LEFT_STOP) {
    uns16 new_speed = speed_right - speed_increment;
    if (new_speed < SERVO_RIGHT_STOP) {
      speed_right = SERVO_RIGHT_STOP;
    } else {
      speed_right = new_speed;
    }

    new_speed = speed_left + speed_increment;
    if (new_speed > SERVO_LEFT_STOP) {
      speed_left = SERVO_LEFT_STOP;
    } else {
      speed_left = new_speed;
    }

    SetRight(speed_right);
    SetLeft(speed_left);
    Delay(SPEED_CHANGE_WAIT);

    speed_increment *= 2;
  }
}


void main(void) {
  Initialization();

  WaitForButton();

  UseServos

  rampForward();
  LongDelay(8);
  OnLED
  rampStop();
  OffLED
}
