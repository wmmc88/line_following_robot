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

#define MAGNET_BLINK_THRESHOLD (VOLTS_TO_HEX(2))
#define MAGNET_BLINK_RESET_THRESHOLD (VOLTS_TO_HEX(2.35))

#define MAGNET_SOLID_THRESHOLD (VOLTS_TO_HEX(3))
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

  // declare and seed reading averages with initial readings
  uns8 avg_hall_effect_reading = VOLTS_TO_HEX(2.5);
  uns8 avg_ir_diff_reading = VOLTS_TO_HEX(2.5);

  //current motor speeds
  robot_state_t robot_state = STOPPED;

  //magnet detection
  bit blink_magnet_found = FALSE;
  bit solid_magnet_found = FALSE;

  while (TRUE) {
#if ENABLE_MAGNET_DETECTION
    // compute new avg hall reading
    uns8 temp_hall_effect = avg_hall_effect_reading / HALL_EFFECT_INV_ALPHA;
    avg_hall_effect_reading -= temp_hall_effect;
    temp_hall_effect = AnalogConvert(ADC_HALL_EFFECT) / HALL_EFFECT_INV_ALPHA;
    avg_hall_effect_reading += temp_hall_effect;

    if (avg_hall_effect_reading < MAGNET_BLINK_THRESHOLD && !blink_magnet_found) {
      // Stop robot
      if (robot_state == FORWARD){
        rampStop();
      } else {
        Stop
      }
      // Blink for 7 Seconds
      uns8 blink_cycles;
      for (blink_cycles = 0; blink_cycles < 7 * MAGNET_BLINK_FREQUENCY; blink_cycles++) {
        OnLED
        LongDelay(SECS_TO_LONG_DELAY_COUNTS(1 / (2 * MAGNET_BLINK_FREQUENCY)));
        OffLED
        LongDelay(SECS_TO_LONG_DELAY_COUNTS(1 / (2 * MAGNET_BLINK_FREQUENCY)));
      }
      //turn back on
      rampForward();
      blink_magnet_found = TRUE;
    } else if (avg_hall_effect_reading > MAGNET_SOLID_THRESHOLD && !solid_magnet_found) {
      //stop robot
      if (robot_state == FORWARD){
        rampStop();
      } else {
        Stop
      }
      // Turn on LED for 7 seconds
      OnLED
      LongDelay(SECS_TO_LONG_DELAY_COUNTS(7));
      OffLED
      // turn back on
      rampForward();
      solid_magnet_found = TRUE;
    }

    if (avg_hall_effect_reading > MAGNET_BLINK_RESET_THRESHOLD ) {
      blink_magnet_found = FALSE;
    }

    if (avg_hall_effect_reading < MAGNET_SOLID_RESET_THRESHOLD) {
      solid_magnet_found = FALSE;
    }
#endif

#if ENABLE_LINE_FOLLOWING
    // compute new avg ir reading
    uns8 temp_ir = avg_ir_diff_reading / IR_INV_ALPHA;
    avg_ir_diff_reading -= temp_ir;
    temp_ir = AnalogConvert(ADC_IR_SENSOR) / IR_INV_ALPHA;
    avg_ir_diff_reading += temp_ir;

    if (avg_ir_diff_reading < TURN_RIGHT_THRESHOLD) {
      // Turn Right
      if(robot_state!=RIGHT){
        GoRight
        robot_state=RIGHT;
      }
    } else if (avg_ir_diff_reading > TURN_LEFT_THRESHOLD) {
      // Turn Left
      if(robot_state!=LEFT){
        GoLeft
        robot_state=LEFT;
      }
    } else {
      // Go Straight
      if (robot_state != FORWARD) {
        if (robot_state == STOPPED){
          rampForward();
        } else {
          GoForward
        }
        robot_state = FORWARD;
      }
    }
#endif
  }
}
