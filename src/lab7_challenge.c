#include "mte220.c"

#define ENABLE_LINE_FOLLOWING 1
#define ENABLE_MAGNET_DETECTION 1

#define TRUE 1
#define FALSE 0

#define VOLTS_TO_HEX(X) ((uns16)(X * 1024 / 5) >> 2)
#define SECS_TO_LONG_DELAY_COUNTS(X) ((uns16)X * 8)

/**Exponential Moving Average
 * Newest value is weighted by Alpha, where Alpha is 1/N
 * Previous average weighted by 1-Alpha
 **/
#define HALL_EFFECT_INV_ALPHA (1 << 1)
#define IR_INV_ALPHA (1 << 3)

#define TURN_RIGHT_THRESHOLD (VOLTS_TO_HEX(2))
#define TURN_LEFT_THRESHOLD (VOLTS_TO_HEX(3))

#define MAGNET_BLINK_THRESHOLD (VOLTS_TO_HEX(2))
#define MAGNET_BLINK_RESET_THRESHOLD (VOLTS_TO_HEX(2.35))

#define MAGNET_SOLID_THRESHOLD (VOLTS_TO_HEX(3))
#define MAGNET_SOLID_RESET_THRESHOLD (VOLTS_TO_HEX(2.65))

#define SEARCHING_FOR_MAGNET 0
#define STOPPING_FOR_MAGNET 1
#define COOLING_DOWN_FROM_MAGNET 2

#define MAGNET_BLINK_FREQUENCY 8

#define SPEED_CHANGE_WAIT 40

void main(void) {
  Initialization();

  WaitForButton();

  UseServos

  // declare and seed reading averages with initial readings
  uns8 avg_hall_effect_reading = VOLTS_TO_HEX(2.5);
  uns8 avg_ir_diff_reading = VOLTS_TO_HEX(2.5);

  // speed-ramping variables
  uns16 right_target_speed = SERVO_RIGHT_STOP;
  uns16 right_prev_target_speed = SERVO_RIGHT_STOP;
  uns16 right_current_speed = SERVO_RIGHT_STOP;
  uns16 right_speed_increment = 1;
  uns16 left_target_speed = SERVO_LEFT_STOP;
  uns16 left_prev_target_speed = SERVO_LEFT_STOP;
  uns16 left_current_speed = SERVO_LEFT_STOP;
  uns16 left_speed_increment = 1;

  //magnet detection
  bit blink_magnet_found = FALSE;
  bit solid_magnet_found = FALSE;
  uns8 magnet_state = SEARCHING_FOR_MAGNET;

  while (TRUE) {
#if ENABLE_MAGNET_DETECTION
    // compute new avg hall reading
    uns8 temp_hall_effect = avg_hall_effect_reading / HALL_EFFECT_INV_ALPHA;
    avg_hall_effect_reading -= temp_hall_effect;
    temp_hall_effect = AnalogConvert(ADC_HALL_EFFECT) / HALL_EFFECT_INV_ALPHA;
    avg_hall_effect_reading += temp_hall_effect;

    if (avg_hall_effect_reading < MAGNET_BLINK_THRESHOLD && magnet_state != COOLING_DOWN_FROM_MAGNET) {
      if(right_current_speed == SERVO_RIGHT_STOP && left_current_speed == SERVO_LEFT_STOP) {
        // Blink for 7 Seconds
        uns8 blink_cycles;
        for (blink_cycles = 0; blink_cycles < 7 * MAGNET_BLINK_FREQUENCY; blink_cycles++) {
          OnLED
          LongDelay(SECS_TO_LONG_DELAY_COUNTS(1 / (2 * MAGNET_BLINK_FREQUENCY)));
          OffLED
          LongDelay(SECS_TO_LONG_DELAY_COUNTS(1 / (2 * MAGNET_BLINK_FREQUENCY)));
        }
        magnet_state = COOLING_DOWN_FROM_MAGNET;
      } else {
        blink_magnet_found = TRUE;
        magnet_state = STOPPING_FOR_MAGNET;
        right_target_speed = SERVO_RIGHT_STOP;
        left_target_speed = SERVO_LEFT_STOP;
      }
    } else if (avg_hall_effect_reading > MAGNET_SOLID_THRESHOLD && magnet_state != COOLING_DOWN_FROM_MAGNET) {
      if(right_current_speed == SERVO_RIGHT_STOP && left_current_speed == SERVO_LEFT_STOP) {
        // Turn on LED for 7 seconds
        OnLED
        LongDelay(SECS_TO_LONG_DELAY_COUNTS(7));
        OffLED
        magnet_state = COOLING_DOWN_FROM_MAGNET;
      } else {
        solid_magnet_found = TRUE;
        magnet_state = STOPPING_FOR_MAGNET
        right_target_speed = SERVO_RIGHT_STOP;
        left_target_speed = SERVO_LEFT_STOP;
      }
    } else if (blink_magnet_found && avg_hall_effect_reading > MAGNET_BLINK_RESET_THRESHOLD ) {
      blink_magnet_found = FALSE;
      magnet_state = SEARCHING_FOR_MAGNET;
    } else if (solid_magnet_found && avg_hall_effect_reading < MAGNET_SOLID_RESET_THRESHOLD) {
      solid_magnet_found = FALSE;
      magnet_state = SEARCHING_FOR_MAGNET;
    }

#endif

#if ENABLE_LINE_FOLLOWING
    if(magnet_state != STOPPING_FOR_MAGNET) {
      // compute new avg ir reading
      uns8 temp_ir = avg_ir_diff_reading / IR_INV_ALPHA;
      avg_ir_diff_reading -= temp_ir;
      temp_ir = AnalogConvert(ADC_IR_SENSOR) / IR_INV_ALPHA;
      avg_ir_diff_reading += temp_ir;

      if (avg_ir_diff_reading < TURN_RIGHT_THRESHOLD) {
        // Turn Right
        right_target_speed = SERVO_RIGHT_STOP;
        left_target_speed = SERVO_1MS;
      } else if (avg_ir_diff_reading > TURN_LEFT_THRESHOLD) {
        // Turn Left
        right_target_speed = SERVO_2MS;
        left_target_speed = SERVO_LEFT_STOP;
      } else {
        // Go Straight
        right_target_speed = SERVO_2MS;
        left_target_speed = SERVO_1MS;
      }
    }

    if(right_target_speed!=right_prev_target_speed){
      right_speed_increment = 1;
      right_prev_target_speed = right_target_speed;
    }

    if(left_target_speed!=left_prev_target_speed){
      left_speed_increment = 1;
      left_prev_target_speed = left_target_speed;
    }

    if(right_current_speed < right_target_speed){
      uns16 temp_speed = right_current_speed + right_speed_increment;
      if(temp_speed > SERVO_2MS){
        right_current_speed = SERVO_2MS;
      } else {
        right_current_speed += right_speed_increment;
        right_speed_increment *= 2;
      }
      Delay(SPEED_CHANGE_WAIT);
    } else if (right_current_speed > right_target_speed){
      uns16 temp_speed = right_current_speed - right_speed_increment;
      if(temp_speed < SERVO_RIGHT_STOP){
        right_current_speed = SERVO_RIGHT_STOP;
      } else {
        right_current_speed -= right_speed_increment;
        right_speed_increment *= 2;
      }
      Delay(SPEED_CHANGE_WAIT);
    }
    SetRight(right_current_speed);

    if(left_current_speed < left_target_speed){
      uns16 temp_speed = left_current_speed + left_speed_increment;
      if(temp_speed > SERVO_LEFT_STOP){
        left_current_speed = SERVO_LEFT_STOP;
      } else {
        left_current_speed += left_speed_increment;
        left_speed_increment *= 2;
      }
      Delay(SPEED_CHANGE_WAIT);
    } else if (left_current_speed > left_target_speed){
      uns16 temp_speed = left_current_speed - left_speed_increment;
      if(temp_speed < SERVO_1MS){
        left_current_speed = SERVO_1MS;
      } else {
        left_current_speed -= left_speed_increment;
        left_speed_increment *= 2;
      }
      Delay(SPEED_CHANGE_WAIT);
    }
    SetLeft(left_current_speed);
#endif
  }
}
