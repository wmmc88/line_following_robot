#include "mte220.c"

#define ENABLE_LINE_FOLLOWING 1
#define ENABLE_MAGNET_DETECTION 1

#define TRUE 1
#define FALSE 0

#define VOLTS_TO_HEX(X) ((X * 1024 / 5) >> 2)
#define SECS_TO_LONG_DELAY_COUNTS(X) ((uns16)X * 8)

/**Exponential Moving Average
 * Newest value is weighted by Alpha, where Alpha is 1/N
 * Previous average weighted by 1-Alpha
 **/
#define HALL_EFFECT_INV_ALPHA (1 << 4)
#define IR_INV_ALPHA (1 << 4)

#define TURN_RIGHT_THRESHOLD (VOLTS_TO_HEX(2))
#define TURN_LEFT_THRESHOLD (VOLTS_TO_HEX(3))

#define MAGNET_BLINK_THRESHOLD (VOLTS_TO_HEX(2))
#define MAGNET_LED_ON_THRESHOLD (VOLTS_TO_HEX(3))

#define MAGNET_BLINK_FREQUENCY 8

void main(void) {
  Initialization();

  WaitForButton();

  UseServos

  // declare seed reading averages with initial readings
  uns8 avg_hall_effect_reading = AnalogConvert(ADC_HALL_EFFECT);
  uns8 avg_ir_diff_reading = AnalogConvert(ADC_IR_SENSOR);

  while (TRUE) {
#if ENABLE_MAGNET_DETECTION
    // compute new avg hall reading
    uns8 temp_hall_effect = avg_hall_effect_reading / HALL_EFFECT_INV_ALPHA;
    avg_hall_effect_reading -= temp_hall_effect;
    temp_hall_effect = AnalogConvert(ADC_HALL_EFFECT) / HALL_EFFECT_INV_ALPHA;
    avg_hall_effect_reading += temp_hall_effect;

    if (avg_hall_effect_reading < MAGNET_BLINK_THRESHOLD) {
      // Blink for 7 Seconds
      uns8 blink_cycles;
      for (blink_cycles = 0; blink_cycles < 7 * MAGNET_BLINK_FREQUENCY; blink_cycles++) {
        OnLED
        LongDelay(SECS_TO_LONG_DELAY_COUNTS(1 / (2 * MAGNET_BLINK_FREQUENCY)));
        OffLED
        LongDelay(SECS_TO_LONG_DELAY_COUNTS(1 / (2 * MAGNET_BLINK_FREQUENCY)));
      }
    } else if (avg_hall_effect_reading > MAGNET_LED_ON_THRESHOLD) {
      // Turn on LED for 7 seconds
      OnLED
      LongDelay(SECS_TO_LONG_DELAY_COUNTS(7));
      OffLED
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
      LeftServoOn
      RightServoOff
    } else if (avg_ir_diff_reading > TURN_LEFT_THRESHOLD) {
      // Turn Left
      RightServoOn
      LeftServoOff
    } else {
      // Go Straight
      BothServosOn
    }
#endif
  }
}
