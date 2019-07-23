#include "mte220.c"

void main(void) {
  Initialization();
  while (1) {
    OnLED
    LongDelay(1);
    OffLED
    LongDelay(1);
  }
}
