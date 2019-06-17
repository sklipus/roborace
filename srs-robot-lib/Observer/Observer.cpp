#include <Arduino.h>
#include "Observer.h"

Observer::Observer(void (*fn) () ) {
  methodToExecute = fn;
  lastTime = 0;
}


int8_t Observer::doCode(uint32_t interwal, uint32_t execTime) {
  uint32_t currentTime = millis();
  if ( (lastTime + interwal) < currentTime ) {
    // call custom method
    lastTime = currentTime;
    (*methodToExecute)();
    // todo millis is too long we should run this only in debug mode
    if (currentTime - millis() > execTime) {
      return -1; // throw exception
    }
    return 1;
  }
  return 0;
}
