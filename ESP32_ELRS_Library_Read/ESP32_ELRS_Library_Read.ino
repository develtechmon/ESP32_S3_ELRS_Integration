
#include "ELRS_CRSF.h"
ELRS_CRSF elrs(5, 6);  // RX pin 5, TX pin 6
void setup() {
    Serial.begin(115200);
    elrs.enableDebug(true);
    elrs.begin();
  }
  
  void loop() {
    elrs.update();
    
    if (elrs.isConnected()) {
      uint16_t throttle = elrs.getChannel(1);
      int16_t roll_percent = elrs.getChannelPercent(2);
      
      // Or print all channels
      elrs.printChannelLine();
    }
  }
 