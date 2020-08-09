#include <MSP.h>
#include <SoftwareSerial.h>

MSP msp;
SoftwareSerial mspSerial(10, 11); // RX TX
    
void setup() {
  Serial.begin(19200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Start");
  
  mspSerial.begin(19200);
  msp.begin(mspSerial);
}

void loop() {
  msp_status_ex_t status_ex;
  if (msp.request(MSP_STATUS_EX, &status_ex, sizeof(status_ex))) {
    
    uint32_t activeModes = status_ex.flightModeFlags;
    if (activeModes>>MSP_MODE_ARM & 1) 
      Serial.println("ARMED");
    else {
      Serial.println("NOT ARMED");
    }
  }
}
