
#include "Haptic_DRV2605.h"
Haptic_DRV2605 haptic;             // Basic I2C instance - only SDA/SCL pins used
int           waveform = 0;        // current waveform
int           waveforms_max = 0;   // max waveforms available (we'll ask...)
int           select_mode = 0; 

//Custom definitions for PCA9548A I2C Data Bus Multiplexer  
#define PCAADDR 0x70      //PCA9548A I2C Data bus multiplexer default address 

//Prototypes 
void PCAselect(uint8_t i);
void switch_LRA();





void setup() {
  Serial.begin(115200);
   while (!Serial) {
    ; // wait for serial port to connect. Needed to see startup messages...
  }

  Serial.println("Haptic: LRA Driver Basic Tests");
  Serial.println("Haptic: Connect I2C pins to WIRE Pins (SDA, SCL)");

  PCAselect(6); //Call PCAselect for device at SD2

  // initialize the Haptic controller
    if (haptic.begin() != HAPTIC_SUCCESS) {
      Serial.println("Haptic: Driver Error: DRV2605L Device not found - check your I2C connections?");
  } else {
      // okay correct chip, let's initialize it
      Serial.println("Haptic: DeviceID = DRV2605L @ Address 0x5A was found! ");
      // config the DRV2605 chip
      haptic.setActuatorType(LRA);              // pick an actuator type
      haptic.setMode(REGISTER_MODE);            // haptic effects triggered by I2C register write 
      waveforms_max = haptic.getWaveforms();    // how many waveforms available?
  }


PCAselect(2);
 if (haptic.begin() != HAPTIC_SUCCESS) {
      Serial.println("Haptic: Driver Error: DRV2605L Device not found - check your I2C connections?");
  } else {
      // okay correct chip, let's initialize it
      Serial.println("Haptic: DeviceID = DRV2605L @ Address 0x5A was found! ");
      // config the DRV2605 chip
      haptic.setActuatorType(LRA);              // pick an actuator type
      haptic.setMode(REGISTER_MODE);            // haptic effects triggered by I2C register write 
      waveforms_max = haptic.getWaveforms();    // how many waveforms available?
  }



}

void loop() {
 
  //Switch between the two LRAs
  switch_LRA();
 
 // loop through all the waveforms
  Serial.print("Waveform #");                   // which waveform
  Serial.println(waveform);
  haptic.setWaveform(0, waveform);              // set the first sequence
  haptic.setWaveform(1, 0);                     // end the sequence
  haptic.goWait();	
 			                      // play the waveform
  delay(100);	
	                          // wait for a while
  waveform++;	
  				                          // next waveform
  if (waveform >= 3) waveform = 0;
}



void switch_LRA(){

   switch(select_mode){

    case 0:
      PCAselect(2);
      select_mode = 1;
      break;

    case 1:
      PCAselect(6);
      select_mode = 0;
      break; 

  }
}



void PCAselect(uint8_t i){

if(i > 7) return;
Wire.beginTransmission(PCAADDR);
Wire.write(1 << i);
Wire.endTransmission();

}
