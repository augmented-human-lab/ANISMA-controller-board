#include "Wire.h"

#include <Adafruit_PWMServoDriver.h>

// Init driver board with driver board I2C ID
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x5D);

const uint8_t P_OFF = 0;
const uint8_t P_GND = 1;
const uint8_t P_VCC = 2;


// Driver board pin mapping
uint8_t pinmap_in[8] = {9,10,13,14,6,5,2,1};
uint8_t pinmap_en[8] = {8,11,12,15,7,4,3,0};

uint16_t pwm_on_1[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t pwm_off_1[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t pwm_on_2[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t pwm_off_2[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t pwm_on_3[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t pwm_off_3[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t pwm_on_4[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t pwm_off_4[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

unsigned long lasttime = micros();
byte lastbyte = '0';
int seqdatanum = 0;

// Controller board serial command bytes
const char cmd_byte_start = 'I';
const char cmd_byte_setpin = 'N';
const char cmd_byte_setpower = 'P';
const char cmd_byte_setphase = 'H';
const char cmd_byte_setmode = 'M';
const char cmd_byte_settime = 'T';
const char cmd_byte_setduration = 'D';
const char cmd_byte_end = 'E';
const char cmd_byte_alloff = 'O';
const char cmd_byte_version = 'V';

// Vars for chained commands
uint8_t conf_num  = 0;
uint8_t conf_pin = 0;
uint8_t conf_phase = 0;

uint8_t conf_pwr[32] = {0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0};
                        
uint8_t conf_pin_mode[32] = {P_OFF,P_OFF,P_OFF,P_OFF,P_OFF,P_OFF,P_OFF,P_OFF,
                             P_OFF,P_OFF,P_OFF,P_OFF,P_OFF,P_OFF,P_OFF,P_OFF,
                             P_OFF,P_OFF,P_OFF,P_OFF,P_OFF,P_OFF,P_OFF,P_OFF,
                             P_OFF,P_OFF,P_OFF,P_OFF,P_OFF,P_OFF,P_OFF,P_OFF};
                             
unsigned int conf_duration[32] = {0,0,0,0,0,0,0,0,
                                  0,0,0,0,0,0,0,0,
                                  0,0,0,0,0,0,0,0,
                                  0,0,0,0,0,0,0,0};
                                  
unsigned long conf_turnoff_time[32] = {0,0,0,0,0,0,0,0,
                                      0,0,0,0,0,0,0,0,
                                      0,0,0,0,0,0,0,0,
                                      0,0,0,0,0,0,0,0};

boolean conf_receiving = false;
boolean sequence_started = false;
long sequence_starttime = 0;

const int pwmfrequency = 1500;
const unsigned long pwmfrequency_us = 3000;
int phase_counter = 1;

// Set pwm on-off times for corresponding pin and phase
void setPinPWM(int phase, int pin, uint16_t on, uint16_t off){
  if (phase == 1) {
    pwm_on_1[pin] = on;
    pwm_off_1[pin] = off;
  } else if (phase == 2) {
    pwm_on_2[pin] = on;
    pwm_off_2[pin] = off;
  } else if (phase == 3) {
    pwm_on_3[pin] = on;
    pwm_off_3[pin] = off;
  } else {
    pwm_on_4[pin] = on;
    pwm_off_4[pin] = off;
  } 
}

void setPinPower(int phase, int pin, int mode, int power){
  int pwrval = map(power, 0, 100, 0, 4095);
  if (mode == P_GND) {
    setPinPWM(phase, pinmap_in[pin], 0, 4096);    // Special value for signal fully off.
    setPinPWM(phase, pinmap_en[pin], 0, pwrval);  // Control power via EN PWM signal
  }
  else if (mode == P_VCC) {
    setPinPWM(phase, pinmap_in[pin], 4096, 0);    // Special value for signal fully on.
    setPinPWM(phase, pinmap_en[pin], 0, pwrval);  // Control power via EN PWM signal
  }
  else {
    // Assume P_OFF
    setPinPWM(phase, pinmap_in[pin], 0, 4096);    // Special value for signal fully off.
    setPinPWM(phase, pinmap_en[pin], 0, 4096);    // Special value for signal fully off.
  }
}


void updatePins(int phase){
  pwm.setAllPWMStart();
  for (int i = 0; i<16; i++) {
    if (phase == 1) {
      pwm.setAllPWM(pwm_on_1[i], pwm_off_1[i]);
    } else if (phase == 2) {
      pwm.setAllPWM(pwm_on_2[i], pwm_off_2[i]);
    } else if (phase == 3) {
      pwm.setAllPWM(pwm_on_3[i], pwm_off_3[i]);
    } else {
      pwm.setAllPWM(pwm_on_4[i], pwm_off_4[i]);
    }
  }
  pwm.setAllPWMEnd();
}

void setup() {
  Serial.begin(9600);

  // On serial connection identify as anisma driver controller board with version
  while (!Serial){};
  Serial.println("anisma-v0.1");

  pwm.begin();
  pwm.setPWMFreq(pwmfrequency);
  Wire.setClock(400000);

  // turn everything off
  for (uint8_t pin=0; pin<8; pin++) {
    setPinPower(1, pin, P_OFF, 0);
    setPinPower(2, pin, P_OFF, 0);
    setPinPower(3, pin, P_OFF, 0);
    setPinPower(4, pin, P_OFF, 0);
  }

}

void checkInput(){
  while (Serial.available() > 0) {
    byte inbyte = Serial.read();

    if (inbyte == cmd_byte_start && !conf_receiving) {
      conf_receiving = true;
      conf_num = 0;
      
      for (int i = 0; i<32; i++){
        conf_duration[i] = 0;
      }
    }
    else if (inbyte == cmd_byte_end) {
      conf_receiving = false;
      
      unsigned long curtime = micros()/1000;
      for (int i = 0; i<8; i++){
        for (int p = 0; p<4; p++){
          uint8_t index = 8*p+i;
          
          if (conf_duration[index] > 0) {
            Serial.println("pin");
            setPinPower(p+1, i, conf_pin_mode[index], conf_pwr[index]);
            conf_turnoff_time[index] = curtime + (unsigned long)conf_duration[index];
          }
        }
      }

    }
    else if(inbyte == cmd_byte_alloff){
      // all off      
      for (int i = 0; i<8; i++){
        for (int p = 1; p<5; p++){
          setPinPower(p, i, P_OFF, 0);
        }
      }
      
      // Turn all off immideately
      updatePins(1);
      updatePins(2);
      updatePins(3);
      updatePins(4);
    }

    if (conf_receiving) {
      switch(lastbyte){
        case cmd_byte_setpin:
          conf_num += 1;
          conf_pin = inbyte;
          break;
        case cmd_byte_setphase:
          conf_phase = inbyte-1;
          break;
        case cmd_byte_setpower:
          conf_pwr[8*conf_phase+conf_pin] = inbyte;
          break;
        case cmd_byte_setmode:
          conf_pin_mode[8*conf_phase+conf_pin] = inbyte;
          break;
        case cmd_byte_setduration:
          while (Serial.available() < 1) {} // wait for high byte
          byte hdatabyte = Serial.read();
          conf_duration[8*conf_phase+conf_pin] = ((hdatabyte) << 8)  | (inbyte);
          break;
        case cmd_byte_version:
          Serial.println("anisma-v0.1");
          break;
      }

      lastbyte = inbyte;
    }
  }
}

void handleSequence(){
  // turn off pins after expire time
  unsigned long curtime = micros();
  
  for (uint8_t i = 0; i<8; i++){
    for (uint8_t p = 0; p<4; p++){
      uint8_t index = 8*p+i;
      if ((conf_turnoff_time[index] != 0) && (curtime/1000 >= conf_turnoff_time[index])){
        setPinPower(p+1, i, P_OFF, 0);
        Serial.print("OFF!!! ");Serial.print((curtime/1000));Serial.print(" ");
        Serial.println(conf_turnoff_time[index]);
        conf_pwr[index] = 0;
        conf_pin_mode[index] = P_OFF;
        conf_turnoff_time[index] = 0;
      }
    }
  }

  if (curtime - lasttime >= pwmfrequency_us) {
    updatePins(phase_counter);

    phase_counter += 1;

    if (phase_counter > 4) {
      phase_counter = 1;
    }
    
    lasttime = curtime;
  }
}


void loop() {
  checkInput();
  handleSequence();
}
