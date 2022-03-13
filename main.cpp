#include "mbed.h"
#include "rmd_can.h"

#define TARGET_TX_PIN USBTX
#define TARGET_RX_PIN USBRX
static BufferedSerial serial_port(TARGET_TX_PIN, TARGET_RX_PIN, 115200);

CAN can0(PA_11, PA_12, 1000000);
rmd_can l7015;
DigitalIn upbutton(PC_13); // User Button
DigitalOut led(LED1);
bool state = false;
// steps and delay counter
Timer t2;
int counter = 0;

// This program will hold the position of the motor at 0deg (encoder) at start 
// when you press on the button the program will light on the led and rotate the motor 180 deg in 720dps CW
// then after 1 sec will back to 0 deg in 500 dps CCW

// all the function usage is locate at rmd_can.cpp
int main() {
  int8_t id = 1;
  l7015.rmd_can_init(&can0);
  l7015.motor_enable(id);
  l7015.DEBUG_RMD_LIB_MSG = false; //disable the debug uart you can enable if you like 
  while (true) {
    if (!upbutton) {
      state = !state;
    }
    if (state) {
      led = 1;
      t2.start();
      counter++;
      // set posistion to 180 deg (encoder) in 720 dps CW
      l7015.set_single_turn_angle_speed(id, 18000, 720, 0x00);
      // delay 1 sec
      if (counter >= 1002) {
        counter = 0;
        t2.stop();
        t2.reset();
        state = !state;
      }
    } else {
      led = 0;
      l7015.set_single_turn_angle_speed(id, 0, 500, 0x01);
      t2.stop();
    }
    // update status
    // l7015.status_update2(id);
    // print status
    //printf("temp: %d current: %d velocity: %d encoder: %d\r\n", l7015.temp,l7015.torq, l7015.velocity, l7015.encoder);
  }
}
