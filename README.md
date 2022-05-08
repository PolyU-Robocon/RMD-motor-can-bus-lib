# RMD-motor-can-bus-lib
Basic Mbed lib for all RMD motor with their CAN BUS (v1.61) protocal, Using Mbed OS 6.13 to compile and tested
![image](https://user-images.githubusercontent.com/45313904/167299222-0d6b70af-4132-4fcf-a85d-0a89c95d0efe.png)


# Function 
```cpp
void rmd_can_init(CAN *_CAN);
void read_pid(int8_t id);
void read_acc_pid_data(int8_t id);
void read_encoder(int8_t id);
void read_global_angle(int8_t id);
void read_angle(int8_t id);

void motor_disable(int8_t id);
void motor_enable(int8_t id);
void clear_error(int8_t id);

void status_update1(int8_t id);
void status_update2(int8_t id);
void status_update3(int8_t id);

void set_zero(int8_t id);
void set_torq(int8_t id,int16_t torq);
void set_velocity(int8_t id,int32_t velocity);
void set_position(int8_t id,int32_t pos);
void set_position_speed(int8_t id,int32_t pos,uint16_t profile_speed);
void set_single_turn_angle(int8_t id,uint16_t angle, uint8_t dir);
void set_single_turn_angle_speed(int8_t id,uint16_t angle, uint16_t profile_speed, uint8_t dir);
void set_pid_RAM(int8_t id,int8_t p_kp, int8_t p_ki, int8_t v_kp, int8_t v_ki, int8_t i_kp, int8_t i_ki);
void set_pid_ROM(int8_t id,int8_t p_kp, int8_t p_ki, int8_t v_kp, int8_t v_ki, int8_t i_kp, int8_t i_ki);
void set_acc_RAM(int8_t id,int32_t Accel);
void set_encoder_offset(int8_t id,int16_t pos_offset);
```
* all function are just like what it means
* the range and basic explaination is locate at rmd_can.cpp and main.cpp
* check the CAN BUS protocal doc in Doc folder for compelete details

# Documentation and user guide 
located at Doc folder

# Tested Model 
1. RMD L 7015 
2. RMD L 9015/9010
3. RMD X8 Series
 
# Test setup 
1. Connect the stm32F446RE to a can tranceiver (TJA1050) via PA11 and PA12
2. Connect the can bus wire to the rmd can bus wire
3. connect you board to host pc
4. Open a new project in Mbed Studio and copy all the file in the repo to your project folder (copy the code of main.cpp or overwwrite the original one with the main.cpp in this repo) 
5. make sure the mbed os verison is 6.13 and the target is set to F446RE
6. compile the program and flash it to your board   
7. Power up the motor with 24v dc 
8. press the button once and the motor will rotate 180 deg CW 
9. the motor will return 0 deg CCW

# Update 
13-3-2022 update the basic function that enough to use, the function not compelete yet
