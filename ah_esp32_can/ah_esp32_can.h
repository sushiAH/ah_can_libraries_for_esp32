#ifndef AH_ESP32_CAN_H
#define AH_ESP32_CAN_H

#include <Arduino.h>
#include <ah_pid_esp.h>
#include <ESP32-TWAI-CAN.hpp>


struct esp32_can{
    int esp32_id;
    int rx_pin;
    int tx_pin;
};

void init_can(motor_controller* p);

void split_int32(int32_t data,uint8_t* splited_data);

void send_frame(int motor_id,int32_t target);

int get_target_from_frame(CanFrame* rxFrame,int32_t* target);

void write_to_control_table(motor_controller* p,uint8_t table_addr,int32_t data);

void can_receive_task(void* pvParameters);





#endif
