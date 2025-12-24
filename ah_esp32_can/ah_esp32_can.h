#ifndef AH_ESP32_CAN_H
#define AH_ESP32_CAN_H

#include <Arduino.h>
#include <ah_control_table.h>
#include <ah_pid_esp.h>
#include <utils.h>

#include <ESP32-TWAI-CAN.hpp>


void init_can(int esp32_id,motor_controller* ctrl);

void send_frame(int tx_frame_id, int32_t target);

int read_target_from_frame(CanFrame* rxFrame, int32_t* target);

int write_to_control_table(motor_controller* ctrl, uint8_t table_addr,
                           int32_t data);
int32_t read_current_data(motor_controller* ctrl, uint8_t table_addr);

void can_receive_task(void* pvParameters);

#endif
