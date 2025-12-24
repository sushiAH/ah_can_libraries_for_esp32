/**
 * @file ah_can_libraries_for_esp32.cpp
 * @brief
 * pcからcan_frameを受け取って、motor_controller構造体内の共有変数に書き込む。
 * table_addrに応じてモーターの現在値を送信する
 * rx_can_frame_id = esp32_id + motor_id
 * tx_can_frame_id = esp32_id + motor_id + 4
 */

#include <ah_esp32_can.h>

#include <ESP32-TWAI-CAN.hpp>


/**
 * @brief freertos can taskを立ち上げる
 *
 * @param ctrl
 */
void init_can(int esp32_id,motor_controller* ctrl) {
  // Everything is defaulted so you can just call .begin() or .begin(TwaiSpeed)
  // Calling begin() to change speed works, it will disable current driver first
  

  unsigned long acceptance_code = (esp32_id << 21);
  unsigned long acceptance_mask = (0x003 << 21) | 0x1FFFFF;

  // hardware can_identifier filtering
  twai_filter_config_t f_config = {.acceptance_code = acceptance_code,
                                   .acceptance_mask = acceptance_mask,
                                   .single_filter = true};

  ESP32Can.begin(TWAI_SPEED_1000KBPS, 5, 4, 10, 10, &f_config, nullptr,
                 nullptr);

  // ESP32Can.begin(ESP32Can.convertSpeed(1000), 5, 4, 10, 10);

  xTaskCreate(can_receive_task,  // タスク関数
              "can",             // タスク名
              4096,              // スタックサイズ
              (void*)ctrl,       // タスクに渡す引数
              2,                 // 優先度
              NULL);             // タスクハンドル
}

/**
 * @brief can_frameを送信する
 *
 * @param motor_id
 * @param send_data　送信するデータ
 */
void send_frame(int tx_frame_id, int32_t send_data_integer) {
  uint8_t split_data[4] = {0};

  CanFrame txFrame;

  from_int32_to_bytes(send_data_integer, split_data);

  txFrame.identifier = tx_frame_id;  // Default OBD2 address;
  txFrame.extd = 0;
  txFrame.data_length_code = 4;
  txFrame.data[0] = split_data[0];
  txFrame.data[1] = split_data[1];
  txFrame.data[2] = split_data[2];
  txFrame.data[3] = split_data[3];

  ESP32Can.writeFrame(txFrame);
}

/**
 * @brief
 * 受信したcan_frameから、リトルエンディアンで、共有変数に書き込むデータを読み出す
 * @param rxFrame can receive frame pointer
 * @param target 共有変数に書き込む値
 * @return err
 * より汎用性を高くできる
 */
int read_target_from_frame(CanFrame* rxFrame, int32_t* target) {
  int frame_length = rxFrame->data_length_code;

  if (frame_length < 2) {
    return 1;

  } else if (frame_length == 2) {
    *target = rxFrame->data[1];
    return 0;

  } else if (frame_length == 3) {
    *target = rxFrame->data[2] << 8 | rxFrame->data[1];
    return 0;

  } else if (frame_length == 4) {
    *target = rxFrame->data[3] << 16 | rxFrame->data[2] << 8 | rxFrame->data[1];
    return 0;

  } else if (frame_length == 5) {
    *target = rxFrame->data[4] << 24 | rxFrame->data[3] << 16 |
              rxFrame->data[2] << 8 | rxFrame->data[1];
    return 0;
  }

  return 1;
}

/**
 * @brief 受け取ったデータを,table_addrに応じて、共有変数に書き込む
 *
 * @param p
 * @param table_addr enumで定義されたテーブルのアドレス
 * @param data 書き込むデータ
 */
int write_to_control_table(motor_controller* ctrl, uint8_t table_addr,
                           int32_t data) {
  if (table_addr == OPERATING_MODE_ADDR) {
    ctrl->operating_mode = data;
    return 0;
  }

  else if (table_addr == GOAL_POS_ADDR) {
    ctrl->goal_pos_int = data;
    return 0;
  }

  else if (table_addr == GOAL_VEL_ADDR) {
    ctrl->goal_vel_int = data;
    return 0;
  }

  else if (table_addr == POS_P_ADDR) {
    ctrl->POS.PID.kp = data / 1000.00;
    return 0;
  }

  else if (table_addr == POS_I_ADDR) {
    ctrl->POS.PID.ki = data / 1000.00;
    return 0;
  }

  else if (table_addr == POS_D_ADDR) {
    ctrl->POS.PID.kd = data / 1000.00;
    return 0;
  }

  else if (table_addr == VEL_P_ADDR) {
    ctrl->VEL.PID.kp = data / 1000.00;
    return 0;
  }

  else if (table_addr == VEL_I_ADDR) {
    ctrl->VEL.PID.ki = data / 1000.00;
    return 0;
  }

  else if (table_addr == VEL_D_ADDR) {
    ctrl->VEL.PID.kd = data / 1000.00;
    return 0;
  }

  else if (table_addr == GOAL_PWM_ADDR) {
    ctrl->goal_pwm = data;
    return 0;
  }

  else if (table_addr == AIR_ADDR) {
    ctrl->air_val = data;
    return 0;
  }

  else {
    return table_addr;
  }
}

/**
 * @brief table_addrに応じて、読み出した現在値を返す
 *
 * @param ctrl
 * @param table_addr コントロールテーブルアドレス
 * @param motor_id モーターID
 * @return 現在値
 */
int32_t read_current_data(motor_controller* ctrl, uint8_t table_addr,
                          uint8_t motor_id) {
  if (table_addr == CURRENT_POS_ADDR) {
    return ctrl->current_pos_int;
  }

  else if (table_addr == CURRENT_SPEED_ADDR) {
    return ctrl->current_vel_int;
  }

  return 0;  // 未定義のアドレス
}

/**
 * @brief freertos can reaceive task
 *
 * @param pvParameters motor_controller array pointer
 */
void can_receive_task(void* pvParameters) {
  uint8_t table_addr = 0;

  // 書き込み用変数
  uint8_t rx_motor_id = 0;
  int32_t target = 0;

  // 読み出し用変数　
  int32_t current_data = 0;

  motor_controller* ctrl = (motor_controller*)pvParameters;

  CanFrame rxFrame;


  while (1) {
    // 受信
    //wait one second
    if (ESP32Can.readFrame(&rxFrame, portMAX_DELAY)) {

      read_target_from_frame(&rxFrame, &target);
      rx_motor_id = rxFrame.identifier & 0x00F;
      table_addr = rxFrame.data[0];

      // 共有データ書き込み
      if (xSemaphoreTake(ctrl[rx_motor_id].mutex, portMAX_DELAY) == pdTRUE) {
        // table_addrに応じて値を書き込む
        write_to_control_table(&ctrl[rx_motor_id], table_addr, target);

        // table_addrに応じて値を読み出す
        current_data =
            read_current_data(&ctrl[rx_motor_id], table_addr, rx_motor_id);

        xSemaphoreGive(ctrl[rx_motor_id].mutex);
      }

      // 送信
      if (table_addr == CURRENT_SPEED_ADDR || table_addr == CURRENT_POS_ADDR) {
        int tx_frame_id = rxFrame.identifier + 4;
        send_frame(tx_frame_id, current_data);
      }
    }
  }
}
