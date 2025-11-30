#include <ah_esp32_can.h>


int esp32_id = 0x010;

void init_can(motor_controller* p){
  
  xTaskCreate(
      can_receive_task,      //タスク関数
      "can",    //タスク名
      4096,        //スタックサイズ
      (void *)p,       //タスクに渡す引数
      2,          //優先度
      NULL);        //タスクハンドル
    
}

void split_int32(int32_t data,uint8_t* splited_data){

  splited_data[0] = (uint8_t)(((data) >> 24) & 0xFF);
  splited_data[1] = (uint8_t)(((data) >> 16) & 0xFF);
  splited_data[2] = (uint8_t)(((data) >> 8) & 0xFF);
  splited_data[3] = (uint8_t)((data)& 0xFF);
}


void send_frame(int motor_id,int32_t target){
  uint8_t packet_length = 8;
  uint8_t packet[8] = {0};
  uint8_t header = 0xAA;
  int32_t send_data_integer = 0;
  uint8_t splited_data[4] = {0};

  CanFrame txFrame;

  send_data_integer = target;
  split_int32(send_data_integer,splited_data);

  txFrame.identifier = esp32_id + motor_id + 4; // Default OBD2 address;
  txFrame.extd = 0;
  txFrame.data_length_code = 4;
  txFrame.data[0] = splited_data[0];
  txFrame.data[1] = splited_data[1];
  txFrame.data[2] = splited_data[2];
  txFrame.data[3] = splited_data[3];

  ESP32Can.writeFrame(txFrame); 
}

int get_target_from_frame(CanFrame* rxFrame,int32_t* target){
    int frame_length = rxFrame->data_length_code;
    
    if(frame_length < 2){
        return 1;
    }
    else if(frame_length == 2){
        *target = rxFrame->data[1];
        return 0;
    }
    else if(frame_length == 3){
        *target = rxFrame->data[1] << 8 | rxFrame->data[2];
        return 0;
    }
    else if(frame_length == 4){
        *target = rxFrame->data[1] << 16 | rxFrame->data[2] << 8 | rxFrame->data[3];
        return 0;
    }
    else if(frame_length == 5){
        *target = rxFrame->data[1] << 24 | rxFrame->data[2] << 16 | rxFrame->data[3]  << 8| rxFrame->data[4];
        return 0;
    }
    return 1;

}

void write_to_control_table(motor_controller* p,uint8_t table_addr,int32_t data){
  if(table_addr == 0){
    p->operating_mode = data;
  }
  else if(table_addr == 1){
    p->goal_pos_int = data;
  }

  else if(table_addr == 2){
    p->goal_vel_int = data;
  }
}


void can_receive_task(void* pvParameters){

  uint8_t table_addr = 0;

  //書き込み用変数
  int32_t operating_mode = 0;
  uint8_t rx_motor_id = 0;
  int32_t target = 0;

  //読み出し用変数　
  int32_t current_pos = 0;
  int32_t current_vel = 0;

  motor_controller* p = (motor_controller*)pvParameters;

  CanFrame rxFrame;


  while(1){
    //受信
    if(ESP32Can.readFrame(&rxFrame,portMAX_DELAY)){
      get_target_from_frame(&rxFrame,&target);
      rx_motor_id = rxFrame.identifier - esp32_id;
      table_addr = rxFrame.data[0];
      //Serial.println(rx_motor_id);

        //共有データ書き込み
      if(xSemaphoreTake(p[rx_motor_id].mutex,portMAX_DELAY) == pdTRUE){
          write_to_control_table(&p[rx_motor_id],table_addr,target);    //アドレスに応じて書き込む
          operating_mode = p[rx_motor_id].operating_mode;  

          if(table_addr == 3){
              current_pos = p[rx_motor_id].current_pos_int;
          }
          if(table_addr == 4){
              current_vel = p[rx_motor_id].current_vel_int;
          }
          xSemaphoreGive(p[rx_motor_id].mutex);
      }

      //送信
      //角度データ送信
      if(table_addr == 3){
        send_frame(rx_motor_id,current_pos);
      }
      //速度度データ送信
      else if(table_addr == 4){
        send_frame(rx_motor_id,current_vel);
      }
    }
  }
}
