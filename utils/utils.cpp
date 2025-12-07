#include <utils.h>

void reset_array(uint8_t* array, int length) {
  for (int i = 0; i < length; i++) {
    array[i] = 0;
  }
}

/**
 * @brief int32をリトルエンディアンで4つのバイト列に変換
 *
 * @param data
 * @param split_data
 */
void from_int32_to_bytes(int32_t data, uint8_t* split_data) {
  split_data[3] = (uint8_t)(((data) >> 24) & 0xFF);
  split_data[2] = (uint8_t)(((data) >> 16) & 0xFF);
  split_data[1] = (uint8_t)(((data) >> 8) & 0xFF);
  split_data[0] = (uint8_t)((data) & 0xFF);
}
