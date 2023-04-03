#include <iostream>

#include "stdint.h"
#include "stdio.h"
using namespace std;
typedef uint8_t tf_uint8_t;
typedef uint16_t tf_uint16_t;
typedef uint32_t tf_uint32_t;

typedef struct {
  tf_uint8_t boot_mode;
  tf_uint8_t ota_status;
  tf_uint8_t ota_pack_num;
  tf_uint8_t boot_status;
  tf_uint16_t app_crc;
  tf_uint16_t ota_crc;
  tf_uint32_t ota_size;
  tf_uint32_t app_size;
  tf_uint32_t dock_sequence;
  tf_uint32_t robot_sequence;
  tf_uint32_t ota_download_time;
  tf_uint32_t comm_433_id;
  char app_version[16];
  char ota_version[16];
  char bootloader_version[16];
  tf_uint16_t reser[3];
  tf_uint16_t para_crc;
} BootParameters1;
typedef struct {
  tf_uint8_t boot_mode;
  tf_uint8_t ota_status;
  tf_uint8_t ota_pack_num;
  tf_uint8_t boot_status;
  tf_uint16_t app_crc;
  tf_uint16_t ota_crc;
  tf_uint32_t ota_size;
  tf_uint32_t app_size;
  tf_uint32_t dock_sequence;
  tf_uint32_t robot_sequence;
  tf_uint32_t ota_download_time;
  tf_uint32_t comm_433_id;
  char app_version[16];
  char ota_version[16];
  char bootloader_version[14];
  tf_uint8_t is_boot_support_433_frequency_hopping;
  tf_uint8_t boot_fm_version_major;
  tf_uint8_t boot_fm_version_minor;
  tf_uint8_t boot_fm_version_revision;
  tf_uint8_t rf_433_ovfth;
  tf_uint8_t rf_433_step;
  tf_uint16_t rf_433_channel;
  tf_uint16_t para_crc;
} BootParameters2;

int main() {
  BootParameters1 boot1;
  BootParameters2 boot2;
  cout << sizeof(BootParameters1) << "," << sizeof(BootParameters2) << endl;
}