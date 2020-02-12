#ifndef ONLY_BS_H
#define ONLY_BS_H

#ifdef __cplusplus
extern "C" {
#endif

#define SELF_ID 1
#define ESP_MAX_LEN 50 // lat,long,alt,bearing = 51 bytes max (28 currently)
#define MAX_DRONES 2   // maximum drones in ESP32's range

typedef enum {
  ACK_FRAME = 0,
  DATA_FRAME
} frame_type_t;


// decoder state
typedef enum {
  ESP_SYNC = 0,
  ESP_DRONE_INFO,
  ESP_DRONE_DATA,
  ESP_ERR_CHK,
  ESP_RX_OK,
  ESP_RX_ERR
} esp_state_t;

typedef struct __attribute__((packed)) {
  float x;
  float y;
  float z;
} vec3f_t;

typedef struct __attribute__((packed)) {
  vec3f_t pos;
  float heading;
  vec3f_t vel;
} drone_data_t;

typedef struct __attribute__((packed)) {
  uint8_t drone_id;
  uint8_t packet_type;
  uint8_t packet_length;
} drone_info_t;

// encapsulated in $ and * 
typedef struct __attribute__((packed)){
  drone_info_t info; 
  drone_data_t data;
} uart_packet_t;

// Beacon Packet buffer
uint8_t packet[] = { 0x80, 0x00, 0x00, 0x00,
                     /*Destination MAC*/
                     /*4*/ 0xff, 0xff, 0xff, 0xff,
                     /*8*/ 0xff, 0xff,
                     /*Source MAC*/
                     /*10*/0x01, 0x02, 0x03, 0x04,
                     /*14*/0x05, 0x06,
                     /*BSS ID*/
                     /*16*/0x01, 0x02, 0x03, 0x04,
                     /*20*/0x05, 0x06, 0xc0, 0x6c,
                     /*Timestamp*/
                     /*24*/0x83, 0x51, 0xf7, 0x8f,
                     /*28*/0x0f, 0x00, 0x00, 0x00,
                     /*32*/0x64, 0x00,/*Beacon interval*/
                     /*34*/0x01, 0x04,/*Capability info*/
                     /* SSID=ELEMENT ID> length> 28 bytes*/
                     /*36*/0x00, 30,
                     /*38*/0x01, 0x01, 0x21, 0x0A,
                     /*42*/0xd7, 0xc3, 0xbf, 0xf4,
                     /*46*/0x1d, 0xad, 0xc3, 0x78,
                     /*50*/0x3e, 0xb7, 0x46, 0x5c,
                     /*54*/0x0f, 0xe2, 0xc3, 0x0a,
                     /*58*/0xd7, 0xc3, 0xbf, 0xf4,
                     /*62*/0x1d, 0xad, 0x78, 0x3e,
                     /*66*/0xb7, 0x46,
                     /*Supported rates*/
                     /*68*/0x01, 0x08, 0x82, 0x84,
                     /*72*/0x8b, 0x96, 0x24, 0x30,
                     /*76*/0x48, 0x6c, 0x03, 0x01,
                     /*80*/0x04
                   };


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
