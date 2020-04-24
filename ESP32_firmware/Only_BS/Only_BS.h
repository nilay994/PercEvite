#ifndef ONLY_BS_H
#define ONLY_BS_H

#ifdef __cplusplus
extern "C" {
#endif

#define SELF_ID 2
#define ESP_MAX_LEN 50 // lat,long,alt,bearing = 51 bytes max (28 currently)
#define LEN_802_11 83

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
} vec2f_t;

typedef struct __attribute__((packed)) {
  vec2f_t pos;
  float heading;
  float vel;
} drone_data_t;

typedef struct __attribute__((packed)) {
  uint8_t drone_id;
  uint8_t packet_type;
  uint8_t packet_length;
} drone_info_t;

// encapsulated in $ and * 
typedef struct __attribute__((packed)) {
  drone_info_t info; 
  drone_data_t data;
} uart_packet_t;

// Beacon Packet buffer
uint8_t packet802[LEN_802_11] = { 0x80, 0x00, 0x00, 0x00,
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
                     /*36*/0x00, 32,
                     /*38*/'$', 178, SELF_ID, 'E', 
                     /*42*/ 'S', 'P', 'M', 'A',
                     /*46*/'V', 'L', 'A', 'B',
                     /*50*/'*', '*', '*', '*',
                     /*54*/'*', '*', '*', '*',
                     /*58*/'*', '*', '*', '*',
                     /*62*/'*', '*', '*', '*',
                     /*66*/'*', '*', '*', '*',
                     /*Supported rates*/
                     /*70*/0x01, 0x08, 0x82, 0x84,
                     /*74*/0x8b, 0x96, 0x24, 0x30,
                     /*80*/0x48, 0x6c, 0x03, 0x01,
                     /*81*/0x04
                   };


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
