#define ESP_MAX_LEN 50
#define DEVCHAR '0'


// generic JEVOIS message structure
struct esp_msg_t {
  uint8_t type;
  uint8_t id;
  char str[ESP_MAX_LEN];
};

// decoder state
enum esp_state {
  ESP_SYNC = 0,
  ESP_ID,
  ESP_RX_MSG,
  ESP_RX_OK,
  ESP_RX_ERR
};

// similar to jevois struct
struct esp_t {
  enum esp_state state; // decoder state
  char buf[ESP_MAX_LEN]; // temp buffer
  uint8_t idx; // temp buffer index
  struct esp_msg_t msg; // last decoded message
  bool data_available; // new data to report
};

struct esp_t esp;


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
                     /*38*/'$', 'D', '0', DEVCHAR,
                     /*42*/'M', 'A', 'V', 'L',
                     /*46*/'A', 'B', '*', '*',
                     /*50*/'*', '*', '*', '*',
                     /*54*/'*', '*', '*', '*',
                     /*58*/'*', '*', '*', '*',
                     /*62*/'*', '*', '*', '*',
                     /*66*/'*', '*',
                     /*Supported rates*/
                     /*68*/0x01, 0x08, 0x82, 0x84,
                     /*72*/0x8b, 0x96, 0x24, 0x30,
                     /*76*/0x48, 0x6c, 0x03, 0x01,
                     /*80*/0x04
                   };
