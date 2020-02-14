#include <WiFi.h>
#include "esp_wifi.h"
#include "Only_BS.h"
#include "wsled.h"

// #define DBG

// esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);

drone_data_t dr_data[MAX_DRONES];
uint8_t esp_state = ESP_SYNC;
uint8_t channel = 0;

void random_mac()
{
  // Randomize SRC MAC
  packet[10] = packet[16] = random(256);
  packet[11] = packet[17] = random(256);
  packet[12] = packet[18] = random(256);
  packet[13] = packet[19] = random(256);
  packet[14] = packet[20] = random(256);
  packet[15] = packet[21] = random(256);
}

#define DEFAULT_SCAN_LIST_SIZE 10
uint16_t number = DEFAULT_SCAN_LIST_SIZE;
wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
char ssidsdk[32] = {0};
wifi_scan_config_t config;

void scan(uint8_t ch, uint8_t Ts)
{
  int numSsid = WiFi.scanNetworks(false, true, true, Ts, ch);

  /* scan for other drone IDs */
  for (int j = 0; j < numSsid; j++) {
    // want to read all the 32B
    uint8_t ssidstr[32] = {0};

    // MAJOR BUG! suspecting wifi.ssid terminating when ssid char = 0x00
    // Hacky patch @ .arduino15/packages/esp32/hardware/esp32/1.0.4/libraries/WiFi/src
    // switch to esp-idf please!! And force promiscuous mode!! 
    // WiFi.SSID(j).toCharArray(ssidstr_tmp, 32);
    
    WiFi.SSID(j, ssidstr);
    #ifdef DBG
    Serial.print(j);
    Serial.print("->");
    for (int iwer = 0; iwer < 32; iwer ++) {
      Serial.print(ssidstr[iwer], HEX);
    }
    #endif

    /* print other drones location but with $ appended and CR/LN at the end of packet */
    if (ssidstr[0] == '$') {
      esp_send_string(&ssidstr[1], sizeof(uart_packet_t));
    }
  }
}

// make this my ssid, can only be 32B long
void broadcastSSID() {
  // fixed header "$"
  random_mac();
  for (int c = 1; c < 15; c++) {
    packet[80] = c;    
    esp_wifi_set_channel(c, WIFI_SECOND_CHAN_NONE);
    esp_wifi_80211_tx(WIFI_IF_AP, packet, LEN_802_11, false);
    delay(1);
  } //14 channels
  delay(8);
}

// tx: finally send a hex array to esp32
static uint8_t esp_send_string(uint8_t *s, uint8_t len) {

  // augment start bytes
  Serial.write('$');
  Serial.write(178);

  uint8_t checksum = 0;

  // maximum of 255 bytes
  uint8_t i = 0;
  for (i = 0; i < len; i ++) {
    Serial.write(s[i]);
    
    // TODO: remove when full checksum
    if (i > 2) {
      checksum += s[i];
    }
  }
  #ifdef DBG
  char tmpstr[30] = {0};
  sprintf(tmpstr, "appended checksum while bbp tx: 0x%02x\n", checksum);
  Serial.print(tmpstr);
  #endif 
  
  Serial.write(checksum);

  return (i+3);
}

// init: clear all data for all drones
static void clear_drone_status(void) {
  for (uint8_t id = 0; id < MAX_DRONES; id++) {
    // initialize at tropical waters of eastern Altanic ocean, facing the artic
    dr_data[id].pos.x = 0;
    dr_data[id].pos.y = 0;
    dr_data[id].pos.z = 0;
    dr_data[id].heading = 0;
    dr_data[id].vel.x = 0;
    dr_data[id].vel.y = 0;
    dr_data[id].vel.z = 0;
  }
}

void setup() {
  // reset uart state machine 
  esp_state = ESP_SYNC;
  
  Serial.begin(115200);
  Serial.setTimeout(100);
  WiFi.mode(WIFI_AP_STA);

  // Set channel
  channel = random(1, 14);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_max_tx_power(78);

  // Select external antenna
  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);

  // Init RGB LED
  FastLED.addLeds<WS2812B, LED_WS2812B, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(32);
  leds[0] = CRGB::Green;
  FastLED.show();

  clear_drone_status();
  delay(1000);  

  // Serial.println("DEVICE: " + DEVCHAR);
}

// rx: print struct received after checksum match
static void print_drone_data_struct(drone_data_t *dat) {
  char tmpstr[200] = {0};
	sprintf(tmpstr, "dat->pos.x: %f, dat->pos.y: %f, dat->pos.z: %f\n"
				 "dat->heading: %f\n"
				 "dat->vel.x: %f, dat->vel.y: %f, dat->vel.z: %f\n",
					dat->pos.x, dat->pos.y,	dat->pos.z,
					dat->heading,
					dat->vel.x,	dat->vel.y, dat->vel.z);
  Serial.println(tmpstr);
}


// rx: receive drone_data from struct to a string
static void rx_struct(drone_data_t *dr_dat, uint8_t* buf) {
	memcpy(dr_dat, buf, sizeof(drone_data_t));
	#ifdef DBG
	print_drone_data_struct(dr_dat);
  #endif
  // rx_success indicate
  static bool toggle = 0;
  toggle = !toggle;
  if (toggle) {
      leds[0] = CRGB::Red;
  } else {
      leds[0] = CRGB::Black;
  }
  FastLED.show(); 
}

// rx: parse data and switch state machines
uint8_t databuf[ESP_MAX_LEN] = {0};
void esp_parse(uint8_t c) {
  
  static uint8_t byte_ctr = 0;
  static uint8_t drone_id = 0;
  static uint8_t packet_length = 0;
  static uint8_t packet_type = 0;
  static uint8_t checksum = 0;
  static uint8_t prev_char = 0;

  #ifdef DBG
  char tmstr[40] = {0};
  sprintf(tmstr, "esp_state: %d, char rxed: 0x%02x\n", esp_state, c);
  Serial.println(tmstr);
  #endif

  switch (esp_state) {
    case ESP_SYNC: {

      /* first char, sync string */
      if (c == '$') {
        byte_ctr = byte_ctr + 1;
      }

      /* second char: are you really the start of the packet? */
      if ((byte_ctr == 1) && (c == 178)) {
        byte_ctr = byte_ctr + 1;
        esp_state = ESP_DRONE_INFO;
      }
    } break;

    case ESP_DRONE_INFO: {
      if (byte_ctr == 2) {
        /* take note of packet length */
        drone_id = c;
        byte_ctr = byte_ctr + 1;
      } else if (byte_ctr == 3) {
        /* take note of packet type */
        packet_type = c;
        byte_ctr = byte_ctr + 1;
      } else if (byte_ctr == 4) {
        /* take note of drone ID */
        packet_length = c;
        byte_ctr = byte_ctr + 1;

        // info frame populated!! 
        // printf("[uart] packet_length: %d, packet_type: %d, drone_id: %d\n", packet_length, packet_type, drone_id);

        if (packet_type == ACK_FRAME && packet_length == 4) {
          // TODO: esp received ssid change signal and sent you ack, 
          // indicate that on bool pprz esp ping? 
          esp_state = ESP_RX_OK;
          byte_ctr = 0;
        }

        /* packet length will always be shorter than padded struct, create some leeway */
        else if ((packet_type == DATA_FRAME) && (packet_length >= (sizeof(drone_data_t)-5))) {
          esp_state = ESP_DRONE_DATA;
        } else if (packet_length > ESP_MAX_LEN) {
          // printf("[uart-err] Packet unexpectedly long \n");
          esp_state = ESP_RX_ERR;
        }	else {
          // do nothing?!
        } 
      } else {
        // do nothing?!
      }
      
    } break;

    case ESP_DRONE_DATA: {
      // TODO: remove when full checksum
      uint8_t st_byte_pos = 5;

      if (byte_ctr < packet_length) {
        /* fill a localbuf and calculate local checksum */
        databuf[byte_ctr - st_byte_pos] = c;

        checksum += databuf[byte_ctr - st_byte_pos];
        
        byte_ctr = byte_ctr + 1;
      }

      /* after receiving the msg, terminate ssid string */
      if (byte_ctr == packet_length) {
        byte_ctr = 0;
        esp_state = ESP_ERR_CHK;
      }
    } break;

    case ESP_ERR_CHK: {
      /* check if last packet matches your checksum */
      if (c == checksum) {
        // printf("[uart] checksum matched!\n");
        esp_state = ESP_RX_OK;
      }
      else {
        esp_state = ESP_RX_ERR;
      }
    } // no break statement required;

    case ESP_RX_OK: {
      #ifdef DBG
      char tmstr1[50] = {0};
      for (int ct = 0; ct < sizeof(drone_data_t); ct++) {
        sprintf(tmstr1, "[uart] ESP_RX_OK: 0x%02x\n", databuf[ct]);
        Serial.println(tmstr1);
      }
      
      #endif

      /* checksum matches, proceed to populate the struct */
      rx_struct(&dr_data[drone_id], databuf);
      drone_info_t info = {
        .drone_id = drone_id,
        .packet_type = packet_type,
        .packet_length = packet_length,
      };
      uint8_t infobuf[3] = {0};
      memcpy(&infobuf, &info, sizeof(drone_info_t));

      // write into 802.11 packet ssid info
      memcpy(&packet[39], &infobuf, sizeof(drone_info_t));
      // write into 802.11 packet ssid data
      memcpy(&packet[42], &databuf, sizeof(drone_data_t));
      
      // broadcastSSID();

      checksum = 0;

      /* reset state machine */
      esp_state = ESP_SYNC;

    } break;
    case ESP_RX_ERR: {
            #ifdef DBG
            char tmstr2[50] = {0};
            sprintf(tmstr2, "[uart] ESP_RX_ERR\n");
            Serial.println(tmstr2);
            #endif 
            
            byte_ctr = 0;
            checksum = 0;
            /* reset state machine, string terminated earlier than expected */
            esp_state = ESP_SYNC;
    } break;
    default: {
            byte_ctr = 0;
            checksum = 0;
            esp_state = ESP_SYNC;
    } break;
  }

  /* reset state machine asap, when start pattern is observed */
  if (prev_char == '$' && c == 178) {
    byte_ctr = 2;
    esp_state = ESP_DRONE_INFO;
  }

  /* for start byte pattern check */
  prev_char = c;
}


// invoked every 2 seconds
static void rate_loop() {
  static int ctr = 0;
  uart_packet_t uart_packet = {
    .info = {
      .drone_id = SELF_ID,
      .packet_type = DATA_FRAME,
      .packet_length = 2 + sizeof(uart_packet_t),
    },
    .data = {
      .pos = {
        .x = -1.53,
        .y = -346.234,
        .z = 23455.234,
      },
      .heading = -452.12,
      .vel = {
        .x = -1.53,
        .y = -346.234,
        .z = 23455.234,
      },
    },
  };

  unsigned long curr_time = millis();
  static unsigned long prev_time = curr_time;

  if ((curr_time - prev_time) > 2000) {
    ctr = ctr + 1;
    // For testing only
    // tx_struct(&uart_packet);
    prev_time = curr_time;
  }
}


void loop() {
  rate_loop();

  // rx: state machine
  if (Serial.available() > 0) {
    char incomingByte = Serial.read();
    esp_parse((uint8_t) incomingByte);
  }
      
  int r = random(0, 9999);

  //50Tx,50Rx
  if (r < 6667) {
    broadcastSSID();
  } 
  else {
    scan(channel, 60); //S
  }
}


// tx: send struct to esp32
static void tx_struct(uart_packet_t *uart_packet) {

  uint8_t tx_string[ESP_MAX_LEN] = {0};

  //uart_packet_t = drone_info_t + drone_data_t;

  // copy packed struct into a string
  memcpy(tx_string, uart_packet, sizeof(uart_packet_t));

  #ifdef DBG
  char tmpstr[100] = {0};
  int idx = sprintf(tmpstr, "ssid should be:\n");
  for (int i = 0; i < sizeof(uart_packet_t); i++) {
    idx= sprintf(&tmpstr[idx], "0x%02x,", tx_string[i]);
  }
  sprintf(&tmpstr[idx], "\n*******\n");
  Serial.write(tmpstr);
  #endif

  // send "stringed" struct
  esp_send_string(tx_string, sizeof(uart_packet_t));
  
}


// config.ssid = 0;
// config.bssid = 0;
// config.channel = ch;
// config.show_hidden = true;
// config.scan_type = WIFI_SCAN_TYPE_PASSIVE;
// config.scan_time.passive = Ts;

// uint16_t ap_count = 0;
// memset(ap_info, 0, sizeof(ap_info));
// ESP_ERROR_CHECK(esp_wifi_start());
// ESP_ERROR_CHECK(esp_wifi_scan_start(&config, true)); // Do make blocking calls
// ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
// ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));

// for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < ap_count); i++) {
    
//     memcpy(&ssidsdk, &ap_info[i].ssid, 32);
//     Serial.print("SSID: ");
//     Serial.println(ssidsdk); // suspecting typecasting problem here
// }
