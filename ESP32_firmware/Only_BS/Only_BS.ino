#include <WiFi.h>
#include "esp_wifi.h"
#include "Only_BS.h"
#include "wsled.h"

// #define DBG
uint8_t esp_state = ESP_SYNC;
uint8_t channel = 0;
char ssidsdk[32] = {0};

void random_mac() {
  // Randomize SRC MAC
  packet802[10] = packet802[16] = random(256);
  packet802[11] = packet802[17] = random(256);
  packet802[12] = packet802[18] = random(256);
  packet802[13] = packet802[19] = random(256);
  packet802[14] = packet802[20] = random(256);
  packet802[15] = packet802[21] = random(256);
}

void scan(uint8_t ch, uint8_t Ts) {
  int numSsid = WiFi.scanNetworks(false, true, true, Ts, ch);

  /* scan for other drone IDs */
  for (int j = 0; j < numSsid; j++) {
    // want to read all the 32B
    uint8_t ssidstr[32] = {0};

    // these are making blocking calls mostly?!
    // BUG! suspecting wifi.ssid terminating when ssid char = 0x00
    // Hacky patch applied @ .arduino15/packages/esp32/hardware/esp32/1.0.4/libraries/WiFi/src
    // switch to esp-idf please!! And force promiscuous mode!!
    WiFi.SSID(j, ssidstr);
    #ifdef DBG
    Serial.print(j);
    Serial.print("->");
    for (int iwer = 0; iwer < 32; iwer ++) {
      Serial.print(ssidstr[iwer], HEX);
    }
    #endif

    /* print other drones location but with $ appended and CR/LN at the end of packet */
    if (ssidstr[0] == '$' && ssidstr[1] == 178) {
      send_to_bebop(&ssidstr[2], sizeof(uart_packet_t));
    }
  }
}

// make this my ssid, can only be 32B long
void broadcastSSID() {
  // fixed header "$"
  random_mac();
  for (int c = 1; c < 15; c++) {
    packet802[80] = c;
    esp_wifi_set_channel(c, WIFI_SECOND_CHAN_NONE);
    esp_wifi_80211_tx(WIFI_IF_AP, packet802, LEN_802_11, false);
    delay(1);
  } //14 channels
  delay(8);
}

// tx: send the ssid hex array to bebop after "$-ssid" was found in AP scan
static uint8_t send_to_bebop(uint8_t *s, uint8_t len) {

  // augment start bytes
  Serial.write('$');
  Serial.write(178);

  uint8_t checksum = 0;

  // maximum of 255 bytes
  uint8_t i = 0;
  for (i = 0; i < len; i ++) {
    Serial.write(s[i]);
    checksum += s[i];
  }
  Serial.write(checksum);

  #ifdef DBG
  char tmpstr[30] = {0};
  sprintf(tmpstr, "appended checksum while bbp tx: 0x%02x\n", checksum);
  Serial.print(tmpstr);
  #endif

  return (i+3);
}

// rx: in debug mode: print struct received after checksum match
static void print_drone_data_struct(drone_data_t *dat) {

}

/* absolutely needed for now, TODO: maybe software flow control instead?! */
void serial_flush() {
  while(Serial.available() > 0) {
    char wipe = Serial.read();
  }
}

// rx: success indication
static void string_to_struct(uint8_t* buf) {

  static bool toggle = 0;
  toggle = !toggle;
  if (toggle) {
    leds[0] = CRGB::Red;
  } else {
    leds[0] = CRGB::Black;
  }
  FastLED.show();

  drone_data_t dr_dat = {0};
	memcpy(&dr_dat, buf, sizeof(drone_data_t));
  #ifdef DBG
	char tmpstr[200] = {0};
	sprintf(tmpstr, "dat.pos.x: %f, dat.pos.y: %f, dat.heading: %f, dat.vel: %f\n",
					dr_dat.pos.x, dr_dat.pos.y,	dr_dat.heading,	dr_dat.vel);
  Serial.println(tmpstr);
  #endif

}

// rx: parse UART bytes to from a packet by switching state machines
uint8_t databuf[ESP_MAX_LEN] = {0};
void parse_received_from_bebop(uint8_t c) {
  static uint8_t byte_ctr      = 0;
  static uint8_t drone_id      = 0;
  static uint8_t packet_length = 0;
  static uint8_t packet_type   = 0;
  static uint8_t checksum      = 0;
  static uint8_t prev_char     = 0;

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
        /* take note of drone id */
        drone_id = c;
        byte_ctr = byte_ctr + 1;
      } else if (byte_ctr == 3) {
        /* take note of packet type */
        packet_type = c;
        byte_ctr = byte_ctr + 1;
      } else if (byte_ctr == 4) {
        /* take note of packet length */
        packet_length = c;
        byte_ctr = byte_ctr + 1;

        if (packet_type == ACK_FRAME && packet_length == 4) {
          // TODO: esp received sid change signal and sent you ack,
          // indicate that on bool pprz esp ping?
          esp_state = ESP_RX_OK;
          byte_ctr = 0;
        }

        /* packet length will always be shorter than padded struct, create some leeway */
        else if ((packet_type == DATA_FRAME || packet_type == COLOR_FRAME) && (packet_length >= (sizeof(drone_info_t) + sizeof(drone_color_t)))) {
					// overwrite old checksum, start afresh
					checksum = drone_id + packet_type + packet_length;
          #ifdef DBG
          char checksumstr[40] = {0};
          sprintf(checksumstr, "cs: 0x%02x\n", checksum);
          Serial.println(checksumstr);
          #endif
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
      const uint8_t st_byte_pos = sizeof(drone_info_t) + 2;

      if (byte_ctr < packet_length) {
        /* fill a databuf from zero and calculate data+info checksum */
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
      /* take in the last byte and check if it matches data+info checksum */
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
      // char tmstr1[50] = {0};
      // for (int ct = 0; ct < sizeof(drone_data_t); ct++) {
      //   sprintf(tmstr1, "[uart] ESP_RX_OK: 0x%02x\n", databuf[ct]);
      //   Serial.println(tmstr1);
      // }
      #endif

      /* checksum matches, proceed to formulate 802.11 packet */
      serial_flush();
      if (packet_type == DATA_FRAME) {
        /* debug via LED and DBG prints */
        string_to_struct(databuf);
        
        drone_info_t info = {
          .drone_id = drone_id,
          .packet_type = packet_type,
          .packet_length = packet_length,
        };
        uint8_t infobuf[3] = {0};
        memcpy(&infobuf, &info, sizeof(drone_info_t));

        // write into 802.11 packet ssid info
        memcpy(&packet802[40], &infobuf, sizeof(drone_info_t));
        // write into 802.11 packet ssid data
        memcpy(&packet802[43], &databuf, sizeof(drone_data_t));
      }

      if (packet_type == COLOR_FRAME) {
        drone_color_t drone_color;
        memcpy(&drone_color, &databuf, sizeof(drone_color_t));
        leds[0].r = drone_color.r;
        leds[0].g = drone_color.g;
        leds[0].b = drone_color.b;
        FastLED.show();
      }

      /* now ready for broadcastSSID() after writing to packet */
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
            serial_flush();
            byte_ctr = 0;
            checksum = 0;
            /* reset state machine, string terminated earlier than expected */
            esp_state = ESP_SYNC;
    } break;
    default: {
            serial_flush();
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
  // pinMode(21, OUTPUT);
  // digitalWrite(21, HIGH);

  // Init RGB LED
  FastLED.addLeds<WS2812B, LED_WS2812B, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(50);
  leds[0] = CRGB::Green;
  FastLED.show();

  delay(1000);

  // Serial.println("DEVICE: " + DEVCHAR);
}

void loop() {

  // rx: state machine received characters from bebop
  if (Serial.available() > 0) {
    char incomingByte = Serial.read();
    parse_received_from_bebop((uint8_t) incomingByte);
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
