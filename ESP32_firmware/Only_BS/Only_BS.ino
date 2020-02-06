#include <WiFi.h>
#include "esp_wifi.h"
#include "Only_BS.h"
#include "wsled.h"

// #define DEBUG

esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);

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


void scan(uint8_t ch, uint8_t Ts)
{
  int numSsid = WiFi.scanNetworks(false, true, true, Ts, ch);
  /* scan for other drone IDs */
  for (int j = 0; j < numSsid; j++) {
    String ssid = WiFi.SSID(j);
    /* print other drones location but with $ appended and CR/LN at the end of packet */
    if (ssid.startsWith("$D")) {
      
      /* UART: esp2pprz
       *  | Start Byte | Drone State (6*4) Bytes | End Byte (1 Byte) |
       *  |------------|-------------------------|-------------------|
       *  | $          | numbers                 | CR/LN             |
       */
      Serial.println('$' + ssid.substring(2));
    }
  }
}

void broadcastSSID()
{
  random_mac();
  for (int c = 1; c < 15; c++) {
    packet[80] = c;
    esp_wifi_set_channel(c, WIFI_SECOND_CHAN_NONE);
    esp_wifi_80211_tx(WIFI_IF_AP, packet, 81, false);
    delay(1);
  } //14 channels
  delay(8);
}

void setup()
{
  // reset uart state machine 
  esp.state = ESP_SYNC;
  
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
  
  delay(1000);

  // Serial.println("DEVICE: " + DEVCHAR);
}

// invoked every second
int ctr = 0;
void onesecondloop() {
  unsigned long curr_time = millis();
  static unsigned long prev_time = curr_time;
  
  esp_state curr_esp_state = esp.state;
  static esp_state prev_esp_state = curr_esp_state;
   
  if ((curr_time - prev_time) > 1000) {
    ctr = ctr + 1;
    
    prev_esp_state = curr_esp_state;
    prev_time = curr_time;
    
    // Serial print current ssid every second
    #ifdef DEBUG
        // Used to synchronize UART communication
        // Serial.println('>'); 
        // Serial.write(17);//XON
        for (int i = 38; i < 68; i++) {
          Serial.print((char)packet[i]);
        }
        Serial.println("EOF");
        // Serial.write(19);//XOFF
    #endif
    

  }
}

void parse_bytes(char c) {

  /* UART: pprz2esp
   *  | Start Byte | Drone State (6*4) Bytes | End Byte (1 Byte) |
   *  |------------|-------------------------|-------------------|
   *  | $          | numbers                 | CR/LN/*           |
   */

  
  static uint8_t byte_ctr = 0;
  // NOTE: CR and LF are two seperate chars
  
  switch (esp.state) {
    case ESP_SYNC: {
      /* first char, sync string */
      if (c == '$') {
        esp.state = ESP_OK_CHECK;
      }
    } break;
    case ESP_OK_CHECK: {
        /* check for OK string */
        if ((c == 'O') && (byte_ctr == 0)) {
          byte_ctr = byte_ctr + 1;
          break;
        } else if ((c == 'K') && (byte_ctr == 1)) {
          static bool toggle = 0; 
          if (toggle) {  
            // blink red led
            leds[0] = CRGB::Red;
            FastLED.show();
          } else {
            leds[0] = CRGB::Green;
            FastLED.show();
          }
          char tmp_str[80] = {0};
          #ifdef DEBUG
            /* string is okay, print it out and reset the state machine */
            sprintf(tmp_str, "esp.state: %d, OKAY received\n", esp.state);
            Serial.print(tmp_str);
          #endif
          toggle = !toggle;
          byte_ctr = 0;
          esp.state = ESP_SYNC;
        } else {
          byte_ctr = 0;
          esp.state = ESP_RX_MSG;
      }
    } break;
    case ESP_RX_MSG: {  
      
      /* if received terminate before string complete (atleast 3 bytes), trigger ERROR */
      if (((c < 48) || (c > 57)) && (c!='.') && (c!= '-') && (byte_ctr < 10)) {
        byte_ctr = 0;
        esp.state = ESP_RX_ERR;
      }
      
      /* after receiving the msg, terminate ssid string */
      if ((c=='\0' || c=='\n' || c=='\r' || c=='*') && (byte_ctr >= 10)) {
        // TODO: no need to terminate ssid
        // esp.msg.str[byte_ctr] = '\0';
        byte_ctr = 0;
        /* received message, now switch state machine */
        esp.state = ESP_RX_OK; 
      }
      
      /* record ssid until full lat,long,alt are stored in esp.msg.str */
      /* don't change state machine until str terminate char is received */
      else {
        esp.msg.str[byte_ctr] = c;
        byte_ctr = byte_ctr + 1;
      }
    } break;
    case ESP_RX_OK: {
            char tmp_str[80] = {0};
            #ifdef DEBUG
            /* string is okay, print it out and reset the state machine */
            sprintf(tmp_str, "esp.state: %d, esp.msg.str: %s\n", esp.state, esp.msg.str);
            Serial.print(tmp_str);
            #endif
            // strncpy(&packet[42], esp.msg.str, 24);
            for (uint8_t len=0; len<24; len++) {
              packet[42+len] = esp.msg.str[len];  
            }

            /* reset state machine */
            esp.state = ESP_SYNC;

    } break;
    case ESP_RX_ERR: {
            char tmp_str[80] = {0};
            #ifdef DEBUG 
            sprintf(tmp_str, "ESP_RX_ERR: string terminated before drone info\n");
            Serial.println(tmp_str);
            #endif
            
            /* reset state machine, string terminated earlier than expected */
            esp.state = ESP_SYNC;
   } break;
    default: {
            esp.state = ESP_SYNC;
    } break;
    
  } /* end switchcase */
}


void loop()
{
  onesecondloop();

  // send to state machine
  if (Serial.available() > 0) {
    char incomingByte = Serial.read();
    parse_bytes(incomingByte);
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

/*
  // three digit alive packet, 4th as terminator (for testing outdoors)
  char eta[4] = {'0'};
  // alive packet
  sprintf(eta, "%03d", ctr);
  Serial.println(eta);
  packet[47] = eta[0];
  packet[48] = eta[1];
  packet[49] = eta[2];
*/
