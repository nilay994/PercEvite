#include <WiFi.h>
#include "esp_wifi.h"

#include "wsled.h"


esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);

uint8_t channel = 0;
#define DEVCHAR '1'

int r;

char buffer[28] = {'*', '*', '*', '*',
                   '*', '*', '*', '*',
                   '*', '*', '*', '*',
                   '*', '*', '*', '*',
                   '*', '*', '*', '*',
                   '*', '*', '*', '*',
                   '*', '*', '*', '*'
                  };

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
                     /*38*/'(', DEVCHAR, 'E', 'S',
                     /*42*/'P', '-', 'M', 'A',
                     /*46*/'V', 'L', 'A', 'B',
                     /*50*/DEVCHAR, '*', '*', '*',
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

  for (int j = 0; j < numSsid; j++) {
    String ssid = WiFi.SSID(j);
    if (ssid.startsWith("(")) {
      Serial.println(ssid);
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
  }//14 channels
  delay(8);
}

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(100);
  WiFi.mode(WIFI_AP_STA);

  //Set channel
  channel = random(1, 14);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_max_tx_power(78);

  

  //Select external antenna
  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);

  // Init RGB LED
  FastLED.addLeds<WS2812B, LED_WS2812B, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(32);
  leds[0] = CRGB::Black;
  FastLED.show();

  delay(1000);

  Serial.println("DEVICE: " + DEVCHAR);
}


int ctr = 0;

void onesecondloop() {
  unsigned long curr_time = millis();
  static unsigned long prev_time = curr_time;
  if ((curr_time - prev_time) > 1000) {
    ctr = ctr + 1;
    prev_time = curr_time;
    // toggle every second
    if (ctr % 2 == 0) {
      if (DEVCHAR == '2') {
        leds[0] = CRGB::Red;
      }
      if (DEVCHAR == '1') {
        leds[0] = CRGB::Green;
      }
    }
    else {
      leds[0] = CRGB::Black;
    }
    FastLED.show();
  }
}


void loop()
{
  onesecondloop();
  r = random(0, 9999);

  // three digit alive packet, 4th as terminator
  char eta[4] = {'0'};

  //50Tx,50Rx
  if (r < 6667) {

    // //digitalWrite(15, HIGH);
    // Serial.println('>'); //hotUsed to synchronize UART communication
    // Serial.write(17);//XON

    // alive packet
    sprintf(eta, "%03d", ctr);
    // Serial.println(eta);
    packet[47] = eta[0];
    packet[48] = eta[1];
    packet[49] = eta[2];

    int b = Serial.readBytesUntil('*', &packet[40], 26);
    // Serial.write(19);//XOFF
    broadcastSSID();//B

  } else {
    unsigned long start = micros();
    //Serial.println('S');
    scan(channel, 60); //S
    unsigned long stop = micros();
    //Serial.printf("s-dur: %u\r\n", stop - start);
  }

}
