#include <WiFi.h>
#include "esp_wifi.h"
#include <WiFiUdp.h>


unsigned int net_counter = 0;
const char* ssid = "PercEvite_P";
const char* password =  "hi64nyvy";
//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * host = "192.168.4.1";
const int port = 8000;
//The udp library class
WiFiUDP udp;
char incomingPacket[80];  // buffer for incoming packets

//Broadcast parameters
uint8_t channel = 0;
int r;
esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);

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
               /*38*/'D', 'N', '*', '*', 
               /*42*/'*', '*', '*', '*',
               /*46*/'*', '*', '*', '*',
               /*50*/'*', '*', '*', '*',
               /*54*/'*', '*', '*', '*',
               /*58*/'*', '*', '*', '*',
               /*62*/'*', '*', '*', '*',
               /*66*/'*', '*',
               /*Supported rates*/
               /*68*/0x01, 0x08, 0x82, 0x84,
               /*72*/0x8b, 0x96, 0x24, 0x30, 
               /*76*/0x48, 0x6c, 0x03, 0x01, 
               /*80*/0x04};        


void random_mac(){
  // Randomize SRC MAC
    packet[10] = packet[16] = random(256);
    packet[11] = packet[17] = random(256);
    packet[12] = packet[18] = random(256);
    packet[13] = packet[19] = random(256);
    packet[14] = packet[20] = random(256);
    packet[15] = packet[21] = random(256);
}
void scan(uint8_t ch, uint8_t Ts){

  int numSsid = WiFi.scanNetworks(false, true, true, Ts, ch);
  
  for (int j = 0; j < numSsid; j++) {
    String ssid = WiFi.SSID(j);
    if(ssid.startsWith("D")){
      Serial.println(ssid);
    
    }
    
  }//for
  WiFi.scanDelete();

}//scan()

void broadcastSSID(){
  //digitalWrite(21, HIGH);
  random_mac();
  
  
  for(int c = 1;c<15;c++){
    
    packet[80] = c;
    esp_wifi_set_channel(c,WIFI_SECOND_CHAN_NONE);      
    uint32_t err = esp_wifi_80211_tx(WIFI_IF_STA, packet, 81, false);
    if(err != 0){
      ESP.restart();
    }
    
    delay(1);  
  }//14 channels
 

}
void netCom(){
  if(WiFi.status() == WL_CONNECTED){
   //Send a packet
    udp.beginPacket(host,port);
    udp.printf("message: %u\r\n", net_counter);
    udp.endPacket();
    net_counter++;
    delay(50);
    int packetSize = udp.parsePacket();
    if (packetSize)
    {
      // receive incoming UDP packets
      Serial.printf("Received %d bytes from %s, port %d\n", packetSize, udp.remoteIP().toString().c_str(), udp.remotePort());
      int len = udp.read(incomingPacket, 255);
      if (len > 0)
      {
        incomingPacket[len] = 0;
      }
      Serial.printf("UDP packet contents: %s\n", incomingPacket);
      udp.flush();
    }
    
 }
 else{
      Serial.println("No Wi-Fi connection");
    }
 
    
}

void setup() {
  Serial.begin(230400);
  Serial.setTimeout(100);
  WiFi.mode(WIFI_STA);
 
  //digitalWrite(21, HIGH); // external antenna on WiPy 3.0
  digitalWrite(21, LOW); // chip antenna on WiPy 3.0
  
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(10);
    Serial.print(".");
  }
  Serial.println(WiFi.localIP());
  
  channel = random(1,14);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_max_tx_power(78);
  packet[38] = 'D';
  packet[39] = 'N';
}

void loop() {
  r = random(1,10000);
   if(r <= 5000){

    
    int count = 0;
    if(WiFi.status() != WL_CONNECTED) {
      WiFi.reconnect();
    }
    while (WiFi.status() != WL_CONNECTED) {
      delay(10);
      if(count == 100){
        Serial.print(WiFi.status());
        ESP.restart();
      }
      count+=1;
      Serial.print(".");
    }
    Serial.println(WiFi.localIP());
    udp.begin(WiFi.localIP(),port);

    netCom();
    delay(20);
    udp.stop();
    
    //WiFi.disconnect();

    }
    else if(r>5000 && r<8000){

       Serial.write(17);//XON
      Serial.println('>'); //Used to synchronize UART communication
      int b = Serial.readBytesUntil('*',&packet[39], 28);
      broadcastSSID();//B   
      Serial.write(19);//XON
   
    }
   else{
     scan(channel,60);//S
   }
   end:if(ESP.getFreeHeap() < 273500){
    Serial.println("Heap is full!");
    delay(200);
   }
  
}
