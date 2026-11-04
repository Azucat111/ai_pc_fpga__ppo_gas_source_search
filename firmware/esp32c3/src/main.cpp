#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "wifi_config.h"
#include "client_router.h"

static WiFiUDP udp;
static ClientRouter router;
static uint32_t lastByte=0;
static uint8_t frame[192];
static size_t used=0;

static uint16_t crc16(const uint8_t *p,size_t n) {
    uint16_t crc=0xffff;
    while(n--) { crc^=uint16_t(*p++)<<8;
        for(unsigned i=0;i<8;i++) crc=(crc&0x8000)?uint16_t((crc<<1)^0x1021):uint16_t(crc<<1);
    }return crc;
}
static bool validFrame(const uint8_t *p,size_t n,uint8_t kind) {
    return n>=6 && p[0]==0xa5 && p[1]==0x5a && p[2]==1 && p[3]==kind &&
           crc16(p,n-2)==(uint16_t(p[n-2])|(uint16_t(p[n-1])<<8));
}

void setup() {
    Serial1.setRxBufferSize(2048);
    Serial1.begin(CAR_UART_BAUD,SERIAL_8N1,CAR_UART_RX,CAR_UART_TX);
    if(CAR_WIFI_AP) {
        WiFi.mode(WIFI_AP);WiFi.softAP(CAR_WIFI_SSID,CAR_WIFI_PASSWORD,1,0,2);
    } else {
        WiFi.mode(WIFI_STA);WiFi.begin(CAR_WIFI_SSID,CAR_WIFI_PASSWORD);
        // No UART command generation here; link loss is handled by the PL lease.
    }
    udp.begin(CAR_UDP_PORT);
}

void loop() {
    int packetSize=udp.parsePacket();
    if(packetSize>0) {
        uint8_t packet[33];int n=udp.read(packet,sizeof(packet));
        if((packetSize==5 && n==5 && memcmp(packet,"FCAR?",5)==0) ||
           (packetSize==32 && n==32 && (validFrame(packet,32,1) || validFrame(packet,32,2)))) {
            CarEndpoint source;source.ip=uint32_t(udp.remoteIP());source.port=udp.remotePort();
            bool registered=router.subscribe(source,millis());
            const char *reply=nullptr;
            if(packetSize==32 && packet[3]==2 && registered) { Serial1.write(packet,32); }
            else if(packetSize==32 && packet[3]==1 && packet[16]==0) { router.command(source,packet,millis());Serial1.write(packet,32); }
            else if(!registered) reply=router.blocked(source) ? "FCAR_BUSY" : "FCAR_FULL";
            else if(packetSize==32) {
                auto decision=router.command(source,packet,millis());
                if(decision==ClientRouter::Forward) Serial1.write(packet,32);
                else reply=decision==ClientRouter::Busy ? "FCAR_BUSY" : "FCAR_NOT_READY";
            }
            if(reply) { udp.beginPacket(udp.remoteIP(),source.port);udp.write(reinterpret_cast<const uint8_t *>(reply),strlen(reply));udp.endPacket(); }
        }
        while(udp.available())udp.read();
    }
    if(used && millis()-lastByte>20)used=0;
    while(Serial1.available()) {
        uint8_t b=uint8_t(Serial1.read());lastByte=millis();
        if(used==0) {if(b==0xa5)frame[used++]=b;continue;}
        if(used==1 && b!=0x5a) {used=(b==0xa5)?1:0;continue;}
        frame[used++]=b;
        if(used==sizeof(frame)) {
            if(validFrame(frame,sizeof(frame),0x81) || validFrame(frame,sizeof(frame),0x82) || validFrame(frame,sizeof(frame),0x83)) {
                if(frame[3]==0x81) router.telemetry(frame,millis());
                for(const auto &s:router.subscribers) if(router.active(s,millis())) {
                    udp.beginPacket(IPAddress(s.endpoint.ip),s.endpoint.port);udp.write(frame,sizeof(frame));udp.endPacket();
                }
            }
            used=0;
        }
    }
    delay(1);
}
