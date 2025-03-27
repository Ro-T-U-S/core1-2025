#include <Arduino.h>
#include <nvs.h>
#include <nvs_flash.h>

void clearNVS() {
    int err;
    err=nvs_flash_init();
    Serial.println("nvs_flash_init: " + err);
    err=nvs_flash_erase();
    Serial.println("nvs_flash_erase: " + err);
 }
// **beegee-tokyo ** You are my god. 
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
clearNVS();
}

void loop() {
  // put your main code here, to run repeatedly:

}