#include <Sodaq_RN2483.h>

#define loraSerial Serial2

void setup()

{

    loraSerial.begin(LoRaBee.getDefaultBaudRate());

    LoRaBee.init(loraSerial, LORA_RESET);

    LoRaBee.sleep(); // command to set the LoRaWAN module to its lowest power mode

}
