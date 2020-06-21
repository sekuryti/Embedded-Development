#include <sodaq_rn2483.h>
#include <sodaq_wdt.h>
#include <spi.h>
#include <rn487x_ble.h>

#define bleSerial Serial1
#define loraSerial Serial2

void setup()
{
    // LoRa module connection and sleep
    loraSerial.begin(LoRaBee.getDefaultBaudRate());
    LoRaBee.init(loraSerial, LORA_RESET);
    LoRaBee.sleep();
    
    // BLE module sleep
    rn487xBle.hwInit();
    bleSerial.begin(rn487xBle.getDefaultBaudRate());
    rn487xBle.initBleStream(&bleSerial);
    rn487xBle.enterCommandMode();
    rn487xBle.hwSleep();
    bleSerial.end();
    
    // set FLASH to deep sleep & reset SPI pins for min. energy consumption
    DFlashUltraDeepSleep();
    
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; // set sleep mode −> deep sleep
    USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE; // Disable USB
}

void loop()
{
    __WFI(); // call again for repeat if interrupted
}

// FLASH chip sleep functions
void DFlashUltraDeepSleep()
{
    static const uint8_t SS_DFLASH = 44 ;
    
    SPI.begin();
    pinMode(SS_DFLASH, OUTPUT);
    digitalWrite(SS_DFLASH, HIGH);
    transmit(0xB9);
    SPI.end();
    resetSPIPins();
}

void transmit(uint8_t val)
{
    SPISettings settings;
    digitalWrite(SS_DFLASH, LOW);
    SPI.beginTransaction(settings);
    SPI.transfer(val);
    SPI.endTransaction();
    digitalWrite(SS_DFLASH, HIGH);
    delayMicroseconds(1000);
}

void resetSPIPins()
{
    resetPin(MISO);
    resetPin(MOSI);
    resetPin(SCK);
    resetPin(SS_DFLASH);
}

void resetPin(uint8_t pin)
{
    PORT->Group[g_APinDescription[pin].ulPort].
    PINCFG[g_APinDescription[pin].ulPin].reg=(uint8_t)(0);
    PORT->Group[g_APinDescription[pin].ulPort].
    DIRCLR.reg = (uint32_t)(1<<g_apindescription[pin].ulpin);
    port-="">Group[g_APinDescription[pin].ulPort].
    OUTCLR.reg = (uint32_t) (1<<g_apindescription[pin].ulpin);
}
