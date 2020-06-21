#include <SPI.h>

void DFlashUltraDeepSleep()

{
    static const uint8_t SS_DFLASH = 44;
    SPI.begin();

    // Initialize the CS pin for the data flash
    pinMode(SS_DFLASH, OUTPUT);
    digitalWrite(SS_DFLASH, HIGH);
    transmit(0xB9 );
    SPI.end(); // close SPI and reset pins to low power state
    resetSPIPins();
}

void transmit (uint8_t val)

{
    SPISettings settings;
    digitalWrite(SS_DFLASH, LOW) ;
    SPI.beginTransaction(settings ) ;
    SPI.transfer(val);
    SPI.endTransaction();
    digitalWrite(SS_DFLASH, HIGH) ;
    delayMicroseconds (1000) ;
}

void resetSPIPins()

{
    resetPin(MISO) ;
    resetPin(MOSI ) ;
    resetPin(SCK) ;
    resetPin(SS_DFLASH) ;
}

void resetPin(uint8_t pin)

{
  PORT->Group[g_APinDescription[pin].ulPort].
  PINCFG[g_APinDescription[pin].ulPin].reg=(uint8_t)(0);
  PORT->Group[g_APinDescription[pin].ulPort].
  DIRCLR.reg = (uint32_t)(1<
  PORT->Group[g_APinDescription[pin].ulPort].
  OUTCLR.reg = (uint32_t) (1<
}
