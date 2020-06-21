uint8_t message[2];

void setup()
{
    pinMode(TEMP_SENSOR, INPUT);
    getTemperature();
}

void loop()
{ 
}

void getTemperature()
{
int int_temp;
    uint8_t negativeFlag;
    
    float mVolts = (float)analogRead(TEMP_SENSOR) * 3300.0 / 1024.0;
    float temp = (mVolts - 500.0) / 10.0;
    
    temp *= 100;
    
    if (temp < 0) negativeFlag = 0x80;
    else negativeFlag = 0x00;
    
    int_temp = abs((int) temp);
    
    message[0] = (int_temp >> 8) | negativeFlag;
    message[1] = int_temp & 0xFF;
}
