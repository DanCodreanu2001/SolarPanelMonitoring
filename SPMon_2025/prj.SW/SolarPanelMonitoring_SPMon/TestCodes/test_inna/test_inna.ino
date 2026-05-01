/*
 * INA3221 SENSOR TEST (Standalone)
 * Uses same library + config as project
 */

#include <Wire.h>
#include <INA3221.h>

/* ========== CONFIG ========== */
#define SHUNT_RESISTOR_VALUE 100u   // 0.1 ohm

/* INA object – same as in your handler */
INA3221 ina3221(INA3221_ADDR40_GND);

/* ===================================== */
void I2C_ScanBus()
{
    Serial.println(">>> I2C: Scanning bus...");
    int count = 0;

    for (uint8_t addr = 1; addr < 127; addr++)
    {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0)
        {
            Serial.printf(">>> I2C: Found device at 0x%02X\n", addr);
            count++;
        }
    }

    Serial.printf(">>> I2C: Scan done. %d device(s) found.\n\n", count);
}

/* ===================================== */
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n===== INA3221 SENSOR TEST =====");

    /* I2C INIT */
    Wire.begin();
    delay(300);

    /* Scan first */
    I2C_ScanBus();

    /* INA INIT */
    Serial.println(">>> INA3221 Init...");

    ina3221.begin(&Wire);
    ina3221.reset();
    ina3221.setShuntRes(
        SHUNT_RESISTOR_VALUE,
        SHUNT_RESISTOR_VALUE,
        SHUNT_RESISTOR_VALUE
    );

    Serial.println(">>> INA3221 Initialized OK\n");
}

/* ===================================== */
void loop()
{
    float current_ch1 = ina3221.getCurrent(INA3221_CH1) * 1000.0;
    float voltage_ch1 = ina3221.getVoltage(INA3221_CH1);

    float current_ch2 = ina3221.getCurrent(INA3221_CH2) * 1000.0;
    float voltage_ch2 = ina3221.getVoltage(INA3221_CH2);

    float current_ch3 = ina3221.getCurrent(INA3221_CH3) * 1000.0;
    float voltage_ch3 = ina3221.getVoltage(INA3221_CH3);

    Serial.println("------------ INA3221 ------------");

    Serial.printf("CH1 -> U: %.3f V | I: %.3f mA\n",
                  voltage_ch1, current_ch1);

    Serial.printf("CH2 -> U: %.3f V | I: %.3f mA\n",
                  voltage_ch2, current_ch2);

    Serial.printf("CH3 -> U: %.3f V | I: %.3f mA\n",
                  voltage_ch3, current_ch3);

    Serial.println("--------------------------------\n");

    delay(2000);
}
