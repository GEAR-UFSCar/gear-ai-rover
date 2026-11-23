#ifndef COMMUNICATION_INTERFACE_H
#define COMMUNICATION_INTERFACE_H

#include <Arduino.h>
#include "FastLED.h"

class RobotController;

namespace LEDConfig {

    constexpr uint8_t PIN_LED = 4;
    constexpr uint8_t NUM_LEDS = 1;
    constexpr uint8_t DEFAULT_BRIGHTNESS = 20;
}

namespace KeyConfig {

    constexpr uint8_t PIN_KEY = 2;
    constexpr uint8_t KEY_VALUE_MAX = 4;
}

namespace IRConfig {

    constexpr uint8_t PIN_IR = 9;
    constexpr uint16_t TIMEOUT_MS = 300;
}

class CommunicationInterface {

public:

    void initialize();

    void processSerialCommands(
        RobotController* robot
    );

    void processIRRemote(
        RobotController* robot
    );

    void processKeyPress(
        RobotController* robot
    );

    void checkDirectionKeyTimeout(
        RobotController* robot
    );

    void updateLEDStatus(
        uint8_t mode,
        bool lowBattery
    );

    void setLEDColor(
        uint8_t ledIndex,
        uint8_t r,
        uint8_t g,
        uint8_t b
    );

    void setAllLEDs(
        uint8_t r,
        uint8_t g,
        uint8_t b
    );

    void setLEDBrightness(
        uint8_t brightness
    );

    void clearLEDs();

    CRGB leds[LEDConfig::NUM_LEDS];

private:

    char m_serialBuffer[256];

    uint16_t m_bufferPos;

    void parseJSONCommand(
        const char* jsonString,
        RobotController* robot
    );

    void executeCommand(
        int cmdId,
        RobotController* robot
    );

    uint32_t m_IR_PreMillis;

    bool m_IRrecv_enabled;

    uint8_t m_manualSpeed;

    uint8_t m_servoAngle;

    bool getIRButton(
        uint8_t* buttonNum
    );

    uint8_t m_lastKeyValue;

    uint8_t readKeyValue();

    uint8_t m_ledBrightness;

    void displayModeColor(
        uint8_t mode
    );

    void displayLowBatteryWarning();

    void displayBreathingEffect();

    bool isInRange(
        long value,
        long min,
        long max
    );

    struct {

        uint8_t brightness;

        bool increasing;

        uint32_t lastUpdate;

    } m_breathingEffect;
};

#endif