#ifndef FUNCTION_MODE_MANAGER_H
#define FUNCTION_MODE_MANAGER_H

#include <Arduino.h>

class MotorDriverSystem;

namespace ModeConfig {

    namespace LineFollow {
        constexpr uint8_t NORMAL_SPEED = 100;
        constexpr uint8_t TURN_SPEED = 60;
        constexpr uint8_t SHARP_TURN_SPEED = 90;
        constexpr uint16_t SCAN_TIMEOUT_MS = 3000;
    }

    namespace ObstacleAvoid {
        constexpr uint8_t DETECTION_DISTANCE = 20;
        constexpr uint8_t MOVE_SPEED = 70;
        constexpr uint8_t TURN_SPEED = 70;
        constexpr uint16_t BACKUP_TIME = 100;
        constexpr uint16_t TURN_TIME = 50;
    }

    namespace TargetFollow {
        constexpr uint8_t MIN_DISTANCE = 5;
        constexpr uint8_t STOP_DISTANCE = 20;
        constexpr uint8_t FOLLOW_DISTANCE = 30;
        constexpr uint8_t BACKWARD_SPEED = 80;
        constexpr uint8_t FORWARD_SPEED = 100;
    }

    // SEGURANÇA DO CONTROLE MANUAL/IA
    namespace Safety {
        constexpr uint8_t STOP_DISTANCE = 20;
        constexpr uint8_t MIN_VALID_DISTANCE = 2;
        constexpr uint16_t SENSOR_TIMEOUT_MS = 500;
    }
}

class FunctionModeManager {

public:

    void initialize(MotorDriverSystem* motorSystem);

    void executeLineFollowing();

    void executeObstacleAvoidance();

    void executeTargetFollowing();

    void executeManualControl(uint8_t direction, uint8_t speed);

    void executeStandby();

    void resetLineFollowingState();

private:

    MotorDriverSystem* m_motorSystem;

    struct {
        bool isFirstEnter;
    } m_obstacleAvoid;

    struct {
        bool isFirstEnter;
    } m_targetFollow;

    bool isFrontBlocked();

    bool isUltrasonicValid();

    bool isInRange(long value, long min, long max) {
        return (value >= min) && (value <= max);
    }

    void safeDelay(uint16_t ms);
};

#endif