/**
 * @file RobotController.h
 * @brief Robot Core Controller
 *
 * Responsibilities:
 * - System initialization and configuration
 * - Main loop scheduling
 * - Mode management and switching
 * - Subsystem coordination
 */

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <Arduino.h>

// Forward declarations
class MotorDriverSystem;
class FunctionModeManager;
class CommunicationInterface;

// Operation Mode Enumeration
enum class OperationMode : uint8_t {
    Standby = 0,
    LineFollowing,
    ObstacleAvoidance,
    TargetFollowing,
    ManualControl,
    Programming
};

/**
 * @brief Main controller of the robot system
 */
class RobotController {

public:

    RobotController();
    ~RobotController();

    /**
     * @brief Initialize the complete robot system
     */
    void initialize();

    /**
     * @brief Main loop update
     *
     * Updates:
     * 1. Sensors
     * 2. Communication
     * 3. Current operation mode
     * 4. LED status
     */
    void update();

    /**
     * @brief Set operation mode
     */
    void setMode(OperationMode mode);

    /**
     * @brief Get current operation mode
     */
    OperationMode getMode() const {
        return m_currentMode;
    }

    /**
     * @brief Set command ID
     */
    void setCommandId(const char* id);

    /**
     * @brief Get command ID
     */
    const char* getCommandId() const {
        return m_commandId;
    }

    // Subsystem access
    MotorDriverSystem* getMotorSystem() {
        return m_motorSystem;
    }

    FunctionModeManager* getModeManager() {
        return m_modeManager;
    }

    CommunicationInterface* getCommInterface() {
        return m_commInterface;
    }

private:

    // Current state
    OperationMode m_currentMode;
    OperationMode m_previousMode;

    char m_commandId[16];

    // Subsystems
    MotorDriverSystem* m_motorSystem;
    FunctionModeManager* m_modeManager;
    CommunicationInterface* m_commInterface;

    /**
     * @brief Execute logic for current operation mode
     */
    void executeCurrentMode();
};

// Global Robot Controller
extern RobotController g_robot;

#endif // ROBOT_CONTROLLER_H