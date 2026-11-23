/**
 * @file RobotController.cpp
 * @brief Robot Core Controller Implementation
 */

#include "RobotController.h"
#include "MotorDriverSystem.h"
#include "FunctionModeManager.h"
#include "CommunicationInterface.h"

#include <avr/wdt.h>

// Global Robot Controller Instance
RobotController g_robot;


// ============================================================
// Constructor
// ============================================================

RobotController::RobotController()
    : m_currentMode(OperationMode::Standby),
      m_previousMode(OperationMode::Standby),
      m_motorSystem(nullptr),
      m_modeManager(nullptr),
      m_commInterface(nullptr)
{
    memset(m_commandId, 0, sizeof(m_commandId));
}


// ============================================================
// Destructor
// ============================================================

RobotController::~RobotController() {

    if (m_motorSystem) {
        delete m_motorSystem;
        m_motorSystem = nullptr;
    }

    if (m_modeManager) {
        delete m_modeManager;
        m_modeManager = nullptr;
    }

    if (m_commInterface) {
        delete m_commInterface;
        m_commInterface = nullptr;
    }
}


// ============================================================
// Initialize
// ============================================================

void RobotController::initialize() {

    // --------------------------------------------------------
    // Serial
    // --------------------------------------------------------

    Serial.begin(9600);

    Serial.println(F("================================"));
    Serial.println(F(" Robot Controller Initializing "));
    Serial.println(F("================================"));


    // --------------------------------------------------------
    // Create subsystems
    // --------------------------------------------------------

    m_motorSystem = new MotorDriverSystem();
    m_modeManager = new FunctionModeManager();
    m_commInterface = new CommunicationInterface();


    // --------------------------------------------------------
    // Initialize Motor System
    // --------------------------------------------------------

    Serial.println(F("Initializing Motor System..."));

    m_motorSystem->initialize();


    // --------------------------------------------------------
    // Initialize Mode Manager
    // --------------------------------------------------------

    Serial.println(F("Initializing Mode Manager..."));

    m_modeManager->initialize(m_motorSystem);


    // --------------------------------------------------------
    // Initialize Communication
    // --------------------------------------------------------

    Serial.println(F("Initializing Communication..."));

    m_commInterface->initialize();


    // --------------------------------------------------------
    // Initial state
    // --------------------------------------------------------

    m_currentMode = OperationMode::Standby;
    m_previousMode = OperationMode::Standby;

    memset(m_commandId, 0, sizeof(m_commandId));


    Serial.println(F("--------------------------------"));
    Serial.println(F("Robot Controller Initialized!"));
    Serial.println(F("Ultrasonic safety: ENABLED"));
    Serial.println(F("Obstacle limit: 20 cm"));
    Serial.println(F("Ready to Start..."));
    Serial.println(F("--------------------------------"));
}


// ============================================================
// Main Update
// ============================================================

void RobotController::update() {

    // ========================================================
    // 1. UPDATE SENSORS
    // ========================================================
    //
    // IMPORTANT:
    //
    // The ultrasonic sensor must be updated regardless
    // of the current operation mode.
    //
    // This guarantees that manual control, line following,
    // AI control, etc. can all use the safety system.
    //
    // updateSensors() internally controls the update rate.
    // ========================================================

    if (m_motorSystem != nullptr) {
        m_motorSystem->updateSensors();
    }


    // ========================================================
    // 2. PROCESS INPUTS
    // ========================================================

    if (m_commInterface != nullptr) {

        // Serial / WiFi commands
        m_commInterface->processSerialCommands(this);

        // IR remote
        m_commInterface->processIRRemote(this);

        // Keyboard / buttons
        m_commInterface->processKeyPress(this);

        // Automatic timeout for IR direction commands
        m_commInterface->checkDirectionKeyTimeout(this);
    }


    // ========================================================
    // 3. EXECUTE CURRENT MODE
    // ========================================================

    executeCurrentMode();


    // ========================================================
    // 4. UPDATE LED STATUS
    // ========================================================

    static uint32_t lastLEDUpdate = 0;

    if (millis() - lastLEDUpdate > 100) {

        lastLEDUpdate = millis();

        if (m_motorSystem != nullptr &&
            m_commInterface != nullptr) {

            const SensorData& sensorData =
                m_motorSystem->getSensorData();

            m_commInterface->updateLEDStatus(
                static_cast<uint8_t>(m_currentMode),
                sensorData.power.isLowBattery
            );
        }
    }
}


// ============================================================
// Set Operation Mode
// ============================================================

void RobotController::setMode(OperationMode mode) {

    if (m_currentMode == mode) {
        return;
    }


    // Save previous mode
    m_previousMode = m_currentMode;

    // Change mode
    m_currentMode = mode;


    // --------------------------------------------------------
    // Safety: stop robot when changing modes
    // --------------------------------------------------------

    if (m_motorSystem != nullptr) {

        m_motorSystem->stop();
    }


    // --------------------------------------------------------
    // Debug
    // --------------------------------------------------------

    Serial.print(F("Mode changed: "));

    Serial.print(
        static_cast<int>(m_previousMode)
    );

    Serial.print(F(" -> "));

    Serial.println(
        static_cast<int>(m_currentMode)
    );
}


// ============================================================
// Set Command ID
// ============================================================

void RobotController::setCommandId(const char* id) {

    if (id == nullptr) {
        return;
    }

    strncpy(
        m_commandId,
        id,
        sizeof(m_commandId) - 1
    );

    m_commandId[sizeof(m_commandId) - 1] = '\0';
}


// ============================================================
// Execute Current Mode
// ============================================================

void RobotController::executeCurrentMode() {

    if (m_modeManager == nullptr) {
        return;
    }


    switch (m_currentMode) {


        // ====================================================
        // STANDBY
        // ====================================================

        case OperationMode::Standby:

            m_modeManager->executeStandby();

            break;


        // ====================================================
        // LINE FOLLOWING
        // ====================================================

        case OperationMode::LineFollowing:

            m_modeManager->executeLineFollowing();

            break;


        // ====================================================
        // OBSTACLE AVOIDANCE
        // ====================================================

        case OperationMode::ObstacleAvoidance:

            m_modeManager->executeObstacleAvoidance();

            break;


        // ====================================================
        // TARGET FOLLOWING
        // ====================================================

        case OperationMode::TargetFollowing:

            m_modeManager->executeTargetFollowing();

            break;


        // ====================================================
        // MANUAL CONTROL
        // ====================================================

        case OperationMode::ManualControl:

            // Movement is controlled by external commands.
            // Safety is handled inside FunctionModeManager.

            break;


        // ====================================================
        // PROGRAMMING
        // ====================================================

        case OperationMode::Programming:

            // Waiting for external commands.

            break;


        // ====================================================
        // INVALID MODE
        // ====================================================

        default:

            Serial.println(
                F("ERROR: Invalid operation mode!")
            );

            setMode(OperationMode::Standby);

            break;
    }
}