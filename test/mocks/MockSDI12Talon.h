#ifndef MOCK_SDI12_TALON_H
#define MOCK_SDI12_TALON_H

#include <gmock/gmock.h>
#include "ISDI12Talon.h"

/**
 * @brief Mock implementation of the ISDI12Talon interface for testing
 */
class MockSDI12Talon : public ISDI12Talon {
public:
    // SDI12 communication methods
    MOCK_METHOD(int, getAddress, (), (override));
    MOCK_METHOD(String, sendCommand, (String command), (override));
    MOCK_METHOD(String, command, (String commandStr, int address), (override));
    MOCK_METHOD(int, startMeasurment, (int Address), (override));
    MOCK_METHOD(int, startMeasurmentIndex, (int index, int Address), (override));
    MOCK_METHOD(String, continuousMeasurmentCRC, (int Measure, int Address), (override));
    MOCK_METHOD(bool, testCRC, (String message), (override));
    
    // Port management methods
    MOCK_METHOD(int, enableData, (uint8_t port, bool state), (override));
    MOCK_METHOD(int, enablePower, (uint8_t port, bool state), (override));
    MOCK_METHOD(void, disableDataAll, (), (override));
    MOCK_METHOD(uint8_t, getNumPorts, (), (override));
    
    // Sensor interrogation
    MOCK_METHOD(bool, isPresent, (), (override));
    
    // Error handling and state reporting
    //MOCK_METHOD(int, throwError, (uint32_t error), (override));
    MOCK_METHOD(String, getSensorPortString, (), (override));
    MOCK_METHOD(String, getTalonPortString, (), (override));
    MOCK_METHOD(uint8_t, getSensorPort, (), (override));
    MOCK_METHOD(uint8_t, getTalonPort, (), (override));
    MOCK_METHOD(int, restart, (), (override));
};

#endif // MOCK_SDI12_TALON_H