// src/hardware/SDI12TalonAdapter.h

#ifndef SDI12_TALON_ADAPTER_H
#define SDI12_TALON_ADAPTER_H

#include "ISDI12Talon.h"
#include "SDI12Talon.h"

/**
 * @brief Adapter implementing ISDI12Talon interface
 * 
 * Adapts the concrete SDI12Talon class to the ISDI12Talon interface
 * for dependency injection and testing purposes.
 */
class SDI12TalonAdapter : public ISDI12Talon {
public:
    /**
     * @brief Constructor that takes a reference to a concrete SDI12Talon
     * @param talon The concrete SDI12Talon implementation to delegate to
     */
    SDI12TalonAdapter(SDI12Talon& talon) : talon(talon) {}
    ~SDI12TalonAdapter() override = default;

    // SDI12 communication methods
    int getAddress() override { return talon.getAddress(); }
    String sendCommand(String command) override { return talon.sendCommand(command); }
    String command(String commandStr, int address) override { return talon.command(commandStr, address); }
    int startMeasurment(int Address) override { return talon.startMeasurment(Address); }
    int startMeasurmentIndex(int index, int Address) override { return talon.startMeasurmentIndex(index, Address); }
    String continuousMeasurmentCRC(int Measure, int Address) override { return talon.continuousMeasurmentCRC(Measure, Address); }
    bool testCRC(String message) override { return talon.testCRC(message); }
    
    // Port management methods
    int enableData(uint8_t port, bool state) override { return talon.enableData(port, state); }
    int enablePower(uint8_t port, bool state) override { return talon.enablePower(port, state); }
    void disableDataAll() override { talon.disableDataAll(); }
    uint8_t getNumPorts() override { return talon.getNumPorts(); }
    
    // Sensor interrogation
    bool isPresent() override { return talon.isPresent(); }
    
    // Error handling and state reporting
    //int throwError(uint32_t error) override { return talon.throwError(error); }
    String getSensorPortString() override { return talon.getSensorPortString(); }
    String getTalonPortString() override { return talon.getTalonPortString(); }
    uint8_t getSensorPort() override { return talon.getSensorPort(); }
    uint8_t getTalonPort() override { return talon.getTalonPort(); }
    int restart() override { return talon.restart(); }

private:
    SDI12Talon& talon; // Reference to the concrete implementation
};

#endif // SDI12_TALON_ADAPTER_H