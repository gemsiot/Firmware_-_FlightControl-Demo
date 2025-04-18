#ifndef TESTING
#include "Particle.h" 
#else
#include "MockParticle.h"
#endif

/*
 * Project FlightControl_Demo
 * Description: Core controller for Flight data logger
 * Author: Bobby Schulz
 * Date: 06/14/2022
 * © 2023 Regents of the University of Minnesota. All rights reserved.
 */

// #define RAPID_START  //Does not wait for remote connection on startup
void setup();
void loop();
void logEvents(uint8_t type, uint8_t destination);
String getErrorString();
String getDataString();
String getDiagnosticString(uint8_t level);
String getMetadataString();
String initSensors();
void quickTalonShutdown();
bool serialConnected();
void systemConfig();
int sleepSensors();
int wakeSensors();
int detectTalons(String dummyStr);
int detectSensors(String dummyStr);
int setNodeID(String nodeID);
int takeSample(String dummy);
int commandExe(String command);
int systemRestart(String resetType);
int configurePowerSave(int desiredPowerSaveMode);

#define WAIT_GPS false
#define USE_CELL  //System attempts to connect to cell
#include <AuxTalon.h>
#include <PCAL9535A.h>
#include <Sensor.h>
#include <Talon.h>
#include <BaroVue10.h>
#include <Kestrel.h>
#include <KestrelFileHandler.h>
#include <Haar.h>
#include <Hedorah.h>
#include <Aleppo.h>
#include <T9602.h>
#include <SO421.h>
#include <SP421.h>
#include <TEROS11.h>
#include <ATMOS22.h>
#include <TDR315H.h>
#include <Li710.h>
#include <I2CTalon.h>
#include <SDI12Talon.h>
#include <Gonk.h>
#include <vector>
#include <memory>

#include "hardware/SDI12TalonAdapter.h"
#include "platform/ParticleTimeProvider.h"

const String firmwareVersion = "2.9.11";
const String schemaVersion = "2.2.9";

const unsigned long maxConnectTime = 180000; //Wait up to 180 seconds for systems to connect 
const unsigned long indicatorTimeout = 60000; //Wait for up to 1 minute with indicator lights on
const uint64_t balancedDiagnosticPeriod = 3600000; //Report diagnostics once an hour //DEBUG!
int powerSaveMode = 0; //Default to 0, update when configure power save mode is called 

Kestrel logger(true);
KestrelFileHandler fileSys(logger);
Gonk battery(5); //Instantiate with defaults, manually set to port 5 
AuxTalon aux(0, 0x14); //Instantiate AUX talon with deaults - null port and hardware v1.4
I2CTalon i2c(0, 0x21); //Instantiate I2C talon with alt - null port and hardware v2.1
SDI12Talon sdi12(0, 0x14); //Instantiate SDI12 talon with alt - null port and hardware v1.4
PCAL9535A ioAlpha(0x20);
PCAL9535A ioBeta(0x21);
SDI12TalonAdapter realSdi12(sdi12);
ParticleTimeProvider realTimeProvider;

String globalNodeID = ""; //Store current node ID

const uint8_t numTalons = 3; //Number must match the number of objects defined in `talonsToTest` array

Talon* talons[Kestrel::numTalonPorts]; //Create an array of the total possible length
Talon* talonsToTest[numTalons] = {
	&aux,
	&i2c,
	&sdi12
};

namespace LogModes {
	constexpr uint8_t STANDARD = 0;
	constexpr uint8_t PERFORMANCE = 1; 
	constexpr uint8_t BALANCED = 2;
	constexpr uint8_t NO_LOCAL = 3; //Same as standard log, but no attempt to log to SD card
};

/////////////////////////// BEGIN USER CONFIG ////////////////////////
//PRODUCT_ID(18596) //Configured based on the target product, comment out if device has no product
PRODUCT_VERSION(33) //Configure based on the firmware version you wish to create, check product firmware page to see what is currently the highest number

const int backhaulCount = 4; //Number of log events before backhaul is performed 
const unsigned long logPeriod = 300; //Number of seconds to wait between logging events 
int desiredPowerSaveMode = PowerSaveModes::LOW_POWER; //Specify the power save mode you wish to use: PERFORMANCE, BALANCED, LOW_POWER, ULTRA_LOW_POWER 
int loggingMode = LogModes::STANDARD; //Specify the type of logging mode you wish to use: STANDARD, PERFORMANCE, BALANCED, NO_LOCAL

Haar haar(0, 0, 0x20); //Instantiate Haar sensor with default ports and version v2.0
// Haar haar1(0, 0, 0x20); //Instantiate Haar sensor with default ports and version v2.0
// Haar haar2(0, 0, 0x20); //Instantiate Haar sensor with default ports and version v2.0
SO421 apogeeO2(sdi12, 0, 0); //Instantiate O2 sensor with default ports and unknown version, pass over SDI12 Talon interface
SP421 apogeeSolar(sdi12, 0, 0); //Instantiate solar sensor with default ports and unknown version, pass over SDI12 Talon interface 
// TEROS11 soil(sdi12, 0, 0); //Instantiate soil sensor with default ports and unknown version, pass over SDI12 Talon interface 
TDR315H soil1(sdi12, 0, 0); //Instantiate soil sensor with default ports and unknown version, pass over SDI12 Talon interface 
TDR315H soil2(sdi12, 0, 0); //Instantiate soil sensor with default ports and unknown version, pass over SDI12 Talon interface 
TDR315H soil3(sdi12, 0, 0); //Instantiate soil sensor with default ports and unknown version, pass over SDI12 Talon interface 
Hedorah gas(0, 0, 0x10); //Instantiate CO2 sensor with default ports and v1.0 hardware
// T9602 humidity(0, 0, 0x00); //Instantiate Telair T9602 with default ports and version v0.0 
LI710 et(realTimeProvider, realSdi12, 0, 0); //Instantiate ET sensor with default ports and unknown version, pass over SDI12 Talon interface 
BaroVue10 campPressure(sdi12, 0, 0x00); // Instantiate Barovue10 with default ports and v0.0 hardware

const uint8_t numSensors = 7; //Number must match the number of objects defined in `sensors` array

Sensor* const sensors[numSensors] = {
	&fileSys,
	&aux,
	&i2c,
	&sdi12,
	&battery,
	&logger, //Add sensors after this line
	&et
	// &haar,
	// &soil1,
	// &apogeeSolar,
	
	// &soil2,
	// &soil3,
	// &gas,
	// &apogeeO2,
};
/////////////////////////// END USER CONFIG /////////////////////////////////

namespace PinsIO { //For Kestrel v1.1
	constexpr uint16_t VUSB = 5;
}

namespace PinsIOAlpha {
	constexpr uint16_t I2C_EXT_EN = 10;
	constexpr uint16_t SD_CD = 8;
	constexpr uint16_t SD_EN = 12;
	constexpr uint16_t AUX_EN = 15;
	constexpr uint16_t CE = 11;
	constexpr uint16_t LED_EN = 13;
}

namespace PinsIOBeta { //For Kestrel v1.1
	constexpr uint16_t SEL1 = 0;
	constexpr uint16_t SEL2 = 4;
	constexpr uint16_t SEL3 = 8;
	constexpr uint16_t SEL4 = 12;
	constexpr uint16_t I2C_EN1 = 1;
	constexpr uint16_t I2C_EN2 = 5;
	constexpr uint16_t I2C_EN3 = 9;
	constexpr uint16_t I2C_EN4 = 13;
	constexpr uint16_t EN1 = 3;
	constexpr uint16_t EN2 = 7;
	constexpr uint16_t EN3 = 11;
	constexpr uint16_t EN4 = 15;
	constexpr uint16_t FAULT1 = 2;
	constexpr uint16_t FAULT2 = 6;
	constexpr uint16_t FAULT3 = 10;
	constexpr uint16_t FAULT4 = 14;
}



// SYSTEM_MODE(MANUAL); //User must call Particle.process() to stay connected to cellular after conecting, not recommended for use.
SYSTEM_MODE(SEMI_AUTOMATIC); //Particle will wait until told to connect to Cellular, but try to stay connected once connected.
// SYSTEM_MODE(AUTOMATIC); //Particle automatically tries to connect to Cellular, once connected, user code starts running.

SYSTEM_THREAD(ENABLED); //SYSTEM_THREAD enabled means Network processign runs on different thread than user loop code, recommended for use.
// SYSTEM_THREAD(DISABLED); 

int detectTalons(String dummyStr = "");
int detectSensors(String dummyStr = "");

String diagnostic = "";
String errors = "";
String metadata = "";
String data = "";

void setup() {
	configurePowerSave(desiredPowerSaveMode); //Setup power mode of the system (Talons and Sensors)
	System.enableFeature(FEATURE_RESET_INFO); //Allows for Particle to see reason for last reset using System.resetReason();
	if(System.resetReason() != RESET_REASON_POWER_DOWN) {
		//DEBUG! Set safe mode 
		Particle.connect(); //DEBUG! //If reset not caused by power switch, assume something bad happened, just connect to particle straight away
	}
	//////////// MANUAL POSITIONING //////////////////
  	// talons[aux.getTalonPort()] = &aux; //Place talon objects at coresponding positions in array
	// talons[aux1.getTalonPort()] = &aux1;
	time_t startTime = millis();
	Particle.function("nodeID", setNodeID);
	Particle.function("findSensors", detectSensors);
	Particle.function("findTalons", detectTalons);
	Particle.function("systemRestart", systemRestart);
	Particle.function("takeSample", takeSample);
	Particle.function("commandExe", commandExe);
	Serial.begin(1000000); 
	waitFor(serialConnected, 10000); //DEBUG! Wait until serial starts sending or 10 seconds 
	Serial.print("RESET CAUSE: "); //DEBUG!
	Serial.println(System.resetReason()); //DEBUG!
	bool hasCriticalError = false;
	bool hasError = false;
	// logger.begin(Time.now(), hasCriticalError, hasError); //Needs to be called the first time with Particle time since I2C not yet initialized 
	logger.begin(0, hasCriticalError, hasError); //Called with 0 since time collection system has not been initialized 
	logger.setIndicatorState(IndicatorLight::ALL,IndicatorMode::INIT);
	bool batState = logger.testForBat(); //Check if a battery is connected
	logger.enableI2C_OB(false);
	logger.enableI2C_External(true); //Connect to Gonk I2C port
	logger.enableI2C_Global(true);
	if(batState) battery.setIndicatorState(GonkIndicatorMode::SOLID); //Turn on charge indication LEDs during setup 
	else battery.setIndicatorState(GonkIndicatorMode::BLINKING); //If battery not switched on, set to blinking 
	fileSys.begin(0, hasCriticalError, hasError); //Initialzie, but do not attempt backhaul
	if(hasCriticalError) {
		Serial.println(getErrorString()); //Report current error codes
		logger.setIndicatorState(IndicatorLight::STAT,IndicatorMode::ERROR); //Display error state if critical error is reported 
	}
	else logger.setIndicatorState(IndicatorLight::STAT,IndicatorMode::PASS); //If no critical fault, switch STAT off
	battery.begin(0, hasCriticalError, hasError); //Init final CORE element
	//   I2C_OnBoardEn(true); 	
	// Wire.setClock(400000); //Confirm operation in fast mode
	// Wire.begin();
	logger.enableI2C_Global(false); //Connect to internal bus
	logger.enableI2C_OB(true);
	ioAlpha.begin(); //RESTORE
	ioBeta.begin(); //RESTORE
	// ioBeta.pinMode(PinsIOBeta::SEL2, OUTPUT); //DEBUG
	// ioBeta.digitalWrite(PinsIOBeta::SEL2, LOW); //DEBUG
	ioAlpha.pinMode(PinsIOAlpha::LED_EN, OUTPUT);
	ioAlpha.digitalWrite(PinsIOAlpha::LED_EN, LOW); //Turn on LED indicators 
	// logger.setIndicatorState(IndicatorLight::ALL,IndicatorMode::IDLE);
	// waitFor(serialConnected, 10000); //DEBUG! Wait until serial starts sending or 10 seconds
	if(Serial.available()) {
		//COMMAND MODE!
		logger.setIndicatorState(IndicatorLight::ALL,IndicatorMode::COMMAND);
		systemConfig();
	}
	logger.setIndicatorState(IndicatorLight::ALL,IndicatorMode::WAITING);
	Particle.connect(); //Once passed attempted serial connect, try to connect to particle 

	////////// ADD INTERRUPTS!
	// for(int i = 1; i <= Kestrel::numTalonPorts; i++) { //Iterate over ALL ports
	// 	logger.enablePower(i, true); //Turn on all power by default
	// 	logger.enablePower(i, false); //Toggle power to reset
	// 	logger.enablePower(i, true); 
	// 	logger.enableData(i, false); //Turn off all data by default
	// }

	detectTalons();
	detectSensors();

	// I2C_OnBoardEn(false);	
	// I2C_GlobalEn(true);

	// bool hasCriticalError = false;
	// bool hasError = false;
	// logger.enableData(4, true);
	// logger.enableI2C_OB(false);
	// logger.enableI2C_Global(true);
	// String initDiagnostic = aux.begin(Time.now(), hasCriticalError, hasError);
	logger.updateLocation(true); //Force GPS update (hopfully connected)
	String initDiagnostic = initSensors();
	Serial.print("DIAGNOSTIC: ");
	Serial.println(initDiagnostic);
	if(loggingMode == LogModes::NO_LOCAL) {
		fileSys.writeToFRAM(initDiagnostic, DataType::Diagnostic, DestCodes::Particle);
		logEvents(4, DestCodes::Particle); //Grab data log with metadata, no diagnostics 
	}
	else {
		fileSys.writeToFRAM(initDiagnostic, DataType::Diagnostic, DestCodes::Both);
		logEvents(4, DestCodes::Both); //Grab data log with metadata, no diagnostics 
	}
	// fileSys.writeToParticle(initDiagnostic, "diagnostic"); 
	// // logger.enableSD(true);
	// fileSys.writeToSD(initDiagnostic, "Dummy.txt");

	#ifndef RAPID_START  //Only do this if not rapid starting
	while((!Particle.connected() || logger.gps.getFixType() == 0) && (millis() - startTime) < maxConnectTime) { //Wait while at least one of the remote systems is not connected 
		if(Particle.connected()) {
			logger.setIndicatorState(IndicatorLight::CELL, IndicatorMode::PASS); //If cell is connected, set to PASS state
			if(WAIT_GPS == false) break; //If not told to wait for GPS, break out after cell is connected 
		}
		if(logger.gps.getTimeValid() == true) {
			if(logger.gps.getFixType() >= 2 && logger.gps.getFixType() <= 4 && logger.gps.getGnssFixOk()) { //If you get a 2D fix or better, pass GPS 
				logger.setIndicatorState(IndicatorLight::GPS, IndicatorMode::PASS); 
			}
			else {
				logger.setIndicatorState(IndicatorLight::GPS, IndicatorMode::PREPASS); //If time is good, set preliminary pass only
			}
		}
		Serial.println("Wait for cell connect..."); //DEBUG!
		delay(5000); //Wait 5 seconds between each check to not lock up the process //DEBUG!
	}
	#endif
	
	if(Particle.connected()) logger.setIndicatorState(IndicatorLight::CELL, IndicatorMode::PASS); //Catches connection of cell is second device to connect
	else {
		logger.setIndicatorState(IndicatorLight::CELL, IndicatorMode::ERROR); //If cell still not connected, display error
		// Particle.disconnect(); //DEBUG!
	}
	if(logger.gps.getFixType() >= 2 && logger.gps.getFixType() <= 4 && logger.gps.getGnssFixOk()) { //Make fix report is in range and fix is OK
		logger.setIndicatorState(IndicatorLight::GPS, IndicatorMode::PASS); //Catches connection of GPS is second device to connect
	}
	else {
		logger.setIndicatorState(IndicatorLight::GPS, IndicatorMode::ERROR); //If GPS fails to connect after period, set back to error
	}
	fileSys.tryBackhaul(); //See if we can backhaul any unsent logs

	// fileSys.writeToFRAM(getDiagnosticString(1), DataType::Diagnostic, DestCodes::Both); //DEBUG!
	// logEvents(3); //Grab data log with metadata //DEBUG!
	fileSys.dumpFRAM(); //Backhaul this data right away
	// Particle.publish("diagnostic", initDiagnostic);

	// logger.enableData(3, true);
	// logger.enableI2C_OB(false);
	// logger.enableI2C_Global(true);
	// aux1.begin(Time.now(), hasCriticalError, hasError);
  	//FIX! RESPOND TO ERROR RESULTS! 
}

void loop() {
  // aux.sleep(false);
  static int count = 1; //Keep track of number of cycles
	logger.wake(); //Wake up logger system
	fileSys.wake(); //Wake up file handling 
	wakeSensors(); //Wake each sensor
	if(System.millis() > indicatorTimeout) {
		logger.setIndicatorState(IndicatorLight::ALL, IndicatorMode::NONE); //Turn LED indicators off if it has been longer than timeout since startup (use system.millis() which does not rollover)
		logger.enableI2C_OB(false);
		logger.enableI2C_External(true); //Connect to Gonk I2C port
		logger.enableI2C_Global(true);
		battery.setIndicatorState(GonkIndicatorMode::PUSH_BUTTON); //Turn off indicator lights on battery, return to push button control
		logger.enableI2C_External(false); //Turn off external I2C
	}
	bool alarm = logger.waitUntilTimerDone(); //Wait until the timer period has finished  //REPLACE FOR NON-SLEEP
	// if(alarm) Serial.println("RTC Wakeup"); //DEBUG!
	// else Serial.println("Timeout Wakeup"); //DEBUG!
	// Serial.print("RAM, Start Log Events: "); //DEBUG!
	// Serial.println(System.freeMemory()); //DEBUG!
	logger.startTimer(logPeriod); //Start timer as soon done reading sensors //REPLACE FOR NON-SLEEP
	switch(loggingMode) {
		static uint64_t lastDiagnostic = System.millis(); 
		case (LogModes::PERFORMANCE):
			logEvents(6, DestCodes::Both);
			break;
		case (LogModes::STANDARD):
			if((count % 16) == 0) logEvents(3, DestCodes::Both);
			else if((count % 8) == 0) logEvents(2, DestCodes::Both);
			else if((count % 1) == 0) logEvents(1, DestCodes::Both);
			break;
		case (LogModes::BALANCED):
			logEvents(7, DestCodes::Both);
			if((System.millis() - lastDiagnostic) > balancedDiagnosticPeriod) {
				logEvents(3, DestCodes::Both); //Do a full diagnostic and metadata report once an hour
				lastDiagnostic = System.millis();
			}
			break;
		case (LogModes::NO_LOCAL):
			if((count % 10) == 0) logEvents(3, DestCodes::Particle);
			else if((count % 5) == 0) logEvents(2, DestCodes::Particle);
			else if((count % 1) == 0) logEvents(1, DestCodes::Particle);
			break;
		default:
			logEvents(1, DestCodes::Both); //If unknown configuration, use general call 
			// break;
	}
	
	
	// Serial.print("RAM, End Log Events: "); //DEBUG!
	// Serial.println(System.freeMemory()); //DEBUG!
	Serial.println("Log Done"); //DEBUG!
	Serial.print("WDT Status: "); //DEBUG!
	Serial.println(logger.feedWDT()); 
	sleepSensors();
	
	
	// Particle.publish("diagnostic", diagnostic);
	// Particle.publish("error", errors);
	// Particle.publish("data", data);
	// Particle.publish("metadata", metadata);

	// Serial.print("DIAGNOSTIC: ");
	// Serial.println(diagnostic);
	// Serial.print("ERROR: ");
	// Serial.println(errors);
	// Serial.print("DATA: ");
	// Serial.println(data);
	// Serial.print("METADATA: ");
	// Serial.println(metadata);

	// logger.enableI2C_OB(true);
	// logger.enableI2C_Global(false);
	// fileSys.writeToFRAM(diagnostic, "diagnostic", DestCodes::Particle);

	if((count % backhaulCount) == 0) {
		Serial.println("BACKHAUL"); //DEBUG!
		if(powerSaveMode >= PowerSaveModes::LOW_POWER) {
			Particle.connect();
			waitFor(Particle.connected, 300000); //Wait up to 5 minutes to connect if using low power modes
		}
		logger.syncTime();
		fileSys.dumpFRAM(); //dump FRAM every Nth log
	}
	count++;
	fileSys.sleep(); //Wait to sleep until after backhaul attempt
	logger.sleep(); //Put system into sleep mode

	// SystemSleepConfiguration config;
	// config.mode(SystemSleepMode::STOP)
    //   .network(NETWORK_INTERFACE_CELLULAR)
    //   .flag(SystemSleepFlag::WAIT_CLOUD)
    //   .duration(2min);
	// System.sleep(config);
}

void logEvents(uint8_t type, uint8_t destination)
{
	
	// String diagnostic = "";
	// String errors = "";
	// String metadata = "";
	// String data = "";
	diagnostic = "";
	errors = "";
	metadata = "";
	data = "";
	Serial.print("LOG: "); //DEBUG!
	Serial.println(type); 
	if(type == 0) { //Grab errors only
		// data = getDataString();
		// diagnostic = getDiagnosticString(4); //DEBUG! RESTORE
		errors = getErrorString(); //Get errors last to wait for error codes to be updated //DEBUG! RESTORE
		// logger.enableI2C_OB(true);
		// logger.enableI2C_Global(false);
		Serial.println(errors); //DEBUG!
		// Serial.println(data); //DEBUG!
		// Serial.println(diagnostic); //DEBUG!

		if(errors.equals("") == false) {
			// Serial.println("Write Errors to FRAM"); //DEBUG!
			fileSys.writeToFRAM(errors, DataType::Error, destination); //Write value out only if errors are reported 
		}
		// fileSys.writeToFRAM(data, DataType::Data, DestCodes::Both);
		// fileSys.writeToFRAM(diagnostic, DataType::Diagnostic, DestCodes::Both);
	}
	if(type == 1) {
		data = getDataString();
		diagnostic = getDiagnosticString(4); //DEBUG! RESTORE
		errors = getErrorString(); //Get errors last to wait for error codes to be updated //DEBUG! RESTORE
		// logger.enableI2C_OB(true);
		// logger.enableI2C_Global(false);
		Serial.println(errors); //DEBUG!
		Serial.println(data); //DEBUG!
		Serial.println(diagnostic); //DEBUG!

		if(errors.equals("") == false) {
			// Serial.println("Write Errors to FRAM"); //DEBUG!
			fileSys.writeToFRAM(errors, DataType::Error, destination); //Write value out only if errors are reported 
		}
		fileSys.writeToFRAM(data, DataType::Data, destination);
		fileSys.writeToFRAM(diagnostic, DataType::Diagnostic, destination);
	}
	else if(type == 2) {
		data = getDataString();
		diagnostic = getDiagnosticString(3);
		errors = getErrorString();
		// logger.enableI2C_OB(true);
		// logger.enableI2C_Global(false);
		if(errors.equals("") == false) fileSys.writeToFRAM(errors, DataType::Error, destination); //Write value out only if errors are reported 
		fileSys.writeToFRAM(data, DataType::Data, destination);
		fileSys.writeToFRAM(diagnostic, DataType::Diagnostic, destination);
	}
	else if(type == 3) {
		data = getDataString();
		diagnostic = getDiagnosticString(2);
		metadata = getMetadataString();
		errors = getErrorString();
		// logger.enableI2C_OB(true);
		// logger.enableI2C_Global(false);
		if(errors.equals("") == false) fileSys.writeToFRAM(errors, DataType::Error, destination); //Write value out only if errors are reported 
		fileSys.writeToFRAM(data, DataType::Data, destination);
		fileSys.writeToFRAM(diagnostic, DataType::Diagnostic, destination);
		fileSys.writeToFRAM(metadata, DataType::Metadata, destination);
	}
	else if(type == 4) { //To be used on startup, don't grab diagnostics since init already got them
		data = getDataString();
		// diagnostic = getDiagnosticString(2);
		metadata = getMetadataString();
		errors = getErrorString();
		// logger.enableI2C_OB(true);
		// logger.enableI2C_Global(false);
		Serial.println(errors); //DEBUG!
		Serial.println(data); //DEBUG
		Serial.println(metadata); //DEBUG!
		if(errors.equals("") == false) fileSys.writeToFRAM(errors, DataType::Error, destination); //Write value out only if errors are reported 
		fileSys.writeToFRAM(data, DataType::Data, destination);
		// fileSys.writeToFRAM(diagnostic, DataType::Diagnostic, DestCodes::Both);
		fileSys.writeToFRAM(metadata, DataType::Metadata, destination);
	}
	else if(type == 5) { //To be used on startup, don't grab diagnostics since init already got them
		data = getDataString();
		diagnostic = getDiagnosticString(5);
		// metadata = getMetadataString();
		errors = getErrorString();
		// logger.enableI2C_OB(true);
		// logger.enableI2C_Global(false);
		Serial.println(errors); //DEBUG!
		Serial.println(data); //DEBUG
		// Serial.println(metadata); //DEBUG!
		if(errors.equals("") == false) fileSys.writeToFRAM(errors, DataType::Error, DestCodes::SD); //Write value out only if errors are reported 
		fileSys.writeToFRAM(data, DataType::Data, DestCodes::SD);
		fileSys.writeToFRAM(diagnostic, DataType::Diagnostic, DestCodes::SD);
		// fileSys.writeToFRAM(metadata, DataType::Metadata, DestCodes::Both);
	}
	else if(type == 6) { //Log ONLY data - fastest method
		data = getDataString();
		// diagnostic = getDiagnosticString(5);
		// metadata = getMetadataString();
		// errors = getErrorString();
		// logger.enableI2C_OB(true);
		// logger.enableI2C_Global(false);
		// Serial.println(errors); //DEBUG!
		// Serial.println(data); //DEBUG
		// Serial.println(metadata); //DEBUG!
		// if(errors.equals("") == false) fileSys.writeToFRAM(errors, DataType::Error, DestCodes::SD); //Write value out only if errors are reported 
		fileSys.writeToFRAM(data, DataType::Data, destination);
		// fileSys.writeToFRAM(diagnostic, DataType::Diagnostic, DestCodes::SD);
		// fileSys.writeToFRAM(metadata, DataType::Metadata, DestCodes::Both);
	}
	else if(type == 7) { //Log data and error if there
		data = getDataString();
		// diagnostic = getDiagnosticString(5);
		// metadata = getMetadataString();
		errors = getErrorString();
		// logger.enableI2C_OB(true);
		// logger.enableI2C_Global(false);
		// Serial.println(errors); //DEBUG!
		// Serial.println(data); //DEBUG
		// Serial.println(metadata); //DEBUG!
		if(errors.equals("") == false) fileSys.writeToFRAM(errors, DataType::Error, destination); //Write value out only if errors are reported 
		fileSys.writeToFRAM(data, DataType::Data, destination);
		// fileSys.writeToFRAM(diagnostic, DataType::Diagnostic, DestCodes::SD);
		// fileSys.writeToFRAM(metadata, DataType::Metadata, DestCodes::Both);
	}
	// switch(type) {
	// 	case 1: //Standard, short interval, log
			
	// 		data = getDataString();
	// 		diagnostic = getDiagnosticString(4); //DEBUG! RESTORE
	// 		errors = getErrorString(); //Get errors last to wait for error codes to be updated //DEBUG! RESTORE
	// 		// logger.enableI2C_OB(true);
	// 		// logger.enableI2C_Global(false);
	// 		Serial.println(errors); //DEBUG!
	// 		Serial.println(data); //DEBUG!
	// 		Serial.println(diagnostic); //DEBUG!

	// 		if(errors.equals("") == false) {
	// 			// Serial.println("Write Errors to FRAM"); //DEBUG!
	// 			fileSys.writeToFRAM(errors, DataType::Error, DestCodes::Both); //Write value out only if errors are reported 
	// 		}
	// 		fileSys.writeToFRAM(data, DataType::Data, DestCodes::Both);
	// 		fileSys.writeToFRAM(diagnostic, DataType::Diagnostic, DestCodes::Both);
	// 	break;

	// 	case 2: //Low period log with diagnostics
	// 		data = getDataString();
	// 		diagnostic = getDiagnosticString(3);
	// 		errors = getErrorString();
	// 		// logger.enableI2C_OB(true);
	// 		// logger.enableI2C_Global(false);
	// 		if(errors.equals("") == false) fileSys.writeToFRAM(errors, DataType::Error, DestCodes::Both); //Write value out only if errors are reported 
	// 		fileSys.writeToFRAM(data, DataType::Data, DestCodes::Both);
	// 		fileSys.writeToFRAM(diagnostic, DataType::Diagnostic, DestCodes::Both);
	// 	break;

	// 	case 3: //Daily log event with increased diagnostics and metadata
	// 		data = getDataString();
	// 		diagnostic = getDiagnosticString(2);
	// 		metadata = getMetadataString();
	// 		errors = getErrorString();
	// 		// logger.enableI2C_OB(true);
	// 		// logger.enableI2C_Global(false);
	// 		if(errors.equals("") == false) fileSys.writeToFRAM(errors, DataType::Error, DestCodes::Both); //Write value out only if errors are reported 
	// 		fileSys.writeToFRAM(data, DataType::Data, DestCodes::Both);
	// 		fileSys.writeToFRAM(diagnostic, DataType::Diagnostic, DestCodes::Both);
	// 		fileSys.writeToFRAM(metadata, DataType::Metadata, DestCodes::Both);
	// 	break;

	// }
	// diagnostic* = (const char*)NULL;
	// data* = (const char*)NULL;
	// metadata* = (const char*)NULL;
	// errors* = (const char*)NULL;
	
}
String getErrorString()
{
	unsigned long numErrors = 0; //Used to keep track of total errors across all devices 
	String errors = "{\"Error\":{";
	errors = errors + "\"Time\":" + logger.getTimeString() + ","; //Concatonate time
	errors = errors + "\"Loc\":[" + logger.getPosLat() + "," + logger.getPosLong() + "," + logger.getPosAlt() + "," + logger.getPosTimeString() + "],";
	if(globalNodeID != "") errors = errors + "\"Node ID\":\"" + globalNodeID + "\","; //Concatonate node ID
	else errors = errors + "\"Device ID\":\"" + System.deviceID() + "\","; //If node ID not initialized, use device ID
	errors = errors + "\"Packet ID\":" + logger.getMessageID() + ","; //Concatonate unique packet hash
	errors = errors + "\"NumDevices\":" + String(numSensors) + ","; //Concatonate number of sensors 
	errors = errors + "\"Devices\":[";
	for(int i = 0; i < numSensors; i++) {
		if(sensors[i]->totalErrors() > 0) {
			numErrors = numErrors + sensors[i]->totalErrors(); //Increment the total error count
			if(!errors.endsWith("[")) errors = errors + ","; //Only append if not first entry
			errors = errors + "{" + sensors[i]->getErrors() + "}";
		}
	}
	errors = errors + "]}}"; //Close data
	Serial.print("Num Errors: "); //DEBUG!
	Serial.println(numErrors); 
	if(numErrors > 0) return errors;
	else return ""; //Return null string if no errors reported 
}

String getDataString()
{
	String leader = "{\"Data\":{";
	leader = leader + "\"Time\":" + logger.getTimeString() + ","; //Concatonate time
	leader = leader + "\"Loc\":[" + logger.getPosLat() + "," + logger.getPosLong() + "," + logger.getPosAlt() + "," + logger.getPosTimeString() + "],";
	if(globalNodeID != "") leader = leader + "\"Node ID\":\"" + globalNodeID + "\","; //Concatonate node ID
	else leader = leader + "\"Device ID\":\"" + System.deviceID() + "\","; //If node ID not initialized, use device ID
	leader = leader + "\"Packet ID\":" + logger.getMessageID() + ","; //Concatonate unique packet hash
	leader = leader + "\"NumDevices\":" + String(numSensors) + ","; //Concatonate number of sensors 
	leader = leader + "\"Devices\":[";
	const String closer = "]}}";
	String output = leader;

	uint8_t deviceCount = 0; //Used to keep track of how many devices have been appended 
	for(int i = 0; i < numSensors; i++) {
		logger.disableDataAll(); //Turn off data to all ports, then just enable those needed
		if(sensors[i]->sensorInterface != BusType::CORE && sensors[i]->getTalonPort() != 0) logger.enablePower(sensors[i]->getTalonPort(), true); //Turn on kestrel port for needed Talon, only if not core system and port is valid
		if(sensors[i]->sensorInterface != BusType::CORE && sensors[i]->getTalonPort() != 0) logger.enableData(sensors[i]->getTalonPort(), true); //Turn on kestrel port for needed Talon, only if not core system and port is valid
		logger.enableI2C_OB(false);
		logger.enableI2C_Global(true);
		bool dummy1;
		bool dummy2;
		
		if(sensors[i]->getTalonPort() > 0 && talons[sensors[i]->getTalonPort() - 1]) { //DEBUG! REPALCE!
			Serial.print("TALON CALL: "); //DEBUG!
			Serial.println(sensors[i]->getTalonPort());
			logger.configTalonSense(); //Setup to allow for current testing
			// talons[sensors[i]->getTalonPort() - 1]->begin(logger.getTime(), dummy1, dummy2); //DEBUG! Do only if talon is associated with sensor, and object exists 
			talons[sensors[i]->getTalonPort() - 1]->restart(); //DEBUG! Do only if talon is associated with sensor, and object exists 
			// logger.enableI2C_OB(false); //Return to isolation mode
			// logger.enableI2C_Global(true);
		}
		if(sensors[i]->getSensorPort() > 0 && sensors[i]->getTalonPort() > 0) { //If not a Talon
			Serial.print("Device "); //DEBUG!
			Serial.print(i);
			Serial.println(" is a sensor");
			talons[sensors[i]->getTalonPort() - 1]->disableDataAll(); //Turn off all data ports to start for the given Talon
			// talons[sensors[i]->getTalonPort() - 1]->disablePowerAll(); //Turn off all power ports to start for the given Talon
			// talons[sensors[i]->getTalonPort() - 1]->enablePower(sensors[i]->getSensorPort(), true); //Turn on power for the given port on the Talon
			talons[sensors[i]->getTalonPort() - 1]->enableData(sensors[i]->getSensorPort(), true); //Turn on data for the given port on the Talon
			// bool dummy1;
			// bool dummy2;
			// sensors[i]->begin(Time.now(), dummy1, dummy2); //DEBUG!
		}
		// delay(100); //DEBUG!
		logger.enableI2C_OB(false);
		logger.enableI2C_Global(true);
		Serial.print("Data string from sensor "); //DEBUG!
		Serial.print(i);
		Serial.print(": ");
		String val = sensors[i]->getData(logger.getTime());
		Serial.println(val);
		if(!val.equals("")) {  //Only append if not empty string
			if(output.length() - output.lastIndexOf('\n') + val.length() + closer.length() + 1 < Kestrel::MAX_MESSAGE_LENGTH) { //Add +1 to account for comma appending, subtract any previous lines from count
				if(deviceCount > 0) output = output + ","; //Add preceeding comma if not the first entry
				output = output + "{" + val + "}"; //Append result 
				deviceCount++;
				// if(i + 1 < numSensors) diagnostic = diagnostic + ","; //Only append if not last entry
			}
			else {
				output = output + closer + "\n"; //End this packet
				output = output + leader + "{" + val + "}"; //Start a new packet and add new payload 
			}
		}
		// if(!val.equals("")) { //Only append if real result
		// 	if(deviceCount > 0) data = data + ","; //Preappend comma only if not first addition
		// 	data = data + "{" + val + "}";
		// 	deviceCount++;
		// 	// if(i + 1 < numSensors) metadata = metadata + ","; //Only append if not last entry
		// }
		Serial.print("Cumulative data string: "); //DEBUG!
		Serial.println(output); //DEBUG!
		// data = data + sensors[i]->getData(logger.getTime()); //DEBUG! REPLACE!
		// if(i + 1 < numSensors) data = data + ","; //Only append if not last entry
		if(sensors[i]->getSensorPort() > 0 && sensors[i]->getTalonPort() > 0) {
			talons[sensors[i]->getTalonPort() - 1]->enableData(sensors[i]->getSensorPort(), false); //Turn off data for the given port on the Talon
			// talons[sensors[i]->getTalonPort() - 1]->enablePower(sensors[i]->getSensorPort(), false); //Turn off power for the given port on the Talon //DEBUG!
		}
	}
	output = output + "]}}"; //Close data
	return output;
}

String getDiagnosticString(uint8_t level)
{
	String leader = "{\"Diagnostic\":{";
	leader = leader + "\"Time\":" + logger.getTimeString() + ","; //Concatonate time
	leader = leader + "\"Loc\":[" + logger.getPosLat() + "," + logger.getPosLong() + "," + logger.getPosAlt() + "," + logger.getPosTimeString() + "],";
	if(globalNodeID != "") leader = leader + "\"Node ID\":\"" + globalNodeID + "\","; //Concatonate node ID
	else leader = leader + "\"Device ID\":\"" + System.deviceID() + "\","; //If node ID not initialized, use device ID
	leader = leader + "\"Packet ID\":" + logger.getMessageID() + ","; //Concatonate unique packet hash
	leader = leader + "\"NumDevices\":" + String(numSensors) + ",\"Level\":" + String(level) + ",\"Devices\":["; //Concatonate number of sensors and level 
	const String closer = "]}}";
	String output = leader;

	uint8_t deviceCount = 0; //Used to keep track of how many devices have been appended 
	for(int i = 0; i < numSensors; i++) {
		logger.disableDataAll(); //Turn off data to all ports, then just enable those needed
		if(sensors[i]->sensorInterface != BusType::CORE && sensors[i]->getTalonPort() != 0) logger.enablePower(sensors[i]->getTalonPort(), true); //Turn on kestrel port for needed Talon, only if not core system and port is valid
		if(sensors[i]->sensorInterface != BusType::CORE && sensors[i]->getTalonPort() != 0) logger.enableData(sensors[i]->getTalonPort(), true); //Turn on kestrel port for needed Talon, only if not core system and port is valid
		logger.enableI2C_OB(false);
		logger.enableI2C_Global(true);
		// if(!sensors[i]->isTalon()) { //If sensor is not Talon
		if(sensors[i]->getSensorPort() > 0 && sensors[i]->getTalonPort() > 0) { //If a Talon is associated with the sensor, turn that port on
			talons[sensors[i]->getTalonPort() - 1]->disableDataAll(); //Turn off all data on Talon
			// talons[sensors[i]->getTalonPort() - 1]->enablePower(sensors[i]->getSensorPort(), true); //Turn on power for the given port on the Talon
			talons[sensors[i]->getTalonPort() - 1]->enableData(sensors[i]->getSensorPort(), true); //Turn back on only port used
			
		}

  		String diagnostic = sensors[i]->selfDiagnostic(level, logger.getTime());
		if(!diagnostic.equals("")) {  //Only append if not empty string
			if(output.length() - output.lastIndexOf('\n') + diagnostic.length() + closer.length() + 1 < Kestrel::MAX_MESSAGE_LENGTH) { //Add +1 to account for comma appending, subtract any previous lines from count
				if(deviceCount > 0) output = output + ","; //Add preceeding comma if not the first entry
				output = output + "{" + diagnostic + "}"; //Append result 
				deviceCount++;
				// if(i + 1 < numSensors) diagnostic = diagnostic + ","; //Only append if not last entry
			}
			else {
				output = output + closer + "\n"; //End this packet
				output = output + leader + "{" + diagnostic + "}"; //Start a new packet and add new payload 
			}
		}

		if(sensors[i]->getSensorPort() > 0 && sensors[i]->getTalonPort() > 0) {
			talons[sensors[i]->getTalonPort() - 1]->enableData(sensors[i]->getSensorPort(), false); //Turn off data for the given port on the Talon
			// talons[sensors[i]->getTalonPort() - 1]->enablePower(sensors[i]->getSensorPort(), false); //Turn off power for the given port on the Talon //DEBUG!
		}
		
	}
	output = output + closer; //Close diagnostic
	return output;
}

String getMetadataString()
{
	String leader = "{\"Metadata\":{";
	leader = leader + "\"Time\":" + logger.getTimeString() + ","; //Concatonate time
	leader = leader + "\"Loc\":[" + logger.getPosLat() + "," + logger.getPosLong() + "," + logger.getPosAlt() + "," + logger.getPosTimeString() + "],";
	if(globalNodeID != "") leader = leader + "\"Node ID\":\"" + globalNodeID + "\","; //Concatonate node ID
	else leader = leader + "\"Device ID\":\"" + System.deviceID() + "\","; //If node ID not initialized, use device ID
	leader = leader + "\"Packet ID\":" + logger.getMessageID() + ","; //Concatonate unique packet hash
	leader = leader + "\"NumDevices\":" + String(numSensors) + ","; //Concatonate number of sensors 
	leader = leader + "\"Devices\":[";
	const String closer = "]}}";
	String output = leader;
	
	output = output + "{\"System\":{";
	// output = output + "\"DUMMY\":\"BLOODYMARYBLOODYMARYBLODDYMARY\",";
	output = output + "\"Schema\":\"" + schemaVersion + "\",";
	output = output + "\"Firm\":\"" + firmwareVersion + "\",";
	output = output + "\"OS\":\"" + System.version() + "\",";
	output = output + "\"ID\":\"" + System.deviceID() + "\",";
	output = output + "\"Update\":" + String(logPeriod) + ",";
	output = output + "\"Backhaul\":" + String(backhaulCount) + ",";
	output = output + "\"LogMode\":" + String(loggingMode) + ",";
	output = output + "\"Sleep\":" + String(powerSaveMode) + "}},";
	//FIX! Add support for device name 
	
	uint8_t deviceCount = 0; //Used to keep track of how many devices have been appended 
	for(int i = 0; i < numSensors; i++) {
		logger.disableDataAll(); //Turn off data to all ports, then just enable those needed
		if(sensors[i]->sensorInterface != BusType::CORE && sensors[i]->getTalonPort() != 0) logger.enableData(sensors[i]->getTalonPort(), true); //Turn on data to required Talon port only if not core and port is valid
			// if(!sensors[i]->isTalon()) { //If sensor is not Talon
		if(sensors[i]->getSensorPort() > 0 && sensors[i]->getTalonPort() > 0) { //If a Talon is associated with the sensor, turn that port on
			talons[sensors[i]->getTalonPort() - 1]->disableDataAll(); //Turn off all data on Talon
			talons[sensors[i]->getTalonPort() - 1]->enableData(sensors[i]->getSensorPort(), true); //Turn back on only port used
		}
		// logger.enablePower(sensors[i]->getTalon(), true); //Turn on power to port
		// logger.enableData(sensors[i]->getTalon(), true); //Turn on data to port
		logger.enableI2C_OB(false);
		logger.enableI2C_Global(true);
		String val = sensors[i]->getMetadata();
		// metadata = metadata + sensors[i]->getMetadata();
		// if(!val.equals("")) { //Only append if real result
		// 	if(deviceCount > 0) metadata = metadata + ","; //Preappend comma only if not first addition
		// 	metadata = metadata + val;
		// 	deviceCount++;
		// 	// if(i + 1 < numSensors) metadata = metadata + ","; //Only append if not last entry
		// }
		if(!val.equals("")) {  //Only append if not empty string
			if(output.length() - output.lastIndexOf('\n') + val.length() + closer.length() + 1 < Kestrel::MAX_MESSAGE_LENGTH) { //Add +1 to account for comma appending, subtract any previous lines from count
				if(deviceCount > 0) output = output + ","; //Add preceeding comma if not the first entry
				output = output + "{" + val + "}"; //Append result 
				deviceCount++;
				// if(i + 1 < numSensors) diagnostic = diagnostic + ","; //Only append if not last entry
			}
			else {
				output = output + closer + "\n"; //End this packet
				output = output + leader + "{" + val + "}"; //Start a new packet and add new payload 
			}
		}
	}

	output = output + closer; //Close metadata
	return output;
}

String initSensors()
{
	String leader = "{\"Diagnostic\":{";
	leader = leader + "\"Time\":" + logger.getTimeString() + ","; //Concatonate time
	leader = leader + "\"Loc\":[" + logger.getPosLat() + "," + logger.getPosLong() + "," + logger.getPosAlt() + "," + logger.getPosTimeString() + "],";
	if(globalNodeID != "") leader = leader + "\"Node ID\":\"" + globalNodeID + "\","; //Concatonate node ID
	else leader = leader + "\"Device ID\":\"" + System.deviceID() + "\","; //If node ID not initialized, use device ID
	leader = leader + "\"Packet ID\":" + logger.getMessageID() + ","; //Concatonate unique packet hash
	leader = leader + "\"NumDevices\":" + String(numSensors) + ",\"Devices\":["; //Concatonate number of sensors and level 
	
	String closer = "]}}";
	String output = leader;
	bool reportCriticalError = false; //Used to keep track of the global status of the error indications for all sensors
	bool reportError = false;
	bool missingSensor = false;
	// output = output + "\"Devices\":[";
	uint8_t deviceCount = 0; //Used to keep track of how many devices have been appended 
	for(int i = 0; i < numSensors; i++) {
		logger.disableDataAll(); //Turn off data to all ports, then just enable those needed
		if(sensors[i]->sensorInterface != BusType::CORE && sensors[i]->getTalonPort() != 0) logger.enableData(sensors[i]->getTalonPort(), true); //Turn on data to required Talon port only if not core and the port is valid
		logger.enableI2C_OB(false);
		logger.enableI2C_Global(true);
		bool dummy1;
		bool dummy2;
		// if(!sensors[i]->isTalon()) { //If sensor is not Talon
		logger.configTalonSense(); //Setup to allow for current testing
		// if(sensors[i]->getTalonPort() > 0 && talons[sensors[i]->getTalonPort() - 1]) talons[sensors[i]->getTalonPort() - 1]->begin(logger.getTime(), dummy1, dummy2); //DEBUG! Do only if talon is associated with sensor, and object exists //DEBUG! REPLACE!
		if(sensors[i]->getTalonPort() > 0 && talons[sensors[i]->getTalonPort() - 1]) talons[sensors[i]->getTalonPort() - 1]->restart(); //DEBUG! Do only if talon is associated with sensor, and object exists //DEBUG! REPLACE!
		if(sensors[i]->getSensorPort() > 0 && sensors[i]->getTalonPort() > 0) { //If a Talon is associated with the sensor, turn that port on
			talons[sensors[i]->getTalonPort() - 1]->disableDataAll(); //Turn off all data on Talon
			// talons[sensors[i]->getTalonPort() - 1]->enablePower(sensors[i]->getSensorPort(), true); //Turn on power for the given port on the Talon
			talons[sensors[i]->getTalonPort() - 1]->enableData(sensors[i]->getSensorPort(), true); //Turn back on only port used
			
		}
		if(sensors[i]->getTalonPort() == 0 && sensors[i]->sensorInterface != BusType::CORE) {
			missingSensor = true; //Set flag if any sensors not assigned to Talon and not a core sensor
			Serial.print("Missing Sensor: "); //DEBUG!
			Serial.print(i);
			Serial.print("\t");
			Serial.println(sensors[i]->sensorInterface);
		}
		bool hasCriticalError = false;
		bool hasError = false;

  		String val;
		if(sensors[i]->getTalonPort() > 0 && sensors[i]->getSensorPort() > 0) val = sensors[i]->begin(logger.getTime(), hasCriticalError, hasError); //If detected sensor, run begin
		else if(sensors[i]->getTalonPort() > 0 && sensors[i]->getSensorPort() == 0 || sensors[i]->sensorInterface == BusType::CORE) val = sensors[i]->selfDiagnostic(2, logger.getTime()); //If sensor is a Talon or CORE type, run diagnostic, begin has already been run
		if(hasError) reportError = true; //Set if any of them throw an error
		if(hasCriticalError) reportCriticalError = true; //Set if any of them throw a critical error
		if(!val.equals("")) {  //Only append if not empty string
			if(output.length() - output.lastIndexOf('\n') + val.length() + closer.length() + 1 < Kestrel::MAX_MESSAGE_LENGTH) { //Add +1 to account for comma appending, subtract any previous lines from count
				if(deviceCount > 0) output = output + ","; //Add preceeding comma if not the first entry
				output = output + "{" + val + "}"; //Append result 
				deviceCount++;
				// if(i + 1 < numSensors) diagnostic = diagnostic + ","; //Only append if not last entry
			}
			else {
				output = output + closer + "\n"; //End this packet
				output = output + leader + "{" + val + "}"; //Start a new packet and add new payload 
			}
		}
		
	}
	if(missingSensor) logger.setIndicatorState(IndicatorLight::SENSORS, IndicatorMode::ERROR);
	else logger.setIndicatorState(IndicatorLight::SENSORS, IndicatorMode::PASS); //If no errors are reported, set to pass state
	//FIX! Replace!
	// if(reportCriticalError) logger.setIndicatorState(IndicatorLight::SENSORS, IndicatorMode::ERROR_CRITICAL);
	// else if(reportError) logger.setIndicatorState(IndicatorLight::SENSORS, IndicatorMode::ERROR); //Only set minimal error state if critical error is not thrown
	// else logger.setIndicatorState(IndicatorLight::SENSORS, IndicatorMode::PASS); //If no errors are reported, set to pass state
	
	output = output + closer; //Close diagnostic
	return output;
}

void quickTalonShutdown()
{
	// Wire.beginTransmission(0x22); //Talk to I2C Talon
	// Wire.write(0x48); //Point to pullup/pulldown select reg
	// Wire.write(0xF0); //Set pins 1 - 4 as pulldown
	// Wire.endTransmission();

	// Wire.beginTransmission(0x22); //Talk to I2C Talon
	// Wire.write(0x46); //Point to pullup/pulldown enable reg
	// Wire.write(0x0F); //Enable pulldown on pins 1-4
	// Wire.endTransmission();

	//////////// DEBUG! /////////////
	//// SET SDI-12 TALON First to eliminiate issue with power being applied to Apogee port 
	Wire.beginTransmission(0x25); //Talk to SDI12 Talon
	Wire.write(0x02); //Point to output port
	Wire.write(0x00); //Set pints 1 - 8 low
	Wire.endTransmission();

	Wire.beginTransmission(0x25); //Talk to SDI12 Talon
	Wire.write(0x06); //Point to config port
	Wire.write(0x00); //Set pins 1 - 8 as output
	Wire.endTransmission();

	Wire.beginTransmission(0x25); //Talk to SDI12 Talon
	Wire.write(0x00); //Point to port reg
	// Wire.write(0xF0); //Set pints 1 - 4 low
	Wire.endTransmission();

	Wire.requestFrom(0x25, 1); 
	Wire.read(); //Read back current value

	Wire.beginTransmission(0x22); //Talk to I2C Talon
	Wire.write(0x06); //Point to config port
	Wire.write(0xF0); //Set pins 1 - 4 as output
	Wire.endTransmission();

	Wire.beginTransmission(0x22); //Talk to I2C Talon
	Wire.write(0x02); //Point to output port
	Wire.write(0xF0); //Set pints 1 - 4 low
	Wire.endTransmission();

	Wire.beginTransmission(0x22); //Talk to I2C Talon
	Wire.write(0x00); //Point to port reg
	// Wire.write(0xF0); //Set pints 1 - 4 low
	Wire.endTransmission();

	Wire.requestFrom(0x22, 1); 
	Wire.read(); //Read back current value
	/////////// END DEBUG! /////////////

	// Wire.beginTransmission(0x25); //Talk to SDI-12 Talon
	// Wire.write(0x48); //Point to pullup/pulldown select reg
	// Wire.write(0xF0); //Set pins 1 - 4 as pulldown
	// Wire.endTransmission();

	// Wire.beginTransmission(0x25); //Talk to SDI-12 Talon
	// Wire.write(0x46); //Point to pullup/pulldown enable reg
	// Wire.write(0x0F); //Enable pulldown on pins 1-4
	// Wire.endTransmission();

	
}

bool serialConnected() //Used to check if a monitor has been connected at the begining of operation for override control 
{
	if(Serial.available() > 0) return true;
	else return false;
}

void systemConfig()
{
	Serial.println("HALT: Entered Command Mode - Here be Dragons"); //DEBUG!
	static int ReadLength = 0;
  	String ReadString;
	char ReadArray[25] = {0};
	while(1) {
		if(Serial.available() > 0) {
			char Input = Serial.read();
			if(Input != '\r') { //Wait for return
				ReadArray[ReadLength] = Input;
				ReadLength++;
			}
			if(Input == '\r') {
				ReadString = String(ReadArray);
				ReadString.trim();
				memset(ReadArray, 0, sizeof(ReadArray));
				ReadLength = 0;

				Serial.print(">");
				Serial.println(ReadString); //Echo back to serial monitor

				if(ReadString.equalsIgnoreCase("Erase FRAM")) {
					fileSys.eraseFRAM();
					Serial.println("\tDone");
				}

				if(ReadString.equalsIgnoreCase("Set Accel Zero")) {
					logger.zeroAccel();
					Serial.println("\tDone");
				}

				if(ReadString.equalsIgnoreCase("Clear Accel Zero")) {
					logger.zeroAccel(true);
					Serial.println("\tDone");
				}

				if(ReadString.equalsIgnoreCase("Exit")) {
					return; //Exit the setup function
				}
			}
		}
	}
}

int sleepSensors()
{
	if(powerSaveMode > PowerSaveModes::PERFORMANCE) { //Only turn off is power save requested 
		Serial.println("BEGIN SENSOR SLEEP"); //DEBUG!
		for(int s = 0; s < numSensors; s++) { //Iterate over all sensors objects
			//If not set to keep power on and Talon is assocated, power down sensor. Ignore if core device, we will handle these seperately 
			if(sensors[s]->keepPowered == false && sensors[s]->sensorInterface != BusType::CORE && sensors[s]->getTalonPort() > 0 && sensors[s]->getTalonPort() < numTalons) {
				Serial.print("Power Down Sensor "); //DEBUG!
				Serial.print(s + 1);
				Serial.print(",");
				Serial.println(sensors[s]->getTalonPort());
				talons[sensors[s]->getTalonPort() - 1]->enablePower(sensors[s]->getSensorPort(), false); //Turn off power for any sensor which does not need to be kept powered
			}
			else if(sensors[s]->sensorInterface != BusType::CORE && sensors[s]->getTalonPort() > 0 && sensors[s]->getTalonPort() < numTalons){ //If sensor has a position and is not core, but keepPowered is true, run sleep routine
				Serial.print("Sleep Sensor "); //DEBUG!
				Serial.println(s + 1);
				sensors[s]->sleep(); //If not powered down, run sleep protocol 
			}
			else if(sensors[s]->sensorInterface == BusType::CORE) {
				Serial.print("Sensor "); //DEBUG!
				Serial.print(s + 1);
				Serial.println(" is core, do nothing"); 
			}
			else {
				Serial.print("Sensor "); //DEBUG!
				Serial.print(s + 1);
				Serial.println(" not detected, do nothing"); 
			}
		}

		for(int t = 0; t < Kestrel::numTalonPorts; t++) { //Iterate over all talon objects
			if(talons[t] && talons[t]->keepPowered == false) { //If NO sensors on a given Talon require it to be kept powered, shut the whole thing down
				Serial.print("Power Down Talon "); //DEBUG!
				Serial.println(talons[t]->getTalonPort());
				logger.enablePower(talons[t]->getTalonPort(), false); //Turn off power to given port 
			}
			else if(!talons[t]) {
				Serial.print("Power Down Empty Port "); //DEBUG!
				Serial.println(t + 1);
				logger.enablePower(t + 1, false); //Turn off power to unused port
			}
		}
	}

	return 0; //DEBUG!
}

int wakeSensors()
{
	logger.enableI2C_Global(true); //Connect to external bus to talk to sensors/Talons
	logger.enableI2C_OB(false);
	logger.disableDataAll(); //Turn off all data to start
	for(int p = 1; p <= Kestrel::numTalonPorts; p++) logger.enablePower(p, true); //Turn power back on to all Kestrel ports
	for(int t = 0; t < Kestrel::numTalonPorts; t++) {
		if(talons[t] && talons[t]->getTalonPort() != 0) {
			logger.enableData(talons[t]->getTalonPort(), true); //Turn on data for given port
			talons[t]->restart(); //Restart all Talons, this turns on all ports it can
			logger.enableData(talons[t]->getTalonPort(), false); //Turn data back off for given port
		}
	}
	for(int s = 0; s < numSensors; s++) {
		if(sensors[s]->getTalonPort() != 0) {
			logger.enableData(sensors[s]->getTalonPort(), true); //Turn on data for given port
			sensors[s]->wake(realTimeProvider); //Wake each sensor
			logger.enableData(sensors[s]->getTalonPort(), false); //Turn data back off for given port
		}
	}
	return 0; //DEBUG!
}

int detectTalons(String dummyStr)
{
		////////////// AUTO TALON DETECTION ///////////////////////
	// talons[0] = &aux; //Place talon objects at arbitrary positions in array
	// talons[1] = &aux1;
	// talons[2] = &i2c;
	
	// bool hasCriticalError = false;
	// bool hasError = false;
	// for(int i = 0; i < numTalons; i++) { //Initialize all Talons //DEBUG!
	// 	talons[i]->begin(Time.now(), hasCriticalError, hasError);
	// }
	// logger.enableI2C_External(false); //Turn off connection to 
	logger.enableI2C_Global(true); //Connect to external bus to talk to sensors/Talons
	logger.enableI2C_OB(false);
	for(int port = 1; port <= Kestrel::numTalonPorts; port++) { //Test all ports
		logger.enableData(port, true); //Turn on specific channel
		logger.enablePower(port, false); 
		logger.enablePower(port, true); 
		// logger.enableAuxPower(true);
		// logger.enableI2C_Global(true);
		// logger.enableI2C_OB(false);
		// delay(1);//DEBUG!
		unsigned long localTime = millis();
		int error = 0;
		while((millis() - localTime) < 10) { //Wait up to 10ms for connection to be established 
			Wire.beginTransmission(0);
			error = Wire.endTransmission();
			if(error == 0) break; //Exit loop once we are able to connect with Talon 
		}
		quickTalonShutdown(); //Quickly disables power to all ports on I2C or SDI talons, this is a kluge 
		for(int t = 0; t < numTalons; t++) { //Iterate over all Talon objects
			if(talonsToTest[t]->getTalonPort() == 0) { //If port not already specified 
				Serial.print("New Talon: ");
				Serial.println(t); 
				// logger.enableAuxPower(false); //Turn aux power off, then configure port to on, then switch aux power back for faster response
				// logger.enablePower(port, true); //Toggle power just before testing to get result within 10ms
				// logger.enablePower(port, false);
				if(talonsToTest[t]->isPresent()) { //Test if that Talon is present, if it is, configure the port
					talonsToTest[t]->setTalonPort(port);
					talons[port - 1] = talonsToTest[t]; //Copy test talon object to index location in talons array
					Serial.print("Talon Port Result "); //DEBUG!
					Serial.print(t);
					Serial.print(": ");
					Serial.println(talonsToTest[t]->getTalonPort());
					break; //Exit the interation after the first one tests positive 
				}
			}
		}
		logger.enableData(port, false); //Turn port back off
	}
	// talons[aux.getTalonPort() - 1] = &aux; //Place talon objects at detected positions in array
	// talons[aux1.getTalonPort() - 1] = &aux1; 
	// talons[i2c.getTalonPort() - 1] = &i2c;
	bool dummy;
	bool dummy1;
	for(int i = 0; i < Kestrel::numTalonPorts - 1; i++) {
		if(talons[i] && talons[i]->getTalonPort() > 0) {
			Serial.print("BEGIN TALON: "); //DEBUG!
			Serial.print(talons[i]->getTalonPort()); 
			Serial.print(",");
			Serial.println(i);
			if(talons[i]->talonInterface == BusType::SDI12) {
				Serial.println("SET FOR SDI12 SEL"); //DEBUG!
				logger.setDirection(talons[i]->getTalonPort(), HIGH); //If the talon is an SDI12 interface type, set port to use serial interface
			}
			else if(talons[i]->talonInterface != BusType::CORE) logger.setDirection(talons[i]->getTalonPort(), LOW); //Otherwise set talon to use GPIO interface, unless bus type is core, in which case ignore it
			logger.enablePower(i + 1, true); //Turn on specific channel
			logger.enableData(i + 1, true);
			if(logger.getFault(talons[i]->getTalonPort())) { //Only toggle power if there is a fault on that Talon line
				logger.enablePower(i + 1, true); //Toggle power just before testing to get result within 10ms
				logger.enablePower(i + 1, false); 
				logger.enablePower(i + 1, true);
			} 
			
			logger.configTalonSense(); //Setup to allow for current testing 
			// Serial.println("TALON SENSE CONFIG DONE"); //DEBUG!
			// Serial.flush(); //DEBUG!
			// logger.enableI2C_Global(true);
			// logger.enableI2C_OB(false);
			// talons[i]->begin(Time.now(), dummy, dummy1); //If Talon object exists and port has been assigned, initialize it //DEBUG!
			talons[i]->begin(logger.getTime(), dummy, dummy1); //If Talon object exists and port has been assigned, initialize it //REPLACE getTime! 
			// talons[i]->begin(0, dummy, dummy1); //If Talon object exists and port has been assigned, initialize it //REPLACE getTime! 
			// Serial.println("TALON BEGIN DONE"); //DEBUG!
			// Serial.flush(); //DEBUG!
			// delay(10000); //DEBUG!
			logger.enableData(i + 1, false); //Turn data back off to prevent conflict 
			// Serial.println("ENABLE DATA DONE"); //DEBUG!
			// Serial.flush(); //DEBUG!
			// delay(10000); //DEBUG!
		}
	}
	return 0; //DEBUG!
}

int detectSensors(String dummyStr)
{
	/////////////// SENSOR AUTO DETECTION //////////////////////
	for(int t = 0; t < Kestrel::numTalonPorts; t++) { //Iterate over each Talon
	// Serial.println(talons[t]->talonInterface); //DEBUG!
	// Serial.print("DETECT ON TALON: "); //DEBUG!
	// Serial.println(t);
	// Serial.flush();
	// if(talons[t]) {
	// 	delay(5000);
	// 	Serial.println("TALON EXISTS"); //DEBUG!
	// 	Serial.flush();
	// }
	// else {
	// 	delay(5000);
	// 	Serial.println("TALON NOT EXISTS"); //DEBUG!
	// 	Serial.flush();
	// }
	// delay(5000);
	// if(talons[t]->talonInterface != BusType::NONE) {
	// 	delay(5000);
	// 	Serial.println("TALON NOT NONE"); //DEBUG!
	// 	Serial.flush();
	// }
	// else {
	// 	delay(5000);
	// 	Serial.println("TALON NONE"); //DEBUG!
	// 	Serial.flush();
	// }
	// delay(10000); //DEBUG!
		// Serial.println(talons[t]->talonInterface); //DEBUG!
		if(talons[t] && talons[t]->talonInterface != BusType::NONE && talons[t]->getTalonPort() != 0) { //Only proceed if Talon has a bus which can be iterated over, and the talon in question exists and has been detected 
			logger.enableData(talons[t]->getTalonPort(), true); //Turn on specific channel
			// logger.enableI2C_Global(true);
			// logger.enableI2C_OB(false);
			talons[t]->disableDataAll(); //Turn off all data ports on Talon
			for(int p = 1; p <= talons[t]->getNumPorts(); p++) { //Iterate over each port on given Talon
				// talons[t]->enablePower(p, true); //Turn data and power on for specific channel
				talons[t]->enableData(p, true);
				delay(10); //Wait to make sure sensor is responsive after power up command 
				Serial.print("Testing Port: "); //DEBUG!
				Serial.print(t + 1);
				Serial.print(",");
				Serial.println(p);
				for(int s = 0; s < numSensors; s++) { //Iterate over all sensors objects
					if(sensors[s]->getTalonPort() == 0 && talons[t]->talonInterface == sensors[s]->sensorInterface) { //If Talon not already specified AND sensor bus is compatible with Talon bus
						Serial.print("Test Sensor: "); //DEBUG!
						Serial.println(s);
						if(sensors[s]->isPresent()) { //Test if that sensor is present, if it is, configure the port
							sensors[s]->setTalonPort(t + 1);
							sensors[s]->setSensorPort(p);
							if(sensors[s]->keepPowered == true) talons[sensors[s]->getTalonPort() - 1]->keepPowered = true; //If any of the sensors on a Talon require power, set the flag for the Talon
							Serial.print("Sensor Found:\n\t"); //DEBUG!
							Serial.println(sensors[s]->getTalonPort());
							Serial.print('\t');
							Serial.println(sensors[s]->getSensorPort());
							// Serial.print("Talon Port Result "); //DEBUG!
							// Serial.print(t);
							// Serial.print(": ");
							// Serial.println(talons[t]->getTalonPort());
							// talons[t]->enableData(p, false);
							break; //Exit the interation after the first sensor tests positive 
						}
						delay(10); //Wait in between sensor calls
						// talons[t]->enableData(p, false);
					}
				}
				talons[t]->enableData(p, false); //Turn data back off when done
			}
			logger.enableData(talons[t]->getTalonPort(), false); //Turn port back off
		}
		// Serial.print("NEXT TALON"); //DEBUG!
		// Serial.flush();
	}
	return 0; //DEBUG!
}

int setNodeID(String nodeID)
{
	if(nodeID.length() > 8 || nodeID.length() < 0) return -1; //Return failure if string is not valid 
	else {
		globalNodeID = nodeID; //If string passed in is valid, copy it to the global value
		return 0;
	}
}

int takeSample(String dummy)
{
	logger.wake(); //Wake logger in case it was sleeping
	wakeSensors(); //Wake up sensors from sleep
	if(dummy == "true") { //If told to use backhaul, use normal FRAM method
		fileSys.writeToFRAM(getDataString(), DataType::Data, DestCodes::Both); 
		fileSys.dumpFRAM(); //Dump data
	}
	else fileSys.writeToParticle(getDataString(), "data/v2"); //Otherwise fast return
	sleepSensors(); //
	logger.sleep();
	return 1;
}

int commandExe(String command)
{
	if(command == "300") {
		logger.releaseWDT();
		return 1; //DEBUG!
	}
	if(command == "102") {
		logger.wake(); //Wake logger in case it was sleeping
		wakeSensors(); //Wake up sensors from sleep
		fileSys.writeToParticle(getDiagnosticString(2), "diagnostic/v2"); 
		sleepSensors(); //
		logger.sleep();
		return 1; //DEBUG!
	}
	if(command == "103") {
		logger.wake(); //Wake logger in case it was sleeping
		wakeSensors(); //Wake up sensors from sleep
		fileSys.writeToParticle(getDiagnosticString(3), "diagnostic/v2"); 
		sleepSensors(); //
		logger.sleep();
		return 1; //DEBUG!
	}
	if(command == "104") {
		logger.wake(); //Wake logger in case it was sleeping
		wakeSensors(); //Wake up sensors from sleep
		fileSys.writeToParticle(getDiagnosticString(4), "diagnostic/v2"); 
		sleepSensors(); //
		logger.sleep();
		return 1; //DEBUG!
	}
	if(command == "111") {
		logger.wake(); //Wake logger in case it was sleeping
		wakeSensors(); //Wake up sensors from sleep
		fileSys.writeToParticle(getDataString(), "data/v2"); 
		sleepSensors(); //
		logger.sleep();
		return 1; //DEBUG!
	}
	if(command == "120") {
		fileSys.writeToParticle(getErrorString(), "error/v2"); 
		return 1; //DEBUG!
	}
	if(command == "130") {
		logger.wake(); //Wake logger in case it was sleeping
		wakeSensors(); //Wake up sensors from sleep
		fileSys.writeToParticle(getMetadataString(), "metadata/v2"); 
		sleepSensors(); //
		logger.sleep();
		return 1; //DEBUG!
	}
	if(command == "401") {
		fileSys.wake();
		fileSys.dumpFRAM();
		fileSys.sleep();
		return 1;
	}
	if(command == "410") {
		fileSys.wake();
		fileSys.eraseFRAM(); //Clear FRAM and start over
		fileSys.sleep();
		return 1; //DEBUG!
	}
	else {
		return -1; //Return unknown command 
	}
}

int systemRestart(String resetType)
{
	if(resetType.equalsIgnoreCase("hard")) System.reset(RESET_NO_WAIT); //Perform a hard reset
	else System.reset(); //Attempt to inform cloud of a reset first 
	return 1;
}

int configurePowerSave(int desiredPowerSaveMode)
{
	powerSaveMode = desiredPowerSaveMode; //Configure global flag
	for(int s = 0; s < numSensors; s++) { //Iterate over all sensors objects
		sensors[s]->powerSaveMode = desiredPowerSaveMode; //Set power save mode for all sensors
	}

	for(int t = 0; t < numTalons; t++)  { //Iterate over all talon objects
		talonsToTest[t]->powerSaveMode = desiredPowerSaveMode; //Set power save mode for all talons
	}
	return 0; //DEBUG!
}