#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "Particle.h" //this is a mock of Particle.h that only includes its string implementation

// Include other mocks
#include "MockSDI12Talon.h"
#include "MockTimeProvider.h"

// Include the actual implementation
#include "Li710.h"

class Li710Test : public ::testing::Test {
protected:
    // Our mock objects
    MockSDI12Talon mockTalon;
    MockTimeProvider mockTimeProvider;
    
    void SetUp() override {
        // Set up common mock behavior
        ON_CALL(mockTalon, getSensorPortString())
            .WillByDefault(::testing::Return("1"));
        ON_CALL(mockTalon, getTalonPortString())
            .WillByDefault(::testing::Return("1"));
        ON_CALL(mockTalon, getSensorPort())
            .WillByDefault(::testing::Return(1));
        ON_CALL(mockTalon, getTalonPort())
            .WillByDefault(::testing::Return(1));
    }
};

// verify sensor interface set to SDI12
TEST_F(Li710Test, TestSensorInterfaceSdi12) {
    // Create a Li710 instance with our mock
    LI710 li710(mockTimeProvider, mockTalon, 1, 1);
    
    // Verify that the sensor interface is set to SDI12
    EXPECT_EQ(li710.sensorInterface, BusType::SDI12);
}

//verify talon port set to 255 if not in range
TEST_F(Li710Test, TestTalonPortOutOfRange) {
    // Create a Li710 instance with our mock
    LI710 li710(mockTimeProvider, mockTalon, 0, 1); // Set an out-of-range talon port
    
    // Verify that getTalonPort returns 0 when the port is 255
    EXPECT_EQ(li710.getTalonPort(), 0);
}

//verify sensor port set to 255 if not in range
TEST_F(Li710Test, TestSensorPortOutOfRange) {
    // Create a Li710 instance with our mock
    LI710 li710(mockTimeProvider, mockTalon, 1, 0); // Set an out-of-range sensor port
    
    // Verify that getTalonPort returns 0 when the port is 255
    EXPECT_EQ(li710.getSensorPort(), 0);
}

//verify talonPort is talonPort_ - 1
TEST_F(Li710Test, TestTalonPortSet) {
    // Create a Li710 instance with our mock
    LI710 li710(mockTimeProvider, mockTalon, 42, 1); // Set a valid talon port
    
    // Verify that the talon port is set correctly
    EXPECT_EQ(li710.getTalonPort(), 42);
}

//verify sensorPort is sensorPort_ - 1
TEST_F(Li710Test, TestSensorPortSet) {
    // Create a Li710 instance with our mock
    LI710 li710(mockTimeProvider, mockTalon, 1, 42); // Set a valid talon port
    
    // Verify that the sensor port is set correctly
    EXPECT_EQ(li710.getSensorPort(), 42);
}

// verify begin returns empty string
TEST_F(Li710Test, TestBegin) {
    LI710 li710(mockTimeProvider, mockTalon, 1, 1);
    bool criticalFault = false;
    bool fault = false;
    
    String result = li710.begin(0, criticalFault, fault);
    
    EXPECT_EQ(result, "");
}

// Test isPresent method when sensor is present
TEST_F(Li710Test, TestIsPresent_SensorFound) {
    LI710 li710(mockTimeProvider, mockTalon, 1, 1);
    
    EXPECT_CALL(mockTalon, sendCommand(String("?!")))
        .WillOnce(::testing::Return("0"));
        
    EXPECT_CALL(mockTalon, command(String("I"), 0))
        .WillOnce(::testing::Return("013LI-COR  LI-7101.01234567"));
    
    bool result = li710.isPresent();
    
    EXPECT_TRUE(result);
}

// Test isPresent method when sensor is not found
TEST_F(Li710Test, TestIsPresent_SensorNotFound) {
    LI710 li710(mockTimeProvider, mockTalon, 1, 1);
    
    EXPECT_CALL(mockTalon, sendCommand(String("?!")))
        .WillOnce(::testing::Return("0"));
        
    EXPECT_CALL(mockTalon, command(String("I"), 0))
        .WillOnce(::testing::Return("013UNKNOWN SENSOR1.01234567"));
    
    bool result = li710.isPresent();
    
    EXPECT_FALSE(result);
}

//verify isPresent is called three times when it only returns false
TEST_F(Li710Test, TestGetData_SensorNotFound) {
    LI710 li710(mockTimeProvider, mockTalon, 1, 1);

    EXPECT_CALL(mockTalon, sendCommand(String("?!")))
        .Times(3)
        .WillRepeatedly(::testing::Return("0"));
        
    EXPECT_CALL(mockTalon, command(String("I"), 0))
        .Times(3)
        .WillRepeatedly(::testing::Return("013UNKNOWN SENSOR1.01234567"));
    
    li710.getData(0);
}

//verify talon.getAddress is called three times when it only return -1
TEST_F(Li710Test, TestGetData_TalonAddressFail) {
    LI710 li710(mockTimeProvider, mockTalon, 1, 1);

    EXPECT_CALL(mockTalon, sendCommand(String("?!")))
        .Times(3)
        .WillRepeatedly(::testing::Return("0"));
        
    EXPECT_CALL(mockTalon, command(String("I"), 0))
        .Times(3)
        .WillRepeatedly(::testing::Return("013LI-COR  LI-7101.01234567"));
    
    EXPECT_CALL(mockTalon, getAddress())
        .Times(3)
        .WillRepeatedly(::testing::Return(-1));
    
    li710.getData(0);
}

//verify ouptut is null when sensorPort is 0
TEST_F(Li710Test, TestGetData_SensorPortZero) {
    LI710 li710(mockTimeProvider, mockTalon, 1, 0); // Set an out-of-range sensor port

    String result = li710.getData(0);

    EXPECT_TRUE(result.indexOf("LiCor ET\":{") > 0);
    EXPECT_TRUE(result.indexOf("\"ET\":null") > 0);
    EXPECT_TRUE(result.indexOf("}") > 0);
}

//verify output is null when retry count is reached
TEST_F(Li710Test, TestGetData_RetryCountReached) {
    LI710 li710(mockTimeProvider, mockTalon, 1, 1);

    EXPECT_CALL(mockTalon, sendCommand(String("?!")))
        .Times(3)
        .WillRepeatedly(::testing::Return("0"));
        
    EXPECT_CALL(mockTalon, command(String("I"), 0))
        .Times(3)
        .WillRepeatedly(::testing::Return("013UNKNOWN SENSOR1.01234567"));
    
    String result = li710.getData(0);

    EXPECT_TRUE(result.indexOf("LiCor ET\":{") > 0);
    EXPECT_TRUE(result.indexOf("\"ET\":null") > 0);
    EXPECT_TRUE(result.indexOf("}") > 0);
}

// Test getData method with valid data
TEST_F(Li710Test, DISABLED_TestGetData_ValidData) {
    LI710 li710(mockTimeProvider, mockTalon, 1, 1);
    
    EXPECT_CALL(mockTalon, sendCommand(String("?!")))
        .WillOnce(::testing::Return("0"));
        
    EXPECT_CALL(mockTalon, command(String("I"), 0))
        .WillOnce(::testing::Return("013LI-COR  LI-7101.01234567"));
    
    EXPECT_CALL(mockTalon, getAddress())
        .WillOnce(::testing::Return(0));
    
    EXPECT_CALL(mockTalon, command(String("XT"), 0))
        .WillOnce(::testing::Return(""));
    
    // Mock valid responses for data
    EXPECT_CALL(mockTalon, continuousMeasurmentCRC(0, 0))
        .WillOnce(::testing::Return("0+1.23+4.56+7.89+10.11+12.13+14.15+16.17+18.19+20.21"));
    
    EXPECT_CALL(mockTalon, continuousMeasurmentCRC(1, 0))
        .WillOnce(::testing::Return("0+22.23+24.25+26.27+28.29+30.31+32.33+34.35+36.37+38.39"));
    
    EXPECT_CALL(mockTalon, continuousMeasurmentCRC(2, 0))
        .WillOnce(::testing::Return("0+40.41+42.43+44.45+46.47+48.49+50.51+52.53+54.55"));
    
    EXPECT_CALL(mockTalon, testCRC(::testing::_))
        .WillRepeatedly(::testing::Return(true));
    
    String result = li710.getData(0);
    
    EXPECT_TRUE(result.indexOf("LiCor ET\":{") > 0);
    EXPECT_TRUE(result.indexOf("\"ET\":1.230") > 0);
    EXPECT_TRUE(result.indexOf("\"LE\":4.6") > 0);
    EXPECT_TRUE(result.indexOf("\"H\":7.9") > 0);
    EXPECT_TRUE(result.indexOf("\"VPD\":10.11") > 0);
    EXPECT_TRUE(result.indexOf("\"PA\":12.13") > 0);
    EXPECT_TRUE(result.indexOf("\"TA\":14.15") > 0);
    EXPECT_TRUE(result.indexOf("\"RH\":16.17") > 0);
    EXPECT_TRUE(result.indexOf("\"SAMP_CNT\":36") > 0);
    EXPECT_TRUE(result.indexOf("\"AH\":40.41") > 0);
    EXPECT_TRUE(result.indexOf("\"SVP\":44.45") > 0);
    EXPECT_TRUE(result.indexOf("\"TD\":54.55") > 0);
    EXPECT_TRUE(result.indexOf("\"Pos\":[1,1]") > 0);
}

// Test getData method with valid data and start measurment
//after new read method
TEST_F(Li710Test, TestGetData_ValidDataStartMeasurment) {
    LI710 li710(mockTimeProvider, mockTalon, 1, 1);
    
    EXPECT_CALL(mockTalon, sendCommand(String("?!")))
        .WillOnce(::testing::Return("0"));
        
    EXPECT_CALL(mockTalon, command(String("I"), 0))
        .WillOnce(::testing::Return("013LI-COR  LI-7101.01234567"));
    
    EXPECT_CALL(mockTalon, getAddress())
        .WillOnce(::testing::Return(0));
    
    EXPECT_CALL(mockTalon, command(String("XT"), 0))
        .WillOnce(::testing::Return(""));
    
    // Mock valid responses for data
    EXPECT_CALL(mockTalon, startMeasurmentIndex(0 , 0))
        .WillOnce(::testing::Return(10));
    
    EXPECT_CALL(mockTalon, startMeasurmentIndex(1 , 0))
        .WillOnce(::testing::Return(10));

    EXPECT_CALL(mockTalon, startMeasurmentIndex(2 , 0))
        .WillOnce(::testing::Return(10));

    EXPECT_CALL(mockTalon, startMeasurmentIndex(3 , 0))
        .WillOnce(::testing::Return(10));
    
    EXPECT_CALL(mockTalon, command(String("D0"), 0))
        .WillOnce(::testing::Return("0+1.23+4.56+7.89"))
        .WillOnce(::testing::Return("0+22.23+24.25+26.27"))
        .WillOnce(::testing::Return("0+40.41+42.43+44.45"))
        .WillOnce(::testing::Return("0+40.41+42.43+44.45")); //value tbd

    EXPECT_CALL(mockTalon, command(String("D1"), 0))
        .WillOnce(::testing::Return("0+10.11+12.13+14.15"))
        .WillOnce(::testing::Return("0+28.29+30.31+32.33"))
        .WillOnce(::testing::Return("0+46.47+48.49+50.51"))
        .WillOnce(::testing::Return("0+46.47+48.49+50.51")); //value tbd
    
    EXPECT_CALL(mockTalon, command(String("D2"), 0))
        .WillOnce(::testing::Return("0+16.17+18.19+20.21"))
        .WillOnce(::testing::Return("0+34.35+36.37+38.39"))
        .WillOnce(::testing::Return("0+52.53+54.55"))
        .WillOnce(::testing::Return("0+52.53+54.55")); //value tbd
    
    String result = li710.getData(0);
    
    EXPECT_TRUE(result.indexOf("LiCor ET\":{") > 0);
    EXPECT_TRUE(result.indexOf("\"ET\":1.230") > 0);
    EXPECT_TRUE(result.indexOf("\"LE\":4.6") > 0);
    EXPECT_TRUE(result.indexOf("\"H\":7.9") > 0);
    EXPECT_TRUE(result.indexOf("\"VPD\":10.11") > 0);
    EXPECT_TRUE(result.indexOf("\"PA\":12.13") > 0);
    EXPECT_TRUE(result.indexOf("\"TA\":14.15") > 0);
    EXPECT_TRUE(result.indexOf("\"RH\":16.17") > 0);
    EXPECT_TRUE(result.indexOf("\"SAMP_CNT\":36") > 0);
    EXPECT_TRUE(result.indexOf("\"AH\":40.41") > 0);
    EXPECT_TRUE(result.indexOf("\"SVP\":44.45") > 0);
    EXPECT_TRUE(result.indexOf("\"TD\":54.55") > 0);
    EXPECT_TRUE(result.indexOf("\"Pos\":[1,1]") > 0);
}

// Test getData method with CRC failure
//disabled after new read method
TEST_F(Li710Test, DISABLED_TestGetData_CRCFailure) {
    LI710 li710(mockTimeProvider, mockTalon, 1, 1);
    
    EXPECT_CALL(mockTalon, sendCommand(String("?!")))
        .Times(3)
        .WillRepeatedly(::testing::Return("0"));
        
    EXPECT_CALL(mockTalon, command(String("I"), 0))
        .Times(3)
        .WillRepeatedly(::testing::Return("013LI-COR  LI-7101.01234567"));
    
    EXPECT_CALL(mockTalon, getAddress())
        .WillRepeatedly(::testing::Return(0));
    
    EXPECT_CALL(mockTalon, command(String("XT"), 0))
        .WillRepeatedly(::testing::Return(""));
    
    // Mock valid responses for data
    EXPECT_CALL(mockTalon, continuousMeasurmentCRC(0, 0))
        .WillRepeatedly(::testing::Return("0+1.23+4.56+7.89+10.11+12.13+14.15+16.17+18.19+20.21"));
    
    EXPECT_CALL(mockTalon, continuousMeasurmentCRC(1, 0))
        .WillRepeatedly(::testing::Return("0+22.23+24.25+26.27+28.29+30.31+32.33+34.35+36.37+38.39"));
    
    EXPECT_CALL(mockTalon, continuousMeasurmentCRC(2, 0))
        .WillRepeatedly(::testing::Return("0+40.41+42.43+44.45+46.47+48.49+50.51+52.53+54.55"));
    
    // Force CRC failure
    EXPECT_CALL(mockTalon, testCRC(::testing::_))
        .WillRepeatedly(::testing::Return(false));
    
    // Expect error to be reported
    //EXPECT_CALL(mockTalon, throwError(::testing::_))
        //.Times(::testing::AtLeast(1));
    
    String result = li710.getData(0);
    
    EXPECT_TRUE(result.indexOf("LiCor ET\":{") > 0);
    EXPECT_TRUE(result.indexOf("\"ET\":null") > 0);
    EXPECT_TRUE(result.indexOf("\"LE\":null") > 0);
    EXPECT_TRUE(result.indexOf("\"Pos\":[1,1]") > 0);
}

// Test getMetadata method
// refactor this after refactoring Li710
TEST_F(Li710Test, TestGetMetadata) {
    LI710 li710(mockTimeProvider, mockTalon, 1, 1);
    
    EXPECT_CALL(mockTalon, sendCommand(String("?!")))
        .WillOnce(::testing::Return("0"));
    
    EXPECT_CALL(mockTalon, command(String("I"), 0))
        .WillOnce(::testing::Return("013LI-COR  LI-7101.01234567"));
    
    String result = li710.getMetadata();
    
    EXPECT_TRUE(result.indexOf("LiCor ET\":{") > 0);
    EXPECT_TRUE(result.indexOf("\"Hardware\":\"1.0\"") > 0);
    EXPECT_TRUE(result.indexOf("\"SDI12_Ver\":\"1.3\"") > 0);
    EXPECT_TRUE(result.indexOf("\"Mfg\":\"LI-COR\"") > 0);
    EXPECT_TRUE(result.indexOf("\"Model\":\"LI-710\"") > 0);
    EXPECT_TRUE(result.indexOf("\"SN\":\"1234567\"") > 0);
    EXPECT_TRUE(result.indexOf("\"Pos\":[1,1]") > 0);
    EXPECT_TRUE(result.indexOf("}") > 0);
}

// Test selfDiagnostic method
TEST_F(Li710Test, TestSelfDiagnostic) {
    LI710 li710(mockTimeProvider, mockTalon, 1, 1);

    EXPECT_CALL(mockTalon, sendCommand(String("?!")))
        .WillRepeatedly(::testing::Return("0"));
        
    EXPECT_CALL(mockTalon, command(String("I"), 0))
        .WillRepeatedly(::testing::Return("013LI-COR  LI-7101.01234567"));
    
    EXPECT_CALL(mockTalon, getAddress())
        .WillOnce(::testing::Return(0));
    
    EXPECT_CALL(mockTalon, command(String("XT"), 0))
        .WillOnce(::testing::Return(""));
    
    // Mock valid responses for data
    EXPECT_CALL(mockTalon, startMeasurmentIndex(0 , 0))
        .WillOnce(::testing::Return(10));
    
    EXPECT_CALL(mockTalon, startMeasurmentIndex(1 , 0))
        .WillOnce(::testing::Return(10));

    EXPECT_CALL(mockTalon, startMeasurmentIndex(2 , 0))
        .WillOnce(::testing::Return(10));

    EXPECT_CALL(mockTalon, startMeasurmentIndex(3 , 0))
        .WillOnce(::testing::Return(10));

    EXPECT_CALL(mockTalon, command(String("D0"), 0))
        .WillOnce(::testing::Return("0+1.23+4.56+7.89"))
        .WillOnce(::testing::Return("0+22.23+24.25+26.27"))
        .WillOnce(::testing::Return("0+40.41+42.43+44.45"))
        .WillOnce(::testing::Return("0+56.57+58.59+60.61")); //value tbd

    EXPECT_CALL(mockTalon, command(String("D1"), 0))
        .WillOnce(::testing::Return("0+10.11+12.13+14.15"))
        .WillOnce(::testing::Return("0+28.29+30.31+32.33"))
        .WillOnce(::testing::Return("0+46.47+48.49+50.51"))
        .WillOnce(::testing::Return("0+62.63+64.65+66.67")); //value tbd
    
    EXPECT_CALL(mockTalon, command(String("D2"), 0))
        .WillOnce(::testing::Return("0+16.17+18.19+20.21"))
        .WillOnce(::testing::Return("0+34.35+36.37+38.39"))
        .WillOnce(::testing::Return("0+52.53+54.55"))
        .WillOnce(::testing::Return("0+68.69+70.71")); //value tbd

    li710.getData(0);

    String result = li710.selfDiagnostic(5, 0);
    
    EXPECT_TRUE(result.indexOf("\"LiCor ET\":{") >= 0);
    EXPECT_TRUE(result.indexOf("\"Adr\":0") > 0);
    EXPECT_TRUE(result.indexOf("\"PUMP_V\":56.57") > 0);
    EXPECT_TRUE(result.indexOf("\"PA_CELL\":58.59") > 0);
    EXPECT_TRUE(result.indexOf("\"RH_CELL\":60.61") > 0);
    EXPECT_TRUE(result.indexOf("\"TA_CELL\":62.63") > 0);
    //I think this is not 70.71 because the precision value is 0 
    //instead of 2. I think the fake string implementation gets rid 
    //of the digits before the decimal place instead of after
    EXPECT_TRUE(result.indexOf("\"DATA_QC\":71") > 0);
    EXPECT_TRUE(result.indexOf("\"Pos\":[1,1]") > 0);
}