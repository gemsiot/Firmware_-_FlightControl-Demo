// tests/mocks/MockTimeProvider.h
#ifndef MOCK_TIME_PROVIDER_H
#define MOCK_TIME_PROVIDER_H

#include "gmock/gmock.h"
#include "ITimeProvider.h" // Include the interface definition

/**
 * @brief Google Mock implementation of ITimeProvider for testing.
 */
class MockTimeProvider : public ITimeProvider {
public:
    // Mock all methods defined in the interface
    MOCK_METHOD(int, year, (), (override));
    MOCK_METHOD(int, year, (time_t t), (override));
    MOCK_METHOD(int, month, (), (override));
    MOCK_METHOD(int, month, (time_t t), (override));
    MOCK_METHOD(int, day, (), (override));
    MOCK_METHOD(int, day, (time_t t), (override));
    MOCK_METHOD(int, hour, (), (override));
    MOCK_METHOD(int, hour, (time_t t), (override));
    MOCK_METHOD(int, minute, (), (override));
    MOCK_METHOD(int, minute, (time_t t), (override));
    MOCK_METHOD(int, second, (), (override));
    MOCK_METHOD(int, second, (time_t t), (override));
    MOCK_METHOD(time_t, now, (), (override));
    MOCK_METHOD(void, setTime, (time_t t), (override));
    MOCK_METHOD(bool, isValid, (), (override));
    MOCK_METHOD(void, zone, (float GMT_Offset), (override));
    MOCK_METHOD(uint32_t, millis, (), (override));
    MOCK_METHOD(void, delay, (uint32_t ms), (override));
};

#endif // MOCK_TIME_PROVIDER_H