// src/platform/ParticleTimeProvider.h
#ifndef PARTICLE_TIME_PROVIDER_H
#define PARTICLE_TIME_PROVIDER_H

#include "ITimeProvider.h" // Include the interface definition
#include "Particle.h" // Include the actual Particle header HERE

/**
 * @brief Concrete implementation of ITimeProvider using Particle API.
 */
class ParticleTimeProvider : public ITimeProvider {
public:
    // Constructor/Destructor (often default is fine)
    ParticleTimeProvider() = default;
    ~ParticleTimeProvider() override = default;

    // Implement methods from ITimeProvider
    int year() override;
    int year(time_t t) override;
    int month() override;
    int month(time_t t) override;
    int day() override;
    int day(time_t t) override;
    int hour() override;
    int hour(time_t t) override;
    int minute() override;
    int minute(time_t t) override;
    int second() override;
    int second(time_t t) override;
    time_t now() override;
    void setTime(time_t t) override;
    bool isValid() override;
    void zone(float GMT_Offset) override;
    uint32_t millis() override;
    void delay(uint32_t ms) override;
};

#endif // PARTICLE_TIME_PROVIDER_H