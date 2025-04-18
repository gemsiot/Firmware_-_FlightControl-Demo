// src/platform/ParticleTimeProvider.cpp
#include "ParticleTimeProvider.h"

// Use :: prefix for global scope functions like millis() and delay()
// to avoid potential name conflicts if methods had same name.

int ParticleTimeProvider::year() { return Time.year(); }
int ParticleTimeProvider::year(time_t t) { return Time.year(t); }
int ParticleTimeProvider::month() { return Time.month(); }
int ParticleTimeProvider::month(time_t t) { return Time.month(t); }
int ParticleTimeProvider::day() { return Time.day(); }
int ParticleTimeProvider::day(time_t t) { return Time.day(t); }
int ParticleTimeProvider::hour() { return Time.hour(); }
int ParticleTimeProvider::hour(time_t t) { return Time.hour(t); }
int ParticleTimeProvider::minute() { return Time.minute(); }
int ParticleTimeProvider::minute(time_t t) { return Time.minute(t); }
int ParticleTimeProvider::second() { return Time.second(); }
int ParticleTimeProvider::second(time_t t) { return Time.second(t); }
time_t ParticleTimeProvider::now() { return Time.now(); }
void ParticleTimeProvider::setTime(time_t t) { Time.setTime(t); }
bool ParticleTimeProvider::isValid() { return Time.isValid(); }
void ParticleTimeProvider::zone(float GMT_Offset) { Time.zone(GMT_Offset); }
uint32_t ParticleTimeProvider::millis() { return ::millis(); }
void ParticleTimeProvider::delay(uint32_t ms) { ::delay(ms); }