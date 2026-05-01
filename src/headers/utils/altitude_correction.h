#ifndef ALTITUDE_CORRECTION_H
#define ALTITUDE_CORRECTION_H

double correct_altitude_for_airbrake(
    double velocity,
    double airbrake_angle,
    double measured_altitude,
    double air_density = 1.174939201718265749,
    double sea_level_pressure = 101325.0,
    double temperature = 288.15
);

double correct_altitude_advanced(
    double velocity,
    double airbrake_angle,
    double measured_altitude,
    double measured_pressure = -1.0,
    double cd_current = -1.0,
    double cd_nominal = 0.3,
    double reference_velocity = 100.0,
    double air_density = 1.174939201718265749
);

#endif // ALTITUDE_CORRECTION_H
