#include "headers/utils/altitude_correction.h"
#include <cmath>

double correct_altitude_for_airbrake(
    double velocity,
    double airbrake_angle,
    double measured_altitude,
    double air_density,
    double sea_level_pressure,
    double temperature
)
{
    // No correction needed if airbrake is not deployed
    if (airbrake_angle <= 0)
        return measured_altitude;
    
    // Clamp airbrake angle to physical limits
    if (airbrake_angle < 0)
        airbrake_angle = 0;
    else if (airbrake_angle > 90)
        airbrake_angle = 90;
    
    // Calculate dynamic pressure (Pa)
    double dynamic_pressure = 0.5 * air_density * velocity * velocity;
    
    const double base_error_coefficient = 0.00390;  // m/Pa (derived from datalog analysis)
    
    // Scale the error coefficient based on airbrake angle
    // Non-linear scaling: error increases superlinearly with airbrake angle
    double airbrake_scaling = pow(airbrake_angle / 5.0, 1.5);
    
    // Calculate the altitude error induced by the airbrake
    double altitude_error = dynamic_pressure * base_error_coefficient * airbrake_scaling;
    
    // Correct the measured altitude by subtracting the error
    double corrected_altitude = measured_altitude - altitude_error;
    
    return corrected_altitude;
}

double correct_altitude_advanced(
    double velocity,
    double airbrake_angle,
    double measured_altitude,
    double measured_pressure,
    double cd_current,
    double cd_nominal,
    double reference_velocity,
    double air_density
)
{
    if (airbrake_angle <= 0)
        return measured_altitude;
    
    // If drag coefficient is provided (positive value), use it for higher accuracy
    if (cd_current > 0)
    {
        double cd_increase = cd_current - cd_nominal;
        double dynamic_pressure = 0.5 * air_density * velocity * velocity;
        
        const double empirical_factor = 0.05;
        double pressure_ratio = cd_increase * empirical_factor;
        double pressure_error = dynamic_pressure * pressure_ratio;
        
        double altitude_error = pressure_error / (air_density * 9.81);
        
        return measured_altitude - altitude_error;
    }
    
    // Fall back to standard method
    return correct_altitude_for_airbrake(
        velocity, airbrake_angle, measured_altitude, air_density
    );
}
