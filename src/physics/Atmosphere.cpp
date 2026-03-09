#include "Atmosphere.h"
#include <cmath>

Atmosphere::Atmosphere(float density)
    : m_density(density) {
}

float Atmosphere::calculateDensityAtAltitude(float altitude) const {
    // International Standard Atmosphere model (simplified)
    
    // Constants
    const float R = 287.05f;          // Specific gas constant for air (J/(kg·K))
    const float g = 9.80665f;         // Gravitational acceleration (m/s²)
    const float T0 = 288.15f;         // Sea level standard temperature (K)
    const float P0 = 101325.0f;       // Sea level standard pressure (Pa)
    const float L = 0.0065f;          // Temperature lapse rate (K/m)
    
    // Temperature at altitude
    float T = T0 - L * altitude;
    
    // If in troposphere (altitude < 11 km)
    if (altitude < 11000.0f) {
        // Pressure at altitude
        float P = P0 * std::pow((T / T0), (g / (R * L)));
        
        // Density at altitude (ideal gas law)
        return P / (R * T);
    }
    // For higher altitudes, we'd need a more complex model with stratosphere layers
    else {
        // Simplified exponential model for higher altitudes
        return 1.225f * std::exp(-altitude / 8000.0f);
    }
} 