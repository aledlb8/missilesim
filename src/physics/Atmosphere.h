#pragma once

class Atmosphere {
public:
    Atmosphere(float density = 1.225f); // Default is sea level air density
    ~Atmosphere() = default;
    
    // Getter and setter for air density
    float getDensity() const { return m_density; }
    void setDensity(float density) { m_density = density; }
    
    // Calculate air density based on altitude using the International Standard Atmosphere model
    float calculateDensityAtAltitude(float altitude) const;
    
private:
    float m_density; // Air density in kg/m³
}; 