#include "application/Application.h"
#include <iostream>

int main() {
    try {
        // Create and run the application
        Application app(1280, 720, "Missile Physics Simulator");
        app.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}