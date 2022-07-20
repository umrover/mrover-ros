#ifndef CONTROLLER_MAP_H
#define CONTROLLER_MAP_H
#include "ROSHandler.h"
#include <fstream>
#include <stdint.h>
#include <string>
#include <unordered_map>


//Forward declaration of Controller class for compilation
class Controller;

/*
The ControllerMap class creates a hash table of virtual Controller objects from the config file
located at "config/motors.yaml". These virtual Controllers are used to contact the physical
controller on the rover, across both RA/SA configurations.
*/
class ControllerMap {
private:
    //Map of i2c_addresses to "live" virtual controllers
    inline static std::unordered_map<uint8_t, std::string> live_map = std::unordered_map<uint8_t, std::string>();

    //Map of virtual controllers to supposed i2c addresses
    inline static std::unordered_map<std::string, uint8_t> name_map = std::unordered_map<std::string, uint8_t>();

    //Helper function to calculate an i2c address based off of nucleo # and channel #
    static uint8_t calculate_i2c_address(uint8_t nucleo, uint8_t channel);

public:
    //Map of virtual controller names to virtual Controller objects
    inline static std::unordered_map<std::string, Controller*> controllers = std::unordered_map<std::string, Controller*>();

    //Initialization function
    static void init();

    //Returns supposed i2c address based off of virtual controller name
    static uint8_t get_i2c_address(std::string name);

    //Returns whether virtual controller name is in the i2c address to "live" virtual controller map
    static bool check_if_live(std::string name);

    //Forces this virtual controller into the i2c address to "live" virtual controller map, replacing any virtual controller already at that i2c address
    static void make_live(std::string name);
};

#endif
