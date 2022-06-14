#include "ControllerMap.h"
#include "Controller.h"

// Helper function to calculate an i2c address based off of nucleo # and channel #
uint8_t ControllerMap::calculate_i2c_address(uint8_t nucleo, uint8_t channel)
{
    return (nucleo << 4) | channel;
}

// Helper function to get the path of the config file
std::string ControllerMap::get_config()
{
    // TODO - fix path
    std::string configPath = getenv("mrover");
    configPath += "/src/motors/config/controller_config.json";
    std::ifstream configFile;
    configFile.open(configPath);

    std::string config = "";
    std::string line;
    while (configFile >> line)
    {
        config += line;
    }

    return config;
}

// Initialization function
void ControllerMap::init()
{
    rapidjson::Document document;
    document.Parse(get_config().c_str());

    rapidjson::Value &root = document;
    assert(root.IsArray());

    for (rapidjson::SizeType i = 0; i < root.Size(); ++i)
    {
        assert(root[i].HasMember("name") && root[i]["name"].IsString());
        std::string name = root[i]["name"].GetString();

        assert(root[i].HasMember("type") && root[i]["type"].IsString());
        std::string type = root[i]["type"].GetString();

        assert(root[i].HasMember("nucleo") && root[i]["nucleo"].IsInt());
        uint8_t nucleo = root[i]["nucleo"].GetInt();

        assert(root[i].HasMember("channel") && root[i]["channel"].IsInt());
        uint8_t channel = root[i]["channel"].GetInt();

        controllers[name] = new Controller(name, type);
        name_map[name] = calculate_i2c_address(nucleo, channel);

        if (root[i].HasMember("quadCPR") && root[i]["quadCPR"].IsFloat())
        {
            controllers[name]->quad_cpr = root[i]["quadCPR"].GetFloat();
        }
        if (root[i].HasMember("kP") && root[i]["kP"].IsFloat())
        {
            controllers[name]->kP = root[i]["kP"].GetFloat();
        }
        if (root[i].HasMember("kI") && root[i]["kI"].IsFloat())
        {
            controllers[name]->kI = root[i]["kI"].GetFloat();
        }
        if (root[i].HasMember("kD") && root[i]["kD"].IsFloat())
        {
            controllers[name]->kD = root[i]["kD"].GetFloat();
        }
        if (root[i].HasMember("inversion") && root[i]["inversion"].IsFloat())
        {
            controllers[name]->inversion = root[i]["inversion"].GetFloat();
        }
        printf("Virtual Controller %s of type %s on Nucleo %i channel %i \n", name.c_str(), type.c_str(), nucleo, channel);
    }
}

// Returns supposed i2c address based off of virtual controller name
uint8_t ControllerMap::get_i2c_address(std::string name)
{
    return name_map[name];
}

// Returns whether virtual controller name is in the "live" virtual controller to i2c address map
bool ControllerMap::check_if_live(std::string name)
{
    return (name == live_map[get_i2c_address(name)]);
}

// Forces this virtual controller into the i2c address to "live" virtual controller map, replacing any virtual controller already at that i2c address
void ControllerMap::make_live(std::string name)
{
    live_map[get_i2c_address(name)] = name;
}
