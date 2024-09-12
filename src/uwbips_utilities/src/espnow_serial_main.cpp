#include "uwbips_utilities/espnow_serial.hpp"



int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);
    std::shared_ptr<uwbips::ESPNOWSerial> espnowser_node = std::make_shared<uwbips::ESPNOWSerial>("uwbips_espnowser");
    rclcpp::spin(espnowser_node);

    return 0;
}