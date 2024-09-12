#include "uwbips_utilities/node_manager.hpp"



int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);
    std::shared_ptr<uwbips::NodeManager> nodeman = std::make_shared<uwbips::NodeManager>("uwbips_nodeman");
    rclcpp::spin(nodeman);
    
    return 0;
}