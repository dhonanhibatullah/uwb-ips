#include "uwbips_utilities/anchor_node.hpp"



uwbips::AnchorNode::AnchorNode(const std::string node_name, bool is_master) : rclcpp::Node(node_name) {

    this->name      = node_name;
    this->is_master = is_master;
}



uwbips::AnchorNode::~AnchorNode() {

}