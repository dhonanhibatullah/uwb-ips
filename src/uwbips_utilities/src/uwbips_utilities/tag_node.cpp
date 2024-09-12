#include "uwbips_utilities/tag_node.hpp"



uwbips::TagNode::TagNode(const std::string node_name, uwbips::TagNode::pos_method positioning_method) : rclcpp::Node(node_name) {

    this->name                  = node_name;
    this->positioning_method    = positioning_method;
}



uwbips::TagNode::~TagNode() {
    
}