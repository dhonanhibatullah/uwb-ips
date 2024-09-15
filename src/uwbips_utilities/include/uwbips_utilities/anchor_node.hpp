#ifndef __ANCHOR_NODE_HPP__
#define __ANCHOR_NODE_HPP__

#include <chrono>
#include <memory>
#include <thread>
#include <map>
#include <string>
#include "rclcpp/rclcpp.hpp"

// #define NODE_MANAGER_TOPIC  "uwbips/nodeman/node_req"
// #define NODE_MANAGER_QOS    1000

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace uwbips {
    
    class AnchorNode : public rclcpp::Node {

        public:

            AnchorNode(const std::string node_name, bool is_master);

            ~AnchorNode();

        private:

            std::string name;

            bool is_master;
    };
}

#endif