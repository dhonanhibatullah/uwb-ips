#ifndef __TAG_NODE_HPP__
#define __TAG_NODE_HPP__

#include <chrono>
#include <memory>
#include <thread>
#include <map>
#include <string>
#include "rclcpp/rclcpp.hpp"

#define NODE_MANAGER_TOPIC  "uwbips/nodeman"
#define NODE_MANAGER_QOS    1000

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace uwbips {
    
    class TagNode : public rclcpp::Node {

        public:

            enum pos_method : uint8_t {
                POSITIONING_TWR,
                POSITIONING_TDOA
            };

            TagNode(const std::string node_name, pos_method positioning_method);

            ~TagNode();

        private:

            std::string name;

            pos_method positioning_method;
    };

}

#endif