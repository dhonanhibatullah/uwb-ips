#ifndef __NODE_MANAGER_HPP__
#define __NODE_MANAGER_HPP__

#include <chrono>
#include <memory>
#include <thread>
#include <map>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "json.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace uwbips {

    class NodeManager : public rclcpp::Node {

        public:

            NodeManager();

            ~NodeManager();

        private:

            enum node_type : uint8_t {
                TYPE_ANCHOR,
                TYPE_ANCHOR_MASTER,
                TYPE_TAG
            };

            enum manager_cmd : uint8_t {
                CMD_CREATE_NODE,
                CMD_PAUSE_NODE,
                CMD_START_NODE,
                CMD_DESTROY_NODE
            };

            struct RequestInfo {
                std::string name;
                node_type   type;
                manager_cmd cmd;
            };

            std::thread *executor_thread;

            rclcpp::executors::MultiThreadedExecutor *executor;

            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr request_sub;

            void executorThread();

            void requestSubCallback(const std_msgs::msg::String::SharedPtr msg) const;

            void parseRequestJson(RequestInfo *info_buf, const std::string serial_str) const;
    };
}

#endif