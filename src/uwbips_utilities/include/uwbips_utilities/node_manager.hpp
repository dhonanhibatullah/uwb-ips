#ifndef __NODE_MANAGER_HPP__
#define __NODE_MANAGER_HPP__

#include <chrono>
#include <memory>
#include <thread>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "uwbips_utilities/anchor_node.hpp"
#include "uwbips_utilities/tag_node.hpp"
#include "uwbips_utilities/msg/nodeman_control.hpp"

#define NODEMAN_CONTROL_TOPIC   "uwbips/nodeman/control"
#define NODEMAN_CONTROL_QOS     1000
#define NODEMAN_NODE_TOPIC      "uwbips/node/"
#define NODEMAN_NODE_QOS        1000

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace uwbips {

    class NodeManager : public rclcpp::Node {

        public:

            NodeManager(std::string node_name);

            ~NodeManager();

        private:

            enum node_type : uint8_t {
                TYPE_ANCHOR,
                TYPE_ANCHOR_MASTER,
                TYPE_TAG_TWR,
                TYPE_TAG_TDOA,
                TYPE_TAG_ALL,
                TYPE_ANCHOR_ALL
            };

            enum manager_cmd : uint8_t {
                CMD_CREATE_NODE,
                CMD_DESTROY_NODE
            };

            struct RequestInfo {
                std::string name;
                node_type   type;
                manager_cmd cmd;
            };

            std::thread *executor_thread;

            std::map<std::string, std::shared_ptr<AnchorNode>> anchor_node_map;

            std::map<std::string, std::shared_ptr<TagNode>> tag_node_map;

            rclcpp::executors::MultiThreadedExecutor *executor;

            rclcpp::Subscription<uwbips_utilities::msg::NodemanControl>::SharedPtr request_sub;

            void requestSubCallback(const uwbips_utilities::msg::NodemanControl::SharedPtr msg);
            
            void executorThread();

            bool parseRequestMessage(RequestInfo *req_buf, const uwbips_utilities::msg::NodemanControl::SharedPtr serial_str);
    
            void requestExecute(RequestInfo *req);

            void createNode(std::string name, node_type type);

            void destroyNode(std::string name, node_type type);

            void shutdown();
    };
}

#endif