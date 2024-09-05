#include "uwbips_utilities/node_manager.hpp"



uwbips::NodeManager::NodeManager() : rclcpp::Node("UWBIPS_NodeManager") {

    this->request_sub = this->create_subscription<std_msgs::msg::String>(
        "uwbips/nodeman",
        1000,
        std::bind(&uwbips::NodeManager::requestSubCallback, this, _1)
    );

    this->executor_thread = new std::thread(&uwbips::NodeManager::executorThread, this);
}



uwbips::NodeManager::~NodeManager() {}



void uwbips::NodeManager::executorThread() {

    this->executor = new rclcpp::executors::MultiThreadedExecutor;
    this->executor->spin();
}



void uwbips::NodeManager::requestSubCallback(const std_msgs::msg::String::SharedPtr msg) const {

    uwbips::NodeManager::RequestInfo temp_info;
    this->parseRequestJson(&temp_info, msg->data);
}



void uwbips::NodeManager::parseRequestJson(uwbips::NodeManager::RequestInfo *info_buf, const std::string serial_str) const {

    std::string temp_name,
                temp_type,
                temp_cmd;

    nlohmann::json data = nlohmann::json::parse(serial_str);

    temp_name   = data["name"];
    temp_type   = data["type"];
    temp_cmd    = data["cmd"];
    
    info_buf->name = temp_name;

    if(temp_type == (std::string)"anchor")
        info_buf->type = uwbips::NodeManager::TYPE_ANCHOR;

    else if(temp_type == (std::string)"anchor_master")
        info_buf->type = uwbips::NodeManager::TYPE_ANCHOR_MASTER;

    else if(temp_type == (std::string)"tag")
        info_buf->type = uwbips::NodeManager::TYPE_TAG;

    if(temp_cmd == (std::string)"create")
        info_buf->cmd = uwbips::NodeManager::CMD_CREATE_NODE;

    else if(temp_cmd == (std::string)"pause")
        info_buf->cmd = uwbips::NodeManager::CMD_PAUSE_NODE;

    else if(temp_cmd == (std::string)"start")
        info_buf->cmd = uwbips::NodeManager::CMD_START_NODE;

    else if(temp_cmd == (std::string)"destroy")
        info_buf->cmd = uwbips::NodeManager::CMD_DESTROY_NODE;
}