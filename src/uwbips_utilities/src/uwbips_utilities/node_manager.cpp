#include "uwbips_utilities/node_manager.hpp"



uwbips::NodeManager::NodeManager(std::string node_name) : rclcpp::Node(node_name) {

    this->request_sub = this->create_subscription<uwbips_utilities::msg::NodemanReq>(
        NODE_MANAGER_TOPIC,
        NODE_MANAGER_QOS,
        std::bind(&uwbips::NodeManager::requestSubCallback, this, _1)
    );

    this->executor_thread = new std::thread(&uwbips::NodeManager::executorThread, this);

    RCLCPP_INFO(this->get_logger(), "UWB IPS Node Manager started");
}



uwbips::NodeManager::~NodeManager() {
    
    this->shutdown();
    RCLCPP_INFO(this->get_logger(), "UWB IPS Node Manager stopped");
}



void uwbips::NodeManager::executorThread() {

    this->executor = new rclcpp::executors::MultiThreadedExecutor;
    this->executor->spin();
}



void uwbips::NodeManager::requestSubCallback(const uwbips_utilities::msg::NodemanReq::SharedPtr msg) {

    uwbips::NodeManager::RequestInfo req_info;

    if(this->parseRequestMessage(&req_info, msg)) 
        this->requestExecute(&req_info);
}



bool uwbips::NodeManager::parseRequestMessage(uwbips::NodeManager::RequestInfo *req_buf, const uwbips_utilities::msg::NodemanReq::SharedPtr msg) {
    
    req_buf->name = msg->name;

    if(msg->type == (std::string)"anchor")
        req_buf->type = uwbips::NodeManager::TYPE_ANCHOR;

    else if(msg->type == (std::string)"anchor_master")
        req_buf->type = uwbips::NodeManager::TYPE_ANCHOR_MASTER;

    else if(msg->type == (std::string)"tag_twr")
        req_buf->type = uwbips::NodeManager::TYPE_TAG_TWR;

    else if(msg->type == (std::string)"tag_tdoa")
        req_buf->type = uwbips::NodeManager::TYPE_TAG_TDOA;

    else {
        RCLCPP_ERROR(this->get_logger(), "Invalid type: '%s'", msg->type.c_str());
        return false;
    }

    if(msg->command == (std::string)"create")
        req_buf->cmd = uwbips::NodeManager::CMD_CREATE_NODE;

    else if(msg->command == (std::string)"destroy")
        req_buf->cmd = uwbips::NodeManager::CMD_DESTROY_NODE;

    else {
        RCLCPP_ERROR(this->get_logger(), "Invalid cmd: '%s'", msg->command.c_str());
        return false;
    }

    return true;
}



void uwbips::NodeManager::requestExecute(uwbips::NodeManager::RequestInfo *req) {

    switch(req->cmd) {

        case uwbips::NodeManager::CMD_CREATE_NODE:
            this->createNode(req->name, req->type);
            break;

        case uwbips::NodeManager::CMD_DESTROY_NODE:
            this->destroyNode(req->name, req->type);
            break;
    }
}



void uwbips::NodeManager::createNode(std::string name, uwbips::NodeManager::node_type type) {

    if(this->anchor_node_map.find(name) != this->anchor_node_map.end() || this->tag_node_map.find(name) != this->tag_node_map.end()) {
        
        RCLCPP_ERROR(this->get_logger(), "Node with name: '%s' already exists, node creation aborted", name.c_str());
        return;
    }

    std::shared_ptr<uwbips::AnchorNode> new_anchor_node;
    std::shared_ptr<uwbips::TagNode>    new_tag_node;

    switch(type) {

        case uwbips::NodeManager::TYPE_ANCHOR:

            new_anchor_node = std::make_shared<uwbips::AnchorNode>(name, false);
            this->anchor_node_map[name] = new_anchor_node;
            this->executor->add_node(new_anchor_node);

            RCLCPP_INFO(this->get_logger(), "Node with name: '%s', type: 'ANCHOR', created", name.c_str());
            break;

        case uwbips::NodeManager::TYPE_ANCHOR_MASTER:

            new_anchor_node = std::make_shared<uwbips::AnchorNode>(name, true);
            this->anchor_node_map[name] = new_anchor_node;
            this->executor->add_node(new_anchor_node);

            RCLCPP_INFO(this->get_logger(), "Node with name: '%s', type: 'ANCHOR_MASTER', created", name.c_str());
            break;

        case uwbips::NodeManager::TYPE_TAG_TWR:

            new_tag_node = std::make_shared<uwbips::TagNode>(name, uwbips::TagNode::POSITIONING_TWR);
            this->tag_node_map[name] = new_tag_node;
            this->executor->add_node(new_tag_node);

            RCLCPP_INFO(this->get_logger(), "Node with name: '%s', type: 'TAG_TWR', created", name.c_str());
            break;

        case uwbips::NodeManager::TYPE_TAG_TDOA:

            new_tag_node = std::make_shared<uwbips::TagNode>(name, uwbips::TagNode::POSITIONING_TDOA);
            this->tag_node_map[name] = new_tag_node;
            this->executor->add_node(new_tag_node);

            RCLCPP_INFO(this->get_logger(), "Node with name: '%s', type: 'TAG_TDOA', created", name.c_str());
            break;

        default:
            break;
    }

}



void uwbips::NodeManager::destroyNode(std::string name, uwbips::NodeManager::node_type type) {
    
    if(
        type == uwbips::NodeManager::TYPE_ANCHOR || 
        type == uwbips::NodeManager::TYPE_ANCHOR_MASTER || 
        type == uwbips::NodeManager::TYPE_ANCHOR_ALL) {

        if(this->anchor_node_map.find(name) == this->anchor_node_map.end()) {
        
            RCLCPP_ERROR(this->get_logger(), "Node with name: '%s', type: 'ANCHOR | ANCHOR_MASTER', does not exist, node destruction aborted", name.c_str());
            return;
        }

        this->executor->remove_node(this->anchor_node_map[name]);
        this->anchor_node_map.erase(name);
        
        RCLCPP_INFO(this->get_logger(), "Node with name: '%s', destroyed", name.c_str());
    }

    else if(
        type == uwbips::NodeManager::TYPE_TAG_TWR || 
        type == uwbips::NodeManager::TYPE_TAG_TDOA || 
        type == uwbips::NodeManager::TYPE_TAG_ALL) {

        if(this->tag_node_map.find(name) == this->tag_node_map.end()) {
        
            RCLCPP_ERROR(this->get_logger(), "Node with name: '%s', type: 'TAG_TWR | TAG_TDOA' does not exist, node destruction aborted", name.c_str());
            return;
        }

        this->executor->remove_node(this->tag_node_map[name]);
        this->tag_node_map.erase(name);
        
        RCLCPP_INFO(this->get_logger(), "Node with name: '%s', destroyed", name.c_str());
    }
}



void uwbips::NodeManager::shutdown() {

    for(auto it = this->tag_node_map.begin(); it != this->tag_node_map.end(); ++it) {
        
        std::string node_name = it->first;
        this->destroyNode(node_name, uwbips::NodeManager::TYPE_TAG_ALL);
    }

    for(auto it = this->anchor_node_map.begin(); it != this->anchor_node_map.end(); ++it) {
        
        std::string node_name = it->first;
        this->destroyNode(node_name, uwbips::NodeManager::TYPE_ANCHOR_ALL);
    }

    this->executor_thread->join();
    rclcpp::shutdown();
}

