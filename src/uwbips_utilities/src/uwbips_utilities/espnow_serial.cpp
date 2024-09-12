#include "uwbips_utilities/espnow_serial.hpp"



uwbips::ESPNOWSerial::ESPNOWSerial(std::string node_name) : rclcpp::Node(node_name) {

    try {
        this->serial = new boost::asio::serial_port(this->boost_io);
        this->serial->open(SERIAL_PORT);
        this->serial->set_option(boost::asio::serial_port_base::baud_rate(SERIAL_BAUDRATE));
    }
    
    catch(const std::exception& err) {

        RCLCPP_ERROR(this->get_logger(), "Failed configuring serial port. Please check wheter '%s' is available or try 'sudo chmod a+rw %s'", SERIAL_PORT, SERIAL_PORT);
    }

    this->serial_tx_sub = this->create_subscription<uwbips_utilities::msg::SerialData>(
        SERIAL_TX_TOPIC,
        SERIAL_TX_QOS,
        std::bind(&uwbips::ESPNOWSerial::serialTxSubCallback, this, _1)
    );

    this->serial_rx_pub = this->create_publisher<uwbips_utilities::msg::SerialData>(
        SERIAL_RX_TOPIC,
        SERIAL_RX_QOS
    );

    this->serial_event_timer = this->create_wall_timer(
        33ms,
        std::bind(&uwbips::ESPNOWSerial::serialEventTask, this)
    );
}



uwbips::ESPNOWSerial::~ESPNOWSerial() {

    this->serial->close();
    rclcpp::shutdown();
}



void uwbips::ESPNOWSerial::serialTxSubCallback(const uwbips_utilities::msg::SerialData::SharedPtr msg) {

    this->ser_tx_q.push(msg->data);
}



void uwbips::ESPNOWSerial::serialEventTask() {

    if(!this->ser_tx_q.empty()) {

        this->serialWrite(this->ser_tx_q.front());
        this->ser_tx_q.pop();
    }

    this->serialRead();

    if(!this->ser_rx_q.empty()) {

        uwbips_utilities::msg::SerialData temp_msg = uwbips_utilities::msg::SerialData();
        temp_msg.data = this->ser_rx_q.front();
        this->serial_rx_pub->publish(temp_msg);
        this->ser_rx_q.pop();
    }
}



void uwbips::ESPNOWSerial::serialWrite(const std::string &data) {

    boost::system::error_code ec;

    boost::asio::write(
        *this->serial, 
        boost::asio::buffer(data),
        ec
    );

    if(ec) RCLCPP_ERROR(this->get_logger(), "Failed writing to serial port: %s", ec.message().c_str());
}



void uwbips::ESPNOWSerial::serialRead() {

    char buffer[256];

    boost::system::error_code ec;

    boost::asio::read(
        *this->serial,
        boost::asio::buffer(buffer),
        ec
    );

    if(ec) RCLCPP_ERROR(this->get_logger(), "Failed reading from serial port: %s", ec.message().c_str());

    std::string data_read(buffer);
    this->ser_rx_q.push(data_read);
}