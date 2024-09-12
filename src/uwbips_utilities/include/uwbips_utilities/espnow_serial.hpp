#ifndef __ESPNOW_SERIAL_HPP__
#define __ESPNOW_SERIAL_HPP__

#include <chrono>
#include <memory>
#include <thread>
#include <mutex>
#include <queue>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "boost/asio.hpp"
#include "uwbips_utilities/msg/serial_data.hpp"

#define SERIAL_PORT     "/dev/ttyUSB0"
#define SERIAL_BAUDRATE 115200
#define SERIAL_TX_TOPIC "espnowser/tx"
#define SERIAL_TX_QOS   1000
#define SERIAL_RX_TOPIC "espnowser/rx"
#define SERIAL_RX_QOS   1000

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace uwbips {

    class ESPNOWSerial : public rclcpp::Node {

        public:

            ESPNOWSerial(std::string node_name);

            ~ESPNOWSerial();

        private:

            std::queue<std::string> ser_tx_q,
                                    ser_rx_q;

            boost::asio::io_service boost_io;

            boost::asio::serial_port *serial;

            rclcpp::Subscription<uwbips_utilities::msg::SerialData>::SharedPtr serial_tx_sub;

            rclcpp::Publisher<uwbips_utilities::msg::SerialData>::SharedPtr serial_rx_pub;

            rclcpp::TimerBase::SharedPtr serial_event_timer;

            void serialTxSubCallback(const uwbips_utilities::msg::SerialData::SharedPtr msg);

            void serialEventTask();

            void serialWrite(const std::string &data);

            void serialRead();
    };
}

#endif