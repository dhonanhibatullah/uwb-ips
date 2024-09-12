#ifndef __IPS_PROCESSOR_HPP__
#define __IPS_PROCESSOR_HPP__

#include <chrono>
#include <memory>
#include <thread>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "uwbips_utilities/msg/anchor_data.hpp"
#include "uwbips_utilities/msg/tag_data.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace uwbips {

    class IPSProcessor : rclcpp::Node {

        public:

            IPSProcessor(std::string node_name);

            ~IPSProcessor();

        private:

            void calcPositionTDOAChan();

            void calcPositionTDOATaylor();

            void calcPositionTWRLS();

            void calcPositionTWRNR();
    };
}

#endif