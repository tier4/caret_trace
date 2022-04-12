// Copyright 2022 Research Institute of Systems Planning, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>
#include <memory>

#define TRACEPOINT_DEFINE
#include "caret_trace/tp.h"


using namespace std::chrono_literals;

class ClockRecorder : public rclcpp::Node
{
public:
  ClockRecorder()
  : Node("clock_recorder")
  {
    auto use_sim_time = rclcpp::Parameter("use_sim_time", true);
    set_parameter(use_sim_time);

    RCLCPP_INFO(get_logger(), "clock_recorder started to record sim time.");
    auto timer_callback = [&]() {
        auto now = this->now();
        // std::cout << static_cast<int>(now.seconds()) << std::endl;
        // The /clock topic will not be recorded while it is not published.
        if (now.nanoseconds() == 0) {
          return;
        }
        RCLCPP_DEBUG(get_logger(), "sim_time recorded: %ld.", now.nanoseconds());
        tracepoint(TRACEPOINT_PROVIDER, sim_time, now.nanoseconds());
      };
    timer_ = create_wall_timer(1s, timer_callback);
  }
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClockRecorder>());
  rclcpp::shutdown();

  return 0;
}
