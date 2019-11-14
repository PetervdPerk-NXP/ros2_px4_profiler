/****************************************************************************
 *
 *   Copyright (C) 2019 Peter van der Perk. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>
#include <memory>
#include <inttypes.h>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/debug_key_value.hpp"
#include "px4_msgs/msg/airspeed.hpp"

using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("ros2_px4_profiler")
  {
    base_clock = std::chrono::system_clock::now();

    subscription_ = this->create_subscription<px4_msgs::msg::Airspeed>(
      "/Airspeed_PubSubTopic",
      [this](px4_msgs::msg::Airspeed::UniquePtr msg) {
      auto now = std::chrono::high_resolution_clock::now() - base_clock;
      auto value = std::chrono::duration_cast<std::chrono::microseconds>(now);

      uint64_t delta_time = value.count() - msg->timestamp;


      RCLCPP_INFO(this->get_logger(), "I heard: %" PRIu64 " delta: %i ", msg->timestamp, delta_time);
    });

    publisher_ = this->create_publisher<px4_msgs::msg::DebugKeyValue>("/DebugKeyValue_PubSubTopic", 10);
    auto timer_callback =
      [this]() -> void {
        auto now = std::chrono::high_resolution_clock::now() - base_clock;
        auto value = std::chrono::duration_cast<std::chrono::microseconds>(now);

        auto message = px4_msgs::msg::DebugKeyValue();
        message.timestamp = value.count();
        message.value = this->count_++;
        std::memcpy(message.key.data(), "ROS2ROSROS", 10);
        //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<px4_msgs::msg::Airspeed>::SharedPtr subscription_;
  rclcpp::Publisher<px4_msgs::msg::DebugKeyValue>::SharedPtr publisher_;
  size_t count_;
  std::chrono::time_point<std::chrono::system_clock> base_clock;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
