#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <wiringPi.h>
#include <map>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class AJ_SR04M_Node : public rclcpp::Node
{
public:
    AJ_SR04M_Node() : Node("aj_sr04m_node")
    {
        this->declare_parameter<int>("sensor_index", -1);
        this->get_parameter("sensor_index", sensor_index);

        if (wiringPiSetup() == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "WiringPi initialization failed!");
            rclcpp::shutdown();
        }

        // for (const auto& [name, pins] : sensors_)
        // {
        //     // GPIO 설정
        //     pinMode(pins[0], OUTPUT);
        //     pinMode(pins[1], INPUT);
        //     pullUpDnControl(pins[1], PUD_DOWN);
        //     digitalWrite(pins[0], LOW);
            
        //     // 각 센서에 대한 퍼블리셔 초기화
        //     publishers_[name] = this->create_publisher<sensor_msgs::msg::Range>(name, 10);
        // }

        // if (sensor_index >= 0 && sensor_index < sensors_.size())
        if (sensor_index >= 0 && sensor_index < static_cast<int>(sensors_.size()))
        {
            auto sensor_it = std::next(sensors_.begin(), sensor_index);
            const auto& [name, pins] = *sensor_it;

            // GPIO 설정
            pinMode(pins[0], OUTPUT);
            pinMode(pins[1], INPUT);
            pullUpDnControl(pins[1], PUD_DOWN);
            digitalWrite(pins[0], LOW);

            // 퍼블리셔 초기화
            publisher_ = this->create_publisher<sensor_msgs::msg::Range>(name, 10);
            timer_ = this->create_wall_timer(200ms, std::bind(&AJ_SR04M_Node::timer_callback, this));
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid sensor index!");
        }        
        // timer_ = this->create_wall_timer(100ms, std::bind(&AJ_SR04M_Node::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // for (const auto& [name, pins] : sensors_)
        // {
        //     auto range_msg = get_sensor_data(pins);
        //     publishers_[name]->publish(range_msg);
        //     delayMicroseconds(1000);
        // }
        auto sensor_it = std::next(sensors_.begin(), sensor_index);
        const auto& [name, pins] = *sensor_it;
        auto range_msg = get_sensor_data(pins);
        publisher_->publish(range_msg);        
    }

    sensor_msgs::msg::Range get_sensor_data(const std::vector<int>& pins)
    {
        sensor_msgs::msg::Range range_msg;
        
        int TRIG = pins[0];
        int ECHO = pins[1];

        digitalWrite(TRIG, LOW);
        delayMicroseconds(1000);

        digitalWrite(TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG, LOW);

        long timeout = micros() + 1000000; // 1초 타임아웃
        while (digitalRead(ECHO) == HIGH && micros() < timeout);

        long start_time = micros();
        while (digitalRead(ECHO) == LOW && micros() < timeout);

        long travel_time = micros() - start_time;

        if (travel_time > 0)
        {
            float distance = 100*((travel_time/1000000.0)*340.29)/2;
            range_msg.range = distance; // 센치미터를 미터로 변환
        }
        else
        {
            range_msg.range = -1.0; // 타임아웃이나 에러 경우
        }

        range_msg.header.stamp = this->now();
        range_msg.header.frame_id = "sensor_" + std::to_string(TRIG);

        return range_msg;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    // std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr> publishers_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;

    // 센서 GPIO 설정, Name, 트리거 핀정보, 에코핀 정보
    std::map<std::string, std::vector<int>> sensors_ = {
        {"front_left", {7, 1}},
        {"left", {0, 4}},
        {"back", {21, 11}},
        {"front_right", {2, 5}},
        {"right", {3, 6}},
        {"bottom", {22, 26}},
    };

    int sensor_index = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<AJ_SR04M_Node>());
    // rclcpp::shutdown();
    auto node = std::make_shared<AJ_SR04M_Node>(); // What is auto??
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();    
    return 0;
}