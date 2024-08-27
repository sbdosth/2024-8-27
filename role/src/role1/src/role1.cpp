#include "rclcpp/rclcpp.hpp" //导入库文件
#include "geometry_msgs/msg/twist.hpp"

class DrawCircle : public rclcpp::Node
{
public:
    DrawCircle()
        : Node("role1_node")
    {
        // 创建一个Twist消息类型的发布者，用于发布速度指令
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // 创建一个定时器，每100毫秒调用一次publishVelocity成员函数
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&DrawCircle::publishVelocity, this));
    }

private:
    void publishVelocity()
    {
        // 创建Twist消息实例
        geometry_msgs::msg::Twist twist;

        // 设置线速度和角速度
        twist.linear.x = 0.2;  // m/s
        twist.angular.z = 1.0; // rad/s

        // 发布Twist消息
        publisher_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // 指向Twist消息发布者的智能指针
    rclcpp::TimerBase::SharedPtr timer_;                                // 指向定时器的智能指针
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // 初始化客户端库

    auto node = std::make_shared<DrawCircle>(); // 创建 DrawCircle 类的实例

    rclcpp::spin(node); // spin循环节点

    rclcpp::shutdown(); // 关闭客户端库

    return 0;
}
