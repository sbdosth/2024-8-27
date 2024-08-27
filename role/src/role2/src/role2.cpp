#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <random>
#include <cmath>

using namespace std::chrono_literals;

//继承于rclcpp::Node
class TurtleChase : public rclcpp::Node
{
public:
    TurtleChase()
    : Node("role2")//节点
    {
        // 初始化乌龟a的速度发步者
        turtle_a_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle_a/cmd_vel", 10);
        // 初始化乌龟b的速度发步者
        turtle_b_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("turtle_b/pose", 10);
        // 订阅乌龟a的位置信息
        turtle_a_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "turtle_a/pose", 10, std::bind(&TurtleChase::turtleAUpdate, this, std::placeholders::_1));
        // 订阅乌龟b的位置信息
        turtle_b_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "turtle_b/pose", 10, std::bind(&TurtleChase::turtleBUpdate, this, std::placeholders::_1));

        // 设置定时器进行乌龟运动
        timer_ = this->create_wall_timer(100ms, std::bind(&TurtleChase::timerCallback, this));

        // 初始化乌龟B的随机位置
        std::random_device rd; //创建随机数
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis_x(0.0, 10.0);//创建均匀分布的随机数生成器
        std::uniform_real_distribution<> dis_y(0.0, 10.0);//创建均匀分布的随机数生成器

        //为乌龟b设置初始位置
        turtle_b_pose_.position.x = dis_x(gen);
        turtle_b_pose_.position.y = dis_y(gen);

        turtle_b_pose_.orientation.w = 1.0; // 无旋转
 
        //发布乌龟b的初始位置
        turtle_b_publisher_->publish(turtle_b_pose_);
    }

private:
    //回调函数
    void timerCallback()
    {
        if (turtle_a_pose_received_ && turtle_b_pose_received_)
        {
            //计算两个乌龟之间距离
            double distance = std::hypot(turtle_a_pose_.position.x - turtle_b_pose_.position.x,
                                         turtle_a_pose_.position.y - turtle_b_pose_.position.y);

            // 距离小于0.5则认为追上
            if (distance < 0.5) 
            {
                // 随机生成B乌龟的新位置
                std::random_device rd; //创建随机数设备
                std::mt19937 gen(rd());
                std::uniform_real_distribution<> dis_x(0.0, 10.0);//创建均匀分布的随机数生成器
                std::uniform_real_distribution<> dis_y(0.0, 10.0);//创建均匀分布的随机数生成器
                
                //生成新的坐标
                turtle_b_pose_.position.x = dis_x(gen);
                turtle_b_pose_.position.y = dis_y(gen);
                turtle_b_publisher_->publish(turtle_b_pose_);
            }
            else
            {
                // 乌龟a朝乌龟b方向移动
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = 1.0 * (turtle_b_pose_.position.x - turtle_a_pose_.position.x);//设置线速度
                cmd_vel.angular.z = 1.0 * (turtle_b_pose_.position.y - turtle_a_pose_.position.y);//设置角速度
                turtle_a_publisher_->publish(cmd_vel);//发布速度命令
            }
        }
    }

    //回调函数处理乌龟a的位置更新
    void turtleAUpdate(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        turtle_a_pose_ = *msg; // 更新乌龟a的位置
        turtle_a_pose_received_ = true; // 标记乌龟a的位置已经被接收
    }
   
    //回调函数处理乌龟b的位置更新
    void turtleBUpdate(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        turtle_a_pose_ = *msg; // 更新乌龟b的位置
        turtle_a_pose_received_ = true; // 标记乌龟b的位置已经被接收
    }

    // 乌龟a的速度发布者
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle_a_publisher_;
    //乌龟b的速度发布者
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr turtle_b_publisher_;
    // 乌龟a的位置订阅者
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr turtle_a_pose_subscriber_;
    // 乌龟b的位置订阅者
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr turtle_b_pose_subscriber_;
    //定时器
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Pose turtle_a_pose_; // 乌龟A的位置
    geometry_msgs::msg::Pose turtle_b_pose_; // 乌龟B的位置
    bool turtle_a_pose_received_ = false; // 是否接收到乌龟A的位置
    bool turtle_b_pose_received_ = false; // 是否接收到乌龟B的位置
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);// 初始化客户端库

    auto node = std::make_shared<TurtleChase>(); // 创建 DrawCircle 类的实例

    rclcpp::spin(node); // spin循环节点

    rclcpp::shutdown();// 关闭客户端库
    return 0;
}
