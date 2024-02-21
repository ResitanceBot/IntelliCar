#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <cstdio>

#define BREAKOUT_JOYSTICKS 0.1
//! Ajustar velocidades
#define INC_SPEED 0.1
#define DEC_SPEED 0.1
#define MAX_SPEED 10.0F
#define MAX_ANGLE 1.22F

// AXES
#define JOY_L_H 0
#define JOY_L_V 1
#define LT 2
#define JOY_R_H 3
#define JOY_R_V 4
#define RT 5
#define R_L 6
#define U_D 7

// BUTTONS
#define A 0
#define B 1
#define X 2
#define Y 3
#define LB 4
#define RB 5
#define SETTINGS 6
#define WINDOWS 7
#define XBOX 8
#define BJI 9
#define BJD 10

float SPEED = 0.0F;
float ANGLE = 0.0F;

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub;

geometry_msgs::msg::Twist twist;

void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
    // FORWARD / BACKWARD (Avance/Retroceso)
    twist.linear.x = 0;
    twist.angular.z = 0;

    SPEED = joy->axes[JOY_L_V] * MAX_SPEED;
    ANGLE = joy->axes[JOY_R_H] * MAX_ANGLE;

    if ((joy->axes[JOY_L_V] < -BREAKOUT_JOYSTICKS) || (joy->axes[JOY_L_V] > BREAKOUT_JOYSTICKS))
    {
        twist.linear.x = SPEED;
    }
    // TURN LEFT / RIGHT (Rotacion)
    if ((joy->axes[JOY_R_H] < -BREAKOUT_JOYSTICKS) || (joy->axes[JOY_R_H] > BREAKOUT_JOYSTICKS))
    {
        twist.angular.z = ANGLE;
    }

    // INTERFAZ TERMINAL
    printf("------------------------- \n");
    // FORWARD / BACKWARD (Avance/Retroceso)
    if (joy->axes[JOY_L_V] < -BREAKOUT_JOYSTICKS)
        printf("BACKWARD \n");
    else if (joy->axes[JOY_L_V] > BREAKOUT_JOYSTICKS)
        printf("FORWARD \n");
    // ROTATE LEFT / RIGHT (Rotacion)
    if (joy->axes[JOY_R_H] < -BREAKOUT_JOYSTICKS)
        printf("TURN.RIGHT \n");
    else if (joy->axes[JOY_R_H] > BREAKOUT_JOYSTICKS)
        printf("TURN.LEFT \n");
    // SPEED
    printf("SPEED = %f \n", SPEED);
    // ANGLE
    printf("ANGLE = %f \n", ANGLE);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("gamepad_driver");

    pub = node->create_publisher<geometry_msgs::msg::Twist>("intellicar/ego_vehicle/control_command", 10);
    sub = node->create_subscription<sensor_msgs::msg::Joy>("joy", 10, joyCallback);

    rclcpp::Rate r(50);
    while (rclcpp::ok())
    {
        pub->publish(twist);
        rclcpp::spin_some(node);
        r.sleep();
    }
    rclcpp::shutdown();

    return 0;
}
