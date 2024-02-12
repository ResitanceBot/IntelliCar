#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
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

ros::Publisher pub;
ros::Subscriber sub;

geometry_msgs::Twist twist;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    // FORWARD / BACKWARD (Avance/Retroceso)
    twist.linear.x = 0;
    twist.angular.z = 0;

    SPEED = joy->axes[JOY_L_V] * MAX_SPEED; 
    ANGLE = joy->axes[JOY_R_H] * MAX_ANGLE; 
    
    if((joy->axes[JOY_L_V] < -BREAKOUT_JOYSTICKS)
        || (joy->axes[JOY_L_V] > BREAKOUT_JOYSTICKS))
    {
        twist.linear.x = SPEED;
    }
    // TURN LEFT / RIGHT (Rotacion)
    if((joy->axes[JOY_R_H] < -BREAKOUT_JOYSTICKS)
        || (joy->axes[JOY_R_H] > BREAKOUT_JOYSTICKS))
    {
        twist.angular.z = ANGLE;
    }

    // INTERFAZ TERMINAL
    printf("------------------------- \n");
    // FORWARD / BACKWARD (Avance/Retroceso)
    if(joy->axes[JOY_L_V] < -BREAKOUT_JOYSTICKS) printf("BACKWARD \n");
    else if(joy->axes[JOY_L_V] > BREAKOUT_JOYSTICKS) printf("FORWARD \n");
    // ROTATE LEFT / RIGHT (Rotacion)
    if(joy->axes[JOY_R_H] < -BREAKOUT_JOYSTICKS) printf("TURN.RIGHT \n");
    else if(joy->axes[JOY_R_H] > BREAKOUT_JOYSTICKS) printf("TURN.LEFT \n");
    // SPEED
    printf("SPEED = %f \n", SPEED);
    // ANGLE
    printf("ANLGE = %f \n", ANGLE);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gamepad_driver");

    ros::NodeHandle nh;
    pub = nh.advertise<geometry_msgs::Twist>("carla/ego_vehicle/twist", 10);
    sub = nh.subscribe("joy", 10, joyCallback);
    ros::Rate r(50);

    while (ros::ok())
    {
        pub.publish(twist);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}