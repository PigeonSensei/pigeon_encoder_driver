#ifndef GPIO_ENCODER_H
#define GPIO_ENCODER_H

#include <ros/ros.h>
#include <wiringPi.h>

#include "encoder_msgs/EncoderCount.h"

#include "encoder_msgs/ResetEncoderCount.h"

#define PI 3.1415927

struct Encoder
{
  int A;
  int B;
};

Encoder pin_encoder_[2];
int encoder_counter_[2];

class GpioEncoder
{
public:
    GpioEncoder(ros::NodeHandle &n)
      : publisher_encoder_count_(n.advertise<encoder_msgs::EncoderCount>("encoder_count",1000)),
        service_server_reset_encoder_count_(n.advertiseService("resetEncoderCount", &GpioEncoder::ResetEncoderCountServiceCallback , this))
       {
          // open run
          ROS_INFO("l298n_node Open");
          GetROSParam();
          SetupPin();
          SetupInterruptPin();
       }
       ~GpioEncoder()
       {
          // close run
          ROS_INFO("l298n_node Close");
       }

    int SetupPin();

    int GetROSParam();

    bool ResetEncoderCountServiceCallback(encoder_msgs::ResetEncoderCount::Request &req,
                                          encoder_msgs::ResetEncoderCount::Response &res);

    static void GetEncoderCount0A();

    static void GetEncoderCount0B();

    static void GetEncoderCount1A();

    static void GetEncoderCount1B();

    void SetupInterruptPin();

    void UpdateEncoderCount();

    void Update();

    void publisher();

    void Spin();


private:

    ros::Time time_now_;

    encoder_msgs::EncoderCount encoder_count_;
    ros::Publisher publisher_encoder_count_;
    ros::ServiceServer service_server_reset_encoder_count_;
};


#endif // GPIO_ENCODER_H
