#include "gpio_encoder/gpio_encoder.h"

int GpioEncoder::SetupPin()
{
  if(wiringPiSetupGpio() == -1)
  {
    ROS_INFO("Fail Setup WiringPI");
    return -1;
  }

  pinMode(pin_encoder_[0].A, INPUT);
  pinMode(pin_encoder_[0].B, INPUT);

  pinMode(pin_encoder_[1].A, INPUT);
  pinMode(pin_encoder_[1].B, INPUT);

  ROS_INFO("Success Setup WiringPI");

  return 0;
}

int GpioEncoder::GetROSParam()
{
  ros::NodeHandle n("~");
  n.param<int>("Encoder0_A", pin_encoder_[0].A, 0);
  n.param<int>("Encoder0_B", pin_encoder_[0].B, 0);
  n.param<int>("Encoder1_A", pin_encoder_[1].A, 0);
  n.param<int>("Encoder1_B", pin_encoder_[1].B, 0);

  return 0;
}

bool GpioEncoder::ResetEncoderCountServiceCallback(encoder_msgs::ResetEncoderCount::Request &req, encoder_msgs::ResetEncoderCount::Response &res)
{
  if(req.command == true)
  {
    encoder_counter_[0] = 0;
    encoder_counter_[1] = 0;
    res.result = "true";
    res.message = "Reset Encoder Count!";
    ROS_INFO("Reset Encoder Count!");
  }
  return true;
}

void GpioEncoder::GetEncoderCount0A()
{
  if(digitalRead(pin_encoder_[0].A) == digitalRead(pin_encoder_[0].B)) encoder_counter_[0]++;
  else encoder_counter_[0]--;
}

void GpioEncoder::GetEncoderCount0B()
{
  if(digitalRead(pin_encoder_[0].A) == digitalRead(pin_encoder_[0].B)) encoder_counter_[0]--;
  else encoder_counter_[0]++;
}

void GpioEncoder::GetEncoderCount1A()
{
  if(digitalRead(pin_encoder_[1].A) == digitalRead(pin_encoder_[1].B)) encoder_counter_[1]++;
  else encoder_counter_[1]--;
}

void GpioEncoder::GetEncoderCount1B()
{
  if(digitalRead(pin_encoder_[1].A) == digitalRead(pin_encoder_[1].B)) encoder_counter_[1]--;
  else encoder_counter_[1]++;
}

void GpioEncoder::SetupInterruptPin()
{
  wiringPiISR(pin_encoder_[0].A, INT_EDGE_FALLING, &GpioEncoder::GetEncoderCount0A);
  wiringPiISR(pin_encoder_[0].B, INT_EDGE_FALLING, &GpioEncoder::GetEncoderCount0B);
  wiringPiISR(pin_encoder_[1].A, INT_EDGE_FALLING, &GpioEncoder::GetEncoderCount1A);
  wiringPiISR(pin_encoder_[1].B, INT_EDGE_FALLING, &GpioEncoder::GetEncoderCount1B);
}

void GpioEncoder::UpdateEncoderCount()
{
  encoder_count_.Stamp = time_now_;
  encoder_count_.Count[0] = encoder_counter_[0];
  encoder_count_.Count[1] = encoder_counter_[1];
}

void GpioEncoder::Update()
{
  UpdateEncoderCount();
}

void GpioEncoder::publisher()
{
  publisher_encoder_count_.publish(encoder_count_);
}

void GpioEncoder::Spin()
{
  time_now_ = ros::Time::now();
  Update();
  publisher();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gpio_encoder_node");
  ros::NodeHandle n;

  ros::Rate loop_rate(60);

  GpioEncoder gpio_encoder(n);

  while (ros::ok())
  {
    gpio_encoder.Spin();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}
