#ifndef PLUG_SENSOR_LISTENER_H_
#define PLUG_SENSOR_LISTENER_H_

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <armadillo>

namespace psm{

class Plug_sensor_listener{

public:

    Plug_sensor_listener(ros::NodeHandle& node,const std::string& topic_name,std::size_t data_size);

private:

    void callback(const std_msgs::Float32MultiArrayConstPtr& msg);

public:

    arma::fcolvec   data;

private:

    ros::Subscriber subscriber;
    std::size_t     data_size;



};

}

#endif
