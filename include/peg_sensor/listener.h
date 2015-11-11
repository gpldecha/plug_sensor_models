#ifndef PEG_SENSOR_LISTENER_H_
#define PEG_SENSOR_LISTENER_H_

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <armadillo>

namespace psm{

class Peg_sensor_listener{

public:

    Peg_sensor_listener(ros::NodeHandle& node, const std::string& topic_name);


private:

    void callback(const std_msgs::Float32MultiArrayConstPtr& msg);

public:

    arma::colvec   Y;

private:

    ros::Subscriber subscriber;
    std::size_t size;



};

}

#endif
