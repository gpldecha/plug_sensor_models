#ifndef PEG_CLASSIFIER_PUBLISHER_H_
#define PEG_CLASSIFIER_PUBLISHER_H_

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#include <armadillo>

namespace psm{


class Peg_classifier_publisher{

public:

    Peg_classifier_publisher(ros::NodeHandle& node,const std::string& topic_name,std::size_t length);

   // void publish(const arma::fvec)

   // void publish(const arma::fcolvec2& features);

private:

    std_msgs::Float32MultiArray feature_msg;
    ros::Publisher              publisher;
  //  vector_type                 v_type;


};

}

#endif
