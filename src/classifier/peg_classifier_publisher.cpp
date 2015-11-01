#include "peg_sensor/classifier/peg_classifier_publisher.h"

namespace psm{

Peg_classifier_publisher::Peg_classifier_publisher(ros::NodeHandle& node, const std::string& topic_name, std::size_t length)
{
    publisher = node.advertise<std_msgs::Float32MultiArray>(topic_name,10);
    feature_msg.data.resize(length);

}

/*
void Peg_classifier_publisher::publish(const arma::fcolvec2 &features){
    feature_msg.data[0] = features(0);
    feature_msg.data[1] = features(1);
    publisher.publish(feature_msg);
}

void Peg_classifier_publisher::publish(const arma::fcolvec6 &features){
    feature_msg.data[0] = features(contact);
    feature_msg.data[1] = features(up);
    feature_msg.data[2] = features(down);
    feature_msg.data[3] = features(left);
    feature_msg.data[4] = features(right);
    feature_msg.data[5] = features(push);
    publisher.publish(feature_msg);
}*/

}
