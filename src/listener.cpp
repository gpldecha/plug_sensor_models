#include "peg_sensor/listener.h"

namespace psm{

Peg_sensor_listener::Peg_sensor_listener(ros::NodeHandle &node, const std::string &topic_name)
    :size(size)
{

    subscriber = node.subscribe(topic_name,10, &Peg_sensor_listener::callback,this);
    //Y.resize(size);
}


void Peg_sensor_listener::callback(const std_msgs::Float32MultiArrayConstPtr& msg){
 //   assert(size == msg->data.size());
    if(msg->data.size() != Y.n_elem){
        Y.resize(msg->data.size());
    }

    for(std::size_t i = 0; i < msg->data.size();i++){
        Y(i) = msg->data[i];
    }
}



}
