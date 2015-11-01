#include "peg_sensor/listener.h"

namespace psm{

Peg_sensor_listener::Peg_sensor_listener(ros::NodeHandle &node, const std::string &topic_name,std::size_t data_size):
    data_size(data_size)
{

    subscriber = node.subscribe(topic_name,100, &Peg_sensor_listener::callback,this);
    data.resize(data_size);

}


void Peg_sensor_listener::callback(const std_msgs::Float32MultiArrayConstPtr& msg){
    for(std::size_t i = 0; i < data_size;i++){
        data(i) = msg->data[i];
    }
}



}
