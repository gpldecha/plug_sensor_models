#include "peg_sensor/peg_sensor_model/peg_distance_model.h"

namespace psm{

Peg_distance_model::Peg_distance_model(wobj::WrapObject &wrap_object):
    wrap_object(wrap_object)
{
    std::string  config_file = "/home/guillaume/roscode/catkin_ws/src/wrapper/models_project/objects/meshes/plug/config/X.txt";
    if(!model_TF.load(config_file)){
        std::cerr<< "Plug_sensor::Plug_sensor failed to load file: " + config_file << std::endl;
    }
    num_model_points = model_TF.n_rows;
    model_points.resize(num_model_points,3);
    model_TF.print("model_TF");

    b_visualise = false;
}

void Peg_distance_model::update_model(const arma::colvec3& T, const arma::mat33& R){

    for(std::size_t i = 0; i < num_model_points;i++)
    {
        plug_point  = R * model_TF.row(i).st() + T;
        model_points(i,0) = plug_point(0);
        model_points(i,1) = plug_point(1);
        model_points(i,2) = plug_point(2);
    }
}


void Peg_distance_model::print(const arma::colvec& Y) const{
    Y.print("Y");
}

void Peg_distance_model::initialise_vision(ros::NodeHandle& node){
    ptr_vis_points = std::shared_ptr<opti_rviz::Vis_points>(new opti_rviz::Vis_points(node,"plug_model"));
    ptr_vis_points->scale = 0.005;
    ptr_vis_points->r = 1;
    ptr_vis_points->g = 1;
    ptr_vis_points->initialise("world",model_points);
    b_visualise       = true;
}

void Peg_distance_model::visualise(){
    if(b_visualise){
        if(ptr_vis_points != NULL){
            ptr_vis_points->update(model_points);
            ptr_vis_points->publish();
        }
    }
}



}
