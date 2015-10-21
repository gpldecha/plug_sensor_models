#include <plug_sensor_models/plug_distance_model.h>

namespace psm{

Plug_distance_model::Plug_distance_model(wobj::WrapObject &wrap_object):
    wrap_object(wrap_object)
{
    std::string  config_file = "/home/guillaume/roscode/catkin_ws/src/models_project/objects/meshes/plug/config/X.txt";
    if(!model_TF.load(config_file)){
        std::cerr<< "Plug_sensor::Plug_sensor failed to load file: " + config_file << std::endl;
    }
    num_model_points = model_TF.n_rows;
    model_points.resize(num_model_points,3);
    model_TF.print("model_TF");

    b_visualise = false;
}

void Plug_distance_model::update_model(const arma::colvec3& T, const arma::mat33& R){

   // R.print("update_model R");

    for(std::size_t i = 0; i < num_model_points;i++)
    {
        plug_point  = R * model_TF.row(i).st() + T;
        model_points(i,0) = plug_point(0);
        model_points(i,1) = plug_point(1);
        model_points(i,2) = plug_point(2);
    }
}


void Plug_distance_model::print(const arma::colvec& Y) const{
    Y.print("Y");
}

void Plug_distance_model::initialise_vision(ros::NodeHandle& node){
    ptr_vis_points = std::shared_ptr<opti_rviz::Vis_points>(new opti_rviz::Vis_points(node,"plug_model"));
    ptr_vis_points->scale = 0.005;
    ptr_vis_points->r = 1;
    ptr_vis_points->g = 1;
    ptr_vis_points->initialise("world",model_points);
    b_visualise       = true;
}

void Plug_distance_model::visualise(){
    if(b_visualise){
        if(ptr_vis_points != NULL){
            ptr_vis_points->update(model_points);
            ptr_vis_points->publish();
        }
    }
}



///
/// \brief Plug_contact_model::Plug_contact_model
/// \param wrap_object
///
Plug_contact_model::Plug_contact_model(wobj::WrapObject &wrap_object):
wrap_object(wrap_object){
    std::string  config_file = "/home/guillaume/roscode/catkin_ws/src/models_project/objects/meshes/plug/config/X.txt";
    if(!model_TF.load(config_file)){
        std::cerr<< "Plug_sensor::Plug_sensor failed to load file: " + config_file << std::endl;
    }
    num_model_points = model_TF.n_rows;
    model_points.resize(num_model_points,3);
}

void Plug_contact_model::update(arma::colvec3& pos,const arma::mat33& Rot){
    for(std::size_t i = 0; i < num_model_points;i++)
    {
        plug_point  = Rot * model_TF.row(i).st() + pos;
        model_points(i,0) = plug_point(0);
        model_points(i,1) = plug_point(1);
        model_points(i,2) = plug_point(2);
    }


    wobj::ID_object id_object;

    for(std::size_t i = 0; i < model_points.n_rows;i++)
    {
        wrap_object.get_closest_surface(model_points.row(i).st(),id_object);
        if(wrap_object.is_inside_surface(model_points.row(i).st(),id_object.index_obj,id_object.index_fea)){
            point_surface  = wrap_object.get_surface_projection(model_points.row(i).st(),id_object);
            if(arma::norm(point_surface) > 0.04){
                correction = point_surface - model_points.row(i).st();
                pos         = pos + correction;
            }
            i = model_points.n_rows;
        }
    }

}


}
