#include "peg_sensor/peg_sensor_model/peg_sensor_model.h"
#include <armadillo>
#include <limits>


Peg_sensor_model::Peg_sensor_model(const std::string& path_to_peg_model,
                                   const std::string& fixed_frame,
                                   wobj::WrapObject& wrap_object)
    :wrapped_world(wrap_object),tf_listener(fixed_frame,"peg_link")
{
    arma::mat points;
    if(!points.load(path_to_peg_model)){
        std::cerr<< "Peg_sensor_vis::Peg_sensor_vis failed to load file: " + path_to_peg_model << std::endl;
    }else{

        model.resize(points.n_rows);
        contact_info.resize(2);
        arrows.resize(2);
        closest_points.resize(2);

        model_TF.resize(points.n_rows);
        for(std::size_t r = 0; r < points.n_rows;r++){
            model_TF[r].setValue(points(r,0),points(r,1),points(r,2));
        }

    }

    contact_info[SURFACE] = Contact_points(SURFACE);
    contact_info[EDGE]    = Contact_points(EDGE);

}


void Peg_sensor_model::update(){
    tf_listener.update(position,orientation);
    update_model(position,orientation);

    get_distance_features();
}

void Peg_sensor_model::get_distance_features(){

    min_distance_edge       = std::numeric_limits<float>::max();
    min_distance_surface    = std::numeric_limits<float>::max();

  //  std::cout<< "Peg_sensor_model::get_distance_features()" << std::endl;


    for(std::size_t i = 0; i <  model.size();i++)
    {

        tmp_vec3f(0) = model[i][0];
        tmp_vec3f(1) = model[i][1];
        tmp_vec3f(2) = model[i][2];

        wrapped_world.distance_to_features(tmp_vec3f);

        current_distance_surface  = wrapped_world.get_distance_to_surface();
        current_distance_edge     = wrapped_world.get_distance_to_edge();

        if(current_distance_surface < min_distance_surface){

            contact_info[SURFACE].index            = i;
            contact_info[SURFACE].closest_point    = wrapped_world.get_closest_point_surface();
            contact_info[SURFACE].distance         = current_distance_surface;

            min_distance_surface = current_distance_surface;
        }

        if(current_distance_edge < min_distance_edge){

            contact_info[EDGE].index                 = i;
            contact_info[EDGE].closest_point         = wrapped_world.get_closest_point_edge();
            contact_info[EDGE].distance              = current_distance_edge;

            min_distance_edge = current_distance_edge;
        }
    }


  //  contact_info[SURFACE].print();
  //  contact_info[EDGE].print();

/*

    tmp_vec3f(0) = model[0][0];
    tmp_vec3f(1) = model[0][1];
    tmp_vec3f(2) = model[0][2];

    wrapped_world.distance_to_features(tmp_vec3f);

    current_distance_surface  = wrapped_world.get_distance_to_surface();
    current_distance_edge     = wrapped_world.get_distance_to_edge();


    contact_info[SURFACE].index            = 0;
    contact_info[SURFACE].closest_point    = wrapped_world.get_closest_point_surface();
    contact_info[SURFACE].distance         = current_distance_surface;

    contact_info[EDGE].index                 = 0;
    contact_info[EDGE].closest_point         = wrapped_world.get_closest_point_edge();
    contact_info[EDGE].distance              = current_distance_edge;

*/

    // SURFACE
    arrows[SURFACE].origin      = model[contact_info[SURFACE].index];

    tmp_Vec3[0]                 = contact_info[SURFACE].closest_point(0);
    tmp_Vec3[1]                 = contact_info[SURFACE].closest_point(1);
    tmp_Vec3[2]                 = contact_info[SURFACE].closest_point(2);
    closest_points[SURFACE]     = tmp_Vec3;

    arrows[SURFACE].direction   = closest_points[SURFACE] - arrows[SURFACE].origin;

    // EDGE
    arrows[EDGE].origin         = model[contact_info[EDGE].index];

    tmp_Vec3[0]                 = contact_info[EDGE].closest_point(0);
    tmp_Vec3[1]                 = contact_info[EDGE].closest_point(1);
    tmp_Vec3[2]                 = contact_info[EDGE].closest_point(2);
    closest_points[EDGE]        = tmp_Vec3;

    arrows[EDGE].direction   = closest_points[EDGE] - arrows[EDGE].origin;

}


void Peg_sensor_model::update_model(const tf::Vector3& T,const tf::Matrix3x3& R){
    for(std::size_t i = 0; i < model.size();i++){
        model[i]  = R * model_TF[i] + T;
    }
}


const std::vector<tf::Vector3> &Peg_sensor_model::get_model(){
    return model;
}

const std::vector<tf::Vector3> &Peg_sensor_model::get_closet_point(){
    return closest_points;
}

const std::vector<opti_rviz::Arrow>& Peg_sensor_model::get_arrows(){
    return arrows;
}
