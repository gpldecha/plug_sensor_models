#include "peg_sensor/peg_sensor_model/distance_features.h"

#include <optitrack_rviz/type_conversion.h>

namespace psm{


Distance_features::Distance_features(wobj::WrapObject &wrapped_world):
wrapped_world(wrapped_world)
{

}


void Distance_features::compute_surface_edge_vector(const arma::fcolvec3 &P){
    wrapped_world.distance_to_features(P);
    point_edge    = wrapped_world.get_closest_point_edge();
    point_surface = wrapped_world.get_closest_point_surface();
    bIsInside     = wrapped_world.is_inside_box();
}

/*
void Distance_features::get_distance_features(const arma::colvec3 &model_center){

    min_distance_edge       = std::numeric_limits<float>::max();

    wobj::ID_object id_surf_obj;
    wobj::ID_object id_edge_obj;

    tmp(0) = model_center(0);
    tmp(1) = model_center(1);
    tmp(2) = model_center(2);

    // get closet surface from origin and check it is not inside an object
    wrapped_world.get_closest_surface(tmp,id_surf_obj);
        // get closest edge from origin
        wrapped_world.get_closest_edge(tmp,id_edge_obj);
        get_distances_model(id_surf_obj,id_edge_obj);
}*/
/*
void Distance_features::get_distances_model(const wobj::ID_object &id_surface, const wobj::ID_object &id_edge){

    min_distance_surface    = std::numeric_limits<float>::max();
    min_distance_edge       = std::numeric_limits<float>::max();

    for(std::size_t i = 0; i < model_points.n_rows;i++)
    {

        current_distance_surface = wrapped_world.distance_to_surface(model_points.row(i).st(),id_surface);
        current_distance_edge    = wrapped_world.distance_to_edge(model_points.row(i).st(),id_edge);

        if(current_distance_surface < min_distance_surface){
            min_distance_surface        = current_distance_surface;
            index_closest_model_surf    = i;

        }
        if(current_distance_edge < min_distance_edge){
            min_distance_edge           = current_distance_edge;
            index_closest_model_edge    = i;
        }
    }*/

    /*if(min_distance_surface != -1){
        // Get closest projected surface and edge points

        // get closest surface_point
        closest_model_surf       = model_points.row(index_closest_model_surf).st();
        closet_point_proj_surf   = wrapped_world.get_surface_projection(closest_model_surf,id_surface);
        direction_surf           = closet_point_proj_surf - closest_model_surf;

        // get closest edge point
        closet_model_edge       = model_points.row(index_closest_model_edge).st();
        closet_point_proj_edge  = wrapped_world.get_edge_projection(closet_model_edge,id_edge);
        direction_edge          = closet_point_proj_edge - closet_model_edge;

    }*/

/*
}
*/
}
