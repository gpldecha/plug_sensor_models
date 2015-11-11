#include "peg_sensor/peg_sensor_model/distance_model.h"
#include <optitrack_rviz/type_conversion.h>

namespace psm {

Contact_distance_model::Contact_distance_model(wobj::WrapObject& wrap_object,obj::Socket_one& socket_one)
    :Peg_distance_model(wrap_object),distance_features(wrap_object),socket_one(socket_one)
{

    dir_vectors.resize(2);
    colors.resize(2);
    colors[0]   = tf::Vector3(1,0,0);
    colors[1]   = tf::Vector3(0,0,1);

    socket_box = wrap_object.wboxes[1];
    proj_points.resize(2,3);

    double sd = 0.02;
    min_half_one_div_var = -0.5 * 1 / (sd * sd);

}

void Contact_distance_model::update(arma::colvec &Y,const arma::colvec3& pos,const arma::mat33& Rot){
  //  std::cout<< " start Contact_distance_model::update" << std::endl;
    Y.resize(3);

    Peg_distance_model::update_model(pos,Rot);
    get_distances();

    Y(C_SURF)   = min_distance_surface;
    Y(C_EDGE)   = min_distance_edge;
    Y(C_SOCKET) = isInSocket; //is_inside_socket_box(pos);

}

void Contact_distance_model::update(arma::mat& hY,const arma::mat& points,const arma::mat33& Rot){
    assert(hY.n_rows == points.n_rows);
    Yone.resize(3);
    for(std::size_t i = 0; i < points.n_rows;i++){

        Peg_distance_model::update_model(points.row(i).st(),Rot);
        get_distances();


        Yone(C_SURF)   = min_distance_surface;
        Yone(C_EDGE)   = min_distance_edge;
        Yone(C_SOCKET) = isInSocket;

        if(isInTable){
            Yone(C_SURF) = 1;
        }

        hY.row(i) = Yone.st();
    }

}


void Contact_distance_model::get_distance_single_point(arma::fcolvec3 &x){
    distance_features.compute_surface_edge_vector(x);

    direction_surf = distance_features.point_surface - x;
    min_distance_surface = arma::norm(direction_surf);

    direction_edge = distance_features.point_edge - x;
    min_distance_edge = arma::norm(direction_edge);

}

void Contact_distance_model::get_distances(){
    min_distance_surface = std::numeric_limits<float>::max();
    min_distance_edge    = std::numeric_limits<float>::max();
    std::size_t index_closet_point_s = -1;
    std::size_t index_closet_point_e = -1;

    isInSocket = true;



    for(std::size_t i = 0; i < 3;i++)
    {
        distance_features.compute_surface_edge_vector(model_points.row(i).st());


        direction_surf = distance_features.point_surface - model_points.row(i).st();
        current_distance_surface = arma::norm(direction_surf);

        direction_edge = distance_features.point_edge - model_points.row(i).st();
        current_distance_edge = arma::norm(direction_edge);

        if(current_distance_surface < min_distance_surface){
            min_distance_surface  =current_distance_surface;
            index_closet_point_s  = i;
            proj_points.row(0) = distance_features.point_surface.st();
        }

        if(current_distance_edge < min_distance_edge){
            min_distance_edge      =   current_distance_edge;
            index_closet_point_e   =   i;
            proj_points.row(1) = distance_features.point_edge.st();

        }

        isInSocket = isInSocket && is_inside_socket_box(model_points.row(i).st());


    }

    isInTable =   distance_features.bIsInside;

    direction_surf = proj_points.row(0).st() - model_points.row(index_closet_point_s).st();
    direction_edge = proj_points.row(1).st() - model_points.row(index_closet_point_e).st();

    if(b_visualise){


        opti_rviz::type_conv::vec2tf(model_points.row(index_closet_point_s).st(),dir_vectors[C_SURF].origin);
        opti_rviz::type_conv::vec2tf(direction_surf,dir_vectors[C_SURF].direction);

        opti_rviz::type_conv::vec2tf(model_points.row(index_closet_point_e).st(),dir_vectors[C_EDGE].origin);
        opti_rviz::type_conv::vec2tf(direction_edge,dir_vectors[C_EDGE].direction);
    }
}

void Contact_distance_model::initialise_vision(ros::NodeHandle& node){
    Peg_distance_model::initialise_vision(node);
    ptr_vis_vectors = std::shared_ptr<opti_rviz::Vis_vectors>( new opti_rviz::Vis_vectors(node,"contact_model"));
    ptr_vis_vectors->scale = 0.002;
    ptr_vis_vectors->set_color(colors);
    ptr_vis_vectors->initialise("world",dir_vectors);


    ptr_proj_points = std::shared_ptr<opti_rviz::Vis_points>( new opti_rviz::Vis_points(node,"projected_points") );
    ptr_proj_points->r = 1;
    ptr_proj_points->scale = 0.005;
    ptr_proj_points->initialise("world",proj_points);

}

void Contact_distance_model::visualise(){
    if(b_visualise){
        Peg_distance_model::visualise();
        if(ptr_vis_vectors != NULL){
            ptr_vis_vectors->update(dir_vectors);
            ptr_vis_vectors->publish();

            ptr_proj_points->update(proj_points);
            ptr_proj_points->publish();
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------





}
