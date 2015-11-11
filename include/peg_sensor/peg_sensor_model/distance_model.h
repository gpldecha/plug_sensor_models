#ifndef DISTANCE_MODEL_H_
#define DISTANCE_MODEL_H_

#include "peg_sensor/peg_sensor_model/peg_distance_model.h"
#include "wrapobject.h"
#include <visualise/vis_vector.h>
#include <objects/socket_one.h>

namespace psm{

class Contact_distance_model : public Peg_distance_model{

public:

    typedef enum {C_SURF=0,C_EDGE=1,C_SOCKET=2} contact_types;

public:

    Contact_distance_model(wobj::WrapObject& wrap_object,obj::Socket_one& socket_one);

    virtual void update(arma::colvec &Y,const arma::colvec3& pos,const arma::mat33& Rot);

    virtual void update(arma::mat& hY, const arma::mat& points, const arma::mat33& Rot);

    virtual void initialise_vision(ros::NodeHandle& node);

    virtual void visualise();

protected:

    void get_distances();

    void get_distance_single_point(arma::fcolvec3 &x);

    inline bool is_inside_socket_box(const arma::fcolvec3 &pos){
        return  socket_one.hole_wboxes[0].is_inside(pos) || socket_one.hole_wboxes[1].is_inside(pos) || socket_one.hole_wboxes[2].is_inside(pos);
    }



private:

    double                              min_half_one_div_var;

protected:

    Distance_features                  distance_features;

    std::vector<opti_rviz::Arrow>           dir_vectors;
    std::vector<tf::Vector3>                colors;
    std::shared_ptr<opti_rviz::Vis_vectors> ptr_vis_vectors;
    std::shared_ptr<opti_rviz::Vis_points>  ptr_proj_points;

    arma::fmat                              proj_points;
    wobj::WBox*                             socket_box;
    obj::Socket_one&                        socket_one;

    arma::fcolvec3                      tmp;
    arma::colvec                        Yone;

    float                               min_distance_edge;
    float                               min_distance_surface;
    float                               current_distance_surface;
    float                               current_distance_edge;
    bool                                isInSocket;
    bool                                isInTable;


    std::size_t                         index_closest_model_surf;
    std::size_t                         index_closest_model_edge;

    arma::fcolvec3                      direction_surf;
    arma::fcolvec3                      direction_edge;

    arma::fcolvec3                      closet_point_proj_surf;
    arma::fcolvec3                      closest_model_surf;

    arma::fcolvec3                      closet_point_proj_edge;
    arma::fcolvec3                      closet_model_edge;


};


/**
 *  === The Insertion_sensor class ===
 *
 *  Given a Socket model, peg model and the origin and orientation of the peg
 *  computes a probability the peg being connected or not to the socket
 *
 *
* */

/*
class Insertion_sensor{

public:

    Insertion_sensor(wobj::WrapObject& wrap_object);

    void update(arma::colvec &Y,const arma::colvec3& pos,const arma::mat33& Rot);

     void update(arma::mat& hY, const arma::mat& points, const arma::mat33& Rot);

private:



private:

    wobj::WrapObject& wrap_object;
    wobj::WBox* socket_box;
    geo::fCVec3 tmp;
};
*/

}

#endif
