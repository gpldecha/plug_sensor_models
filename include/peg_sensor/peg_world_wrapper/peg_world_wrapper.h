#ifndef PEG_WORLD_WRAPPER_H_
#define PEG_WORLD_WRAPPER_H_

#include <world_wrapper/world_wrapper.h>
#include "node/publisher.h"

#include <objects/socket_one.h>
#include <optitrack_rviz/listener.h>

#include <objects/vis_socket.h>
#include <world_wrapper/world_wrapper.h>
#include <world_wrapper/visualisation/vis_wbox.h>

#include <optitrack_rviz/listener.h>
#include <optitrack_rviz/input.h>
#include <visualise/vis_points.h>
#include <visualise/vis_vector.h>
#include "peg_sensor/peg_sensor_model/peg_sensor_model.h"


/**
 *      === Peg World Wrapper ===
 *
 *      Wrappes the world items with boxes such to be able to compute distances
 *      from the peg tips to them. These distances and directions are then used
 *      in a sensor model Y = h(X)
 *
 */

class Peg_world_wrapper{

public:

    Peg_world_wrapper(ros::NodeHandle& nh,
                      const std::string &path_sensor_model,
                      const std::string& fixed_frame,
                      const std::string table_link_name      = "link_wall",
                      const std::string socket_link_name     = "link_socket",
                      const std::string socket_link_box_name = "link_wall");

    void set_table_socket_origin(const arma::fcolvec3& origin,const arma::fcolvec3& rpy);

    void transform_table_socket(const arma::fcolvec3& origin);

    ww::World_wrapper& get_world_wrapper();

    wobj::WrapObject& get_wrapped_objects();


    ///
    /// \brief update : updates (position) and publishes (rviz) all information
    ///                 related to the peg socket world
    ///
    void update();

private:

    void initialise_table_wall(const std::string table_link_name="link_wall");

    void initialise_socket(const std::string &socket_link_name, const std::string &wall_link_name);

    void initialise_urdf(const std::string& table_urdfs,const std::string& fixed_frame);


    void initialise_objects();


private:

         ww::World_wrapper  world_wrapper;

         std::string        table_link_name;
         std::string        fixed_frame;
         std::string        socket_link_name;
         std::string        socket_link_box_name;


         obj::Socket_one    socket_one;

         std::shared_ptr<ww::Publisher>             world_publisher;
         std::shared_ptr<obj::Vis_socket>           vis_socket;
         std::shared_ptr<Peg_sensor_model>          peg_sensor_model;
         std::shared_ptr<opti_rviz::Vis_points>     vis_points;
         std::shared_ptr<opti_rviz::Vis_vectors>    vis_vectors;



};


#endif
