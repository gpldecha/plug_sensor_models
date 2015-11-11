#include "peg_sensor/peg_world_wrapper/peg_world_wrapper.h"


Peg_world_wrapper::Peg_world_wrapper(ros::NodeHandle &nh,
                                     const std::string& node_name,
                                     const std::string& path_sensor_model,
                                     const std::string& fixed_frame,
                                     const std::string table_link_name,
                                     const std::string socket_link_name,
                                     const std::string socket_link_box_name)
    :fixed_frame(fixed_frame),
     table_link_name(table_link_name),
     socket_link_name(socket_link_name),
     socket_link_box_name(socket_link_box_name)
{


    // Initialise the table wall
    initialise_table_wall(table_link_name);
    initialise_socket(socket_link_name,socket_link_box_name);


    world_publisher = std::shared_ptr<ww::Publisher>(new ww::Publisher( node_name + "/visualization_marker",&nh,&world_wrapper));
    world_publisher->init(fixed_frame);
    world_publisher->update_position();

   /// Visualise socket

    vis_socket = std::shared_ptr<obj::Vis_socket>(new  obj::Vis_socket(nh,world_wrapper.wrapped_objects.wsocket));
    vis_socket->initialise(25,0.01);


    /// Peg model (Cartesian points);

     peg_sensor_model = std::shared_ptr<Peg_sensor_model>(new Peg_sensor_model(path_sensor_model,fixed_frame,world_wrapper.wrapped_objects));


    vis_points = std::shared_ptr<opti_rviz::Vis_points>( new  opti_rviz::Vis_points(nh,"peg_model"));
    vis_points->scale = 0.005;
    vis_points->initialise(fixed_frame,peg_sensor_model->get_model());

    vis_vectors = std::shared_ptr<opti_rviz::Vis_vectors>(new opti_rviz::Vis_vectors(nh,"closest_features") );
    vis_vectors->scale = 0.005;
    std::vector<tf::Vector3> colors(2);
    colors[0] = tf::Vector3(1,0,0);
    colors[1] = tf::Vector3(0,0,1);
    vis_vectors->set_color(colors);
    vis_vectors->initialise(fixed_frame,peg_sensor_model->get_arrows());

}

/*
void Peg_world_wrapper::set_table_socket_origin(const arma::fcolvec3& origin, const arma::fcolvec3 &rpy){

    world_wrapper.set_origin_orientation(table_link_name,origin,rpy);

}*/
/*
void Peg_world_wrapper::transform_table_socket(const arma::fcolvec3& origin){
   wobj::WBox& table_wall = world_wrapper.get_wbox(0);
   table_wall.transform(origin);

   wobj::WBox& socket_box = world_wrapper.get_wbox(1);
   socket_box.transform(origin);
}
*/
void Peg_world_wrapper::initialise_table_wall(const std::string table_link_name){

    tf::StampedTransform transform;
    opti_rviz::Listener::get_tf_once(fixed_frame,table_link_name,transform);

    tf::Vector3  wall_origin = transform.getOrigin();

    geo::fCVec3 origin       = {{(float)wall_origin.x(),(float)wall_origin.y(),(float)wall_origin.z()}};//{{0,0,-0.02/2}};
    float      dx            = 0.1;
    origin(0)                = origin(0)-dx/2;
    geo::fCVec3 dim          = {{static_cast<float>(0.02+dx),0.8,0.4}};
    geo::fCVec3 orientation  = {{0,0,0}};

    wbox = wobj::WBox(table_link_name,dim,origin,orientation);
    world_wrapper.wrapped_objects.push_back_box(&wbox);
}

void Peg_world_wrapper::initialise_socket(const std::string& socket_link_name,const std::string& wall_link_name){

    tf::StampedTransform transform;
    opti_rviz::Listener::get_tf_once(fixed_frame,socket_link_name,transform);
    opti_rviz::Listener::print(transform);

    /// add a socket
    tf::Vector3 origin = transform.getOrigin();
    tf::Vector3 rpy(M_PI/2,0,M_PI/2);
    socket_one = obj::Socket_one(socket_link_name,wall_link_name,origin,rpy,1);

    world_wrapper.wrapped_objects.push_back_box(&(socket_one.wbox));
    world_wrapper.wrapped_objects.push_back_socket(socket_one.wsocket);

    world_wrapper.wrapped_objects.push_back_box(&(socket_one.hole_wboxes[0]));
    world_wrapper.wrapped_objects.push_back_box(&(socket_one.hole_wboxes[1]));
    world_wrapper.wrapped_objects.push_back_box(&(socket_one.hole_wboxes[2]));

}


void Peg_world_wrapper::update(){

    peg_sensor_model->update();

    vis_points->update(peg_sensor_model->get_model());
    vis_vectors->update(peg_sensor_model->get_arrows());

    vis_points->publish();
    vis_vectors->publish();

    vis_socket->publish();


    world_publisher->update_position();
    world_publisher->publish();

}

ww::World_wrapper& Peg_world_wrapper::get_world_wrapper(){
    return world_wrapper;
}

wobj::WrapObject& Peg_world_wrapper::get_wrapped_objects(){
    return world_wrapper.wrapped_objects;
}




void Peg_world_wrapper::initialise_urdf(const std::string& table_urdfs, const std::string &fixed_frame){
/*
    world_wrapper.loadURDF(table_urdfs);
    world_wrapper.initialise_origin_orientation(world_wrapper,fixed_frame);
    geo::fCVec3 T = {{0,0,-0.02}};
    for(std::size_t i = 0; i < world_wrapper.wrapped_objects.wboxes.size();i++){
        world_wrapper.wrapped_objects.wboxes[i].transform(T);
    }*/

}


void Peg_world_wrapper::initialise_objects(){

/*
    tf::StampedTransform transform;
    opti_rviz::Listener::get_tf_once("world_frame","link_wall",transform);
    opti_rviz::Listener::print(transform);



    tf::Vector3     wall_origin = transform.getOrigin();

    geo::fCVec3 origin_      = {{(float)wall_origin.x(),(float)wall_origin.y(),(float)wall_origin.z()}};//{{0,0,-0.02/2}};
  //  geo::fCVec3 dim_         = {{0.02,0.8,0.4}};
  //  geo::fCVec3 orientation_ = {{0,0,0}};

   float      dx            = 0.1;
    origin_(0)               = origin_(0)-dx/2;
    geo::fCVec3 dim_         = {{static_cast<float>(0.02+dx),0.8,0.4}};
    geo::fCVec3 orientation_ = {{0,0,0}};


    wobj::WBox  wsocket_wall("link_wall",dim_,origin_,orientation_);

    opti_rviz::Listener::get_tf_once("world_frame","link_socket",transform);
    opti_rviz::Listener::print(transform);

    /// add a socket
    tf::Vector3 origin = transform.getOrigin();
    //tf::Vector3 rpy(0,0,0);
    tf::Vector3 rpy(M_PI/2,0,M_PI/2);
    obj::Socket_one socket_one("link_socket","link_wall",origin,rpy,1);
/*


    geo::fCVec3 origin_      = {{0,0,-0.02/2}};
    geo::fCVec3 dim_         = {{0.8,0.4,0.02}};
    geo::fCVec3 orientation_ = {{M_PI/2,0,M_PI/2}};

    wobj::WBox wsocket_wall("socket_wall",dim_,origin_,orientation_);

    /// add a socket
    tf::Vector3 origin(0,0,0);
    tf::Vector3 rpy(M_PI/2,0,0);

    obj::Socket_one socket_one("link_socket","link_wall",origin,rpy,1);*/
/*
    world_wrapper.wrapped_objects.push_back_box(wsocket_wall);
    world_wrapper.wrapped_objects.push_back_box(socket_one.wbox);
    world_wrapper.wrapped_objects.push_back_socket(socket_one.wsocket);

*/

}

