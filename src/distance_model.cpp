#include <plug_sensor_models/distance_model.h>
#include <optitrack_rviz/type_conversion.h>

namespace psm {


Three_pin_distance_model::Three_pin_distance_model(wobj::WrapObject &wrap_object):
    Plug_distance_model(wrap_object),distance_features(wrap_object){

    dir_vectors.resize(6);
    colors.resize(6);
    colors[0]   = tf::Vector3(1,0,0);
    colors[1]   = tf::Vector3(0,0,1);

    colors[2]   = tf::Vector3(1,0,0);
    colors[3]   = tf::Vector3(0,0,1);

    colors[4]   = tf::Vector3(1,0,0);
    colors[5]   = tf::Vector3(0,0,1);
}

void Three_pin_distance_model::update(arma::colvec &Y,const arma::colvec3& pos,const arma::mat33& Rot){


   update_model(pos,Rot);


   Y.resize(3 * 2 * 3);
   std::size_t index = 0;

    for(std::size_t i = 0; i < 3;i++)
    {
         distance_features.compute_surface_edge_vector(model_points.row(i).st());

         Y(6*i+0) = distance_features.point_surface(0) -  model_points(i,0);
         Y(6*i+1) = distance_features.point_surface(1) -  model_points(i,1);
         Y(6*i+2) = distance_features.point_surface(2) -  model_points(i,2);

         Y(6*i+3) = distance_features.point_edge(0) -  model_points(i,0);
         Y(6*i+4) = distance_features.point_edge(1) -  model_points(i,1 );
         Y(6*i+5) = distance_features.point_edge(2) -  model_points(i,2);

         if(b_visualise){

             dir_vectors[index].origin.setX(model_points(i,0));
             dir_vectors[index].origin.setY(model_points(i,1));
             dir_vectors[index].origin.setZ(model_points(i,2));

             dir_vectors[index].direction.setX(Y(6*i+0));
             dir_vectors[index].direction.setY(Y(6*i+1));
             dir_vectors[index].direction.setZ(Y(6*i+2));

             dir_vectors[index+1].origin.setX(model_points(i,0));
             dir_vectors[index+1].origin.setY(model_points(i,1));
             dir_vectors[index+1].origin.setZ(model_points(i,2));

             dir_vectors[index+1].direction.setX(Y(6*i+3));
             dir_vectors[index+1].direction.setY(Y(6*i+4));
             dir_vectors[index+1].direction.setZ(Y(6*i+5));
             index = index + 2;
         }
    }

}

void Three_pin_distance_model::initialise_vision(ros::NodeHandle& node){
    Plug_distance_model::initialise_vision(node);
    ptr_vis_vectors = std::shared_ptr<opti_rviz::Vis_vectors>( new opti_rviz::Vis_vectors(node,"three_pin_model"));
    ptr_vis_vectors->scale = 0.002;
    ptr_vis_vectors->set_color(colors);
    ptr_vis_vectors->initialise("world",dir_vectors);

}

void Three_pin_distance_model::visualise(){
    if(b_visualise){
        Plug_distance_model::visualise();
        if(ptr_vis_vectors != NULL){
            ptr_vis_vectors->update(dir_vectors);
            ptr_vis_vectors->publish();
        }
    }
}



Contact_distance_model::Contact_distance_model(wobj::WrapObject& wrap_object)
  :Plug_distance_model(wrap_object),distance_features(wrap_object)
{

    dir_vectors.resize(2);
    colors.resize(2);
    colors[0]   = tf::Vector3(1,0,0);
    colors[1]   = tf::Vector3(0,0,1);

    proj_points.resize(2,3);

    double sd = 0.02;
    min_half_one_div_var = -0.5 * 1 / (sd * sd);

}

void Contact_distance_model::update(arma::colvec &Y,const arma::colvec3& pos,const arma::mat33& Rot){
    Plug_distance_model::update_model(pos,Rot);
    Y.resize(2);

    get_distances();





   // if(distance_features.bIsInside){
   //        Y(C_SURF) = -1;
   //        Y(C_EDGE) = -1;
   // }else{
       // Y(C_SURF) = exp(min_half_one_div_var *  min_distance_surface);
       // Y(C_EDGE) = exp(min_half_one_div_var *  min_distance_edge);

        if(min_distance_surface < 0.02){
            Y(C_SURF) = min_distance_surface;
        }else{
            Y(C_SURF) = 0;
        }
        if(min_distance_edge < 0.02){
            Y(C_EDGE) = min_distance_edge;
        }else{
            Y(C_EDGE) = 0;
        }
 // }

    // this is a particle
   /* if(!b_visualise){
        if(pos(1) < -0.5){
            Y.print("Y particle");

        }
    }*/




   /* Y(C_SURF) =  exp(min_half_one_div_var * min_distance_surface );
    Y(C_EDGE) =  exp(min_half_one_div_var * min_distance_edge    );

    if(Y(C_SURF) < 0.0001){
        Y(C_SURF) = 0;
    }

    if(Y(C_EDGE) < 0.0001){
        Y(C_EDGE) = 0;
    }*/

}

void Contact_distance_model::get_distances(){
    min_distance_surface = std::numeric_limits<float>::max();
    min_distance_edge    = std::numeric_limits<float>::max();
    std::size_t index_closet_point_s = -1;
    std::size_t index_closet_point_e = -1;


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
    }
    direction_surf = proj_points.row(0).st() - model_points.row(index_closet_point_s).st();
    direction_edge = proj_points.row(1).st() - model_points.row(index_closet_point_e).st();





    if(b_visualise){


        opti_rviz::type_conv::vec2tf(model_points.row(index_closet_point_s).st(),dir_vectors[C_SURF].origin);
        opti_rviz::type_conv::vec2tf(direction_surf,dir_vectors[C_SURF].direction);

        opti_rviz::type_conv::vec2tf(model_points.row(index_closet_point_e).st(),dir_vectors[C_EDGE].origin);
        opti_rviz::type_conv::vec2tf(direction_edge,dir_vectors[C_EDGE].direction);
    }
}

/*
void Contact_distance_model::get_distance_features(const arma::colvec3& model_center){
    min_distance_edge       = std::numeric_limits<float>::max();

    wobj::ID_object id_surf_obj;
    wobj::ID_object id_edge_obj;

    tmp(0) = model_center(0);
    tmp(1) = model_center(1);
    tmp(2) = model_center(2);*/

    // get closet surface from origin and check it is not inside an object
    /*wrap_object.get_closest_surface(tmp,id_surf_obj);
    if(wrap_object.is_inside_surface(tmp,id_surf_obj.index_obj,id_surf_obj.index_fea)){
        min_distance_surface = -1;
     }else{
        wrap_object.get_closest_edge(tmp,id_edge_obj);
        get_distances_model(id_surf_obj,id_edge_obj);
    }*/
//}

/*
void Contact_distance_model::get_distances_model(const wobj::ID_object& id_surface, const wobj::ID_object& id_edge)
{
    min_distance_surface    = std::numeric_limits<float>::max();
    min_distance_edge       = std::numeric_limits<float>::max();*/

   /* for(std::size_t i = 0; i < num_model_points;i++)
    {*/

        //current_distance_surface = wrap_object.distance_to_surface(model_points.row(i).st(),id_surface);
        //current_distance_edge    = wrap_object.distance_to_edge(model_points.row(i).st(),id_edge);

        /*if(wrap_object.is_inside_surface(model_points.row(i).st(),id_surface.index_obj,id_surface.index_fea)){

            if(current_distance_surface > 0.004){
                min_distance_surface = -1;
            }else{
                if(current_distance_surface < min_distance_surface){
                    min_distance_surface        = current_distance_surface;
                    index_closest_model_surf    = i;
                }
                if(current_distance_edge < min_distance_edge){
                    min_distance_edge           = current_distance_edge;
                    index_closest_model_edge    = i;
                }
            }
        }else{
                if(current_distance_surface < min_distance_surface){
                    min_distance_surface        = current_distance_surface;
                    index_closest_model_surf    = i;

                }
                if(current_distance_edge < min_distance_edge){
                    min_distance_edge           = current_distance_edge;
                    index_closest_model_edge    = i;
                }
        }
    }*/

   /* if(min_distance_surface != -1){
        // Get closest projected surface and edge points

        // get closest surface_point
        closest_model_surf       = model_points.row(index_closest_model_surf).st();
        closet_point_proj_surf   = wrap_object.get_surface_projection(closest_model_surf,id_surface);
        direction_surf           = closet_point_proj_surf - closest_model_surf;

        // get closest edge point
        closet_model_edge       = model_points.row(index_closest_model_edge).st();
        closet_point_proj_edge  = wrap_object.get_edge_projection(closet_model_edge,id_edge);
        direction_edge          = closet_point_proj_edge - closet_model_edge;

        if(b_visualise){
            opti_rviz::type_conv::vec2tf(closest_model_surf,dir_vectors[C_SURF].origin);
            opti_rviz::type_conv::vec2tf(direction_surf,dir_vectors[C_SURF].direction);

            opti_rviz::type_conv::vec2tf(closet_model_edge,dir_vectors[C_EDGE].origin);
            opti_rviz::type_conv::vec2tf(direction_edge,dir_vectors[C_EDGE].direction);
        }
    }*/
//}

void Contact_distance_model::initialise_vision(ros::NodeHandle& node){
    Plug_distance_model::initialise_vision(node);
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
        Plug_distance_model::visualise();
        if(ptr_vis_vectors != NULL){
            ptr_vis_vectors->update(dir_vectors);
            ptr_vis_vectors->publish();

            ptr_proj_points->update(proj_points);
            ptr_proj_points->publish();
        }
    }
}



Four_contact_distance_model::Four_contact_distance_model(wobj::WrapObject &wrap_object)
    :Contact_distance_model(wrap_object)
{


    /*directions.resize(4);
    for(std::size_t i = 0; i < 4;i++){
        directions[i].zeros();
    }*/

    /*directions[0](1) =  -1;  // left
    directions[1](1) =   1;  // right
    directions[2](2) =   1;  // up
    directions[3](2) =  -1;  // down*/

    /*directions.resize(4);
    for(std::size_t i = 0; i < 4;i++){
        directions[i] = arma::normalise(directions[i]);
    }*/

    degree_var  = std::pow((M_PI/2)/3,2);
    dist_var    = std::pow(0.01/3,2);
    three_std   = 3 * sqrt(dist_var);
}

void Four_contact_distance_model::update(arma::colvec &Y, const arma::colvec3 &pos, const arma::mat33 &Rot){
    Plug_distance_model::update_model(pos,Rot);
    Contact_distance_model::get_distances();

    Y.resize(5);

    if(distance_features.bIsInside){
        if(min_distance_surface < 0.01){
            Y(C_SURF) = 1;//exp(min_distance_surface);
        }else{
            Y(C_SURF) = -1;
        }
    }else{

        if(min_distance_surface < 0.01){
            Y(C_SURF) = 1;//exp(min_distance_surface);//exp(-0.5*(1/(0.02*0.02))*min_distance_surface);
        }else{
            Y(C_SURF) = 0;
        }

        if(min_distance_edge < 0.01){
            Y(C_EDGE) = min_distance_edge;
        }else{
            Y(C_EDGE) = 0;
        }

        edge_dir_norm = arma::normalise(direction_edge);

        Y(2) = edge_dir_norm(0);
        Y(3) = edge_dir_norm(1);
        Y(4) = edge_dir_norm(2);
    }
}


//void Four_contact_distance_model::smooth_update(arma::colvec &Y){


       /*Y(0) = gaussian(min_distance_surface,0,dist_var);

       if(norm_direction > 0.01){
           for(std::size_t i = 0; i < directions.size();i++){
               Y(i+1) = 0;
           }
               Y(5) = 0;
       }else{
           for(std::size_t i = 0; i < directions.size();i++){
               Y(i+1) = gaussian(std::acos(arma::dot(edge_dir_norm,directions[i])),0,degree_var);
           }
           Y(5) = gaussian(norm_direction,0,dist_var);
       }

       for(std::size_t i = 0; i < Y.n_elem;i++){
           Y(i) = step_f(Y(i),0.05);
       }*/

//}



}
