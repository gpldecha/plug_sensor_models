#include "peg_sensor_manager/sensor_manager.h"


namespace psm{


Sensor_manager::Sensor_manager(ros::NodeHandle& nh, wobj::WrapObject& wrapped_objects, obj::Socket_one &socket_one):
wrapped_objects(wrapped_objects),socket_one(socket_one)
{
    t_sensor       = psm::NONE;
    service_server = nh.advertiseService("sensor_manager_cmd",&Sensor_manager::sensor_manager_callback,this);
    initialise();
}

void Sensor_manager::initialise(){
        sptr_cdist           = Sptr_cdist( new psm::Contact_distance_model(wrapped_objects,socket_one) );


        //sptr_three_dist      = Sptr_three_dist( new psm::Three_pin_distance_model(wrapped_objects));
       // ptr_sensor_force_idd = Sptr_fii(new psm::Force_iid_model(SIMPLE));
}

/*
void Sensor_manager::update(arma::colvec& Y,
                            const arma::colvec3& pos,
                            const arma::mat33& Rot,
                            const arma::fcolvec3& force,
                            const arma::fcolvec3& torque)
{
  //  switch(t_sensor){

  //  case SIMPLE_CONTACT_DIST:
  //  {
        sptr_cdist->update(Y,pos,Rot);
   /*     break;
    }
    case THREE_PIN_DIST:
    {
        sptr_three_dist->update(Y,pos,Rot);
        break;
    }
    case FORCE_IID:
    {
    //   ptr_sensor_force_idd->update(Y,force);
        break;
    }
    case NONE:
    {
        break;
    }
    default:
    {
        break;
    }
    }*/
//}


void Sensor_manager::update_peg(arma::colvec& Y,const arma::colvec3& pos, const arma::mat33& Rot){
    sptr_cdist->update(Y,pos,Rot);
}


void Sensor_manager::update_particles(arma::mat& Y,const arma::mat& points, const arma::mat33& Rot){
    sptr_cdist->update(Y,points,Rot);
}


bool Sensor_manager::sensor_manager_callback(peg_sensor::String_cmd::Request& req, peg_sensor::String_cmd::Response& res){


    std::string cmd = req.req;

    if(cmd == "simple"){
         t_sensor = SIMPLE_CONTACT_DIST;
         res.res  = "sensor type: simple_contact_dist set!";
         return true;
    }else if(cmd == "three"){
         t_sensor = THREE_PIN_DIST;
         res.res  = "sensor type: three_pin_dist set!";
         return true;
    }else if(cmd == "force_iid"){
         t_sensor = FORCE_IID;
         res.res  = "sensor type: force_iid set!";
         return true;
    }else{
        res.res = "no such cmd: " + cmd + " !";
        return  false;
    }


}


/*
void Sensor_manager::init_visualise(ros::NodeHandle& node){
    if(sptr_dist != NULL){
        std::cout<< "(visual marker initalised for sensor manager)" << std::endl;
        sptr_dist->initialise_vision(node);
    }
}
*/
/*
void Sensor_manager::visualise(){

  //  std::cout<< "Sensor_manager:visualise" << std::endl;
  //  std::cout<< "t_sensor: " << t_sensor << std::endl;

    switch(t_sensor){
    case FORCE_IID:
    {
        break;
    }
    default:
    {
               // std::cout<< "Sensor_manager::visualise(): THREE_PIN_DIST" << std::endl;
        sptr_dist->visualise();
        break;
    }
    }

}
*/

}
