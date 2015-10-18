#include <plug_sensor_manager/sensor_manager.h>


namespace psm{


Sensor_manager::Sensor_manager(wobj::WrapObject& wrapped_objects):
wrapped_objects(wrapped_objects)
{

    t_sensor = psm::NONE;
}

void Sensor_manager::initialise(type_sensor t_sensor){
    this->t_sensor = t_sensor;
    switch(t_sensor){
    case SIMPLE_CONTACT_DIST:
    {
        sptr_dist = Sptr_cdist( new psm::Contact_distance_model(wrapped_objects) );
        break;
    }
    case FOUR_CONTACT_DIST:
    {
        sptr_dist = Sptr_cdist( new psm::Four_contact_distance_model(wrapped_objects) );
        break;
    }
    case THREE_PIN_DIST:
    {
        sptr_dist = Sptr_three_dist( new psm::Three_pin_distance_model(wrapped_objects));
        break;
    }
    case FORCE_IID:
    {
        ptr_sensor_force_idd = Sptr_fii(new psm::Force_iid_model(SIMPLE));
        break;
    }
    default:
    {
        std::cerr<< "Sensor_manager::initialise no such sensor type: " << t_sensor << std::endl;
        break;
    }
    }
}

void Sensor_manager::update(arma::colvec& Y, const arma::colvec3& pos, const arma::mat33& Rot, const arma::fcolvec& force){
    switch(t_sensor){

    case FOUR_CONTACT_DIST:
    {
        sptr_dist->update(Y,pos,Rot);
        break;
    }
    case SIMPLE_CONTACT_DIST:
    {
        sptr_dist->update(Y,pos,Rot);
        break;
    }
    case THREE_PIN_DIST:
    {
        sptr_dist->update(Y,pos,Rot);
        break;
    }
    case FORCE_IID:
    {
        ptr_sensor_force_idd->update(Y,force);
        break;
    }
    }
}

void Sensor_manager::print(arma::colvec& Y){
    switch(t_sensor){
    case FORCE_IID:
    {
        ptr_sensor_force_idd->print();
        break;
    }
    default:
    {
        sptr_dist->print(Y);
        break;
    }
    }

}

void Sensor_manager::init_visualise(ros::NodeHandle& node){
    if(sptr_dist != NULL){
        std::cout<< "(visual marker initalised for sensor manager)" << std::endl;
        sptr_dist->initialise_vision(node);
    }
}

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

}
