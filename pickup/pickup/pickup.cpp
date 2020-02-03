#include "pickup.h"

bool pickup::epm_status = false;   // make sure that the epm is initially inactive
 
//constructo with member initialization list 
pickup::pickup() : pb_status_srv("/pb_status", &pb_status_cb) , 	
		 toggle_epm_srv("/toggle_epm", &toggle_epm_cb)
{
}
//-----------------------------

bool pickup::toggle_epm_cb(const std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if(pickup::epm_status)
    {
        analogWrite(epm_pin, 50);
        delay(1100);
        analogWrite(epm_pin, 138);
        delay(1500);
        analogWrite(epm_pin, 0);
       // delay(8000);  // a long delay causes a problem with the serial communication. a proper delay must be added in the code that will use this service beteen service calls

        pickup::epm_status = false;
    }
    else
    {
        analogWrite(epm_pin, 250);
        delay(1000);
        analogWrite(epm_pin, 0);
      //  delay(1000);

        pickup::epm_status = true;
    }
    
    res.success = pickup::epm_status;
    res.message = "Recieved EPM toggle Request"; // currently not being used
    return pickup::epm_status;
}

//-----------------------------

bool pickup::pb_status_cb(const std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
        res.success = digitalRead(pb_pin);
        res.message = "Recieved PB_Status Request"; // currently not being used
        return digitalRead(pb_pin);
}
//-----------------------------

void pickup::init(void)
{
    pinMode(epm_pin, OUTPUT);
    pinMode(pb_pin, INPUT);
    digitalWrite(epm_pin, LOW);

    nh.initNode();
    //nh.setSpinTimeout(900);
    nh.advertiseService(toggle_epm_srv);
    nh.advertiseService(pb_status_srv);
}
//-----------------------------

void pickup::spin(void)
{
    nh.spinOnce();
}
