#include "pedro.h"

bool pedro::epm_status = false;

pedro::pedro() : set_epm_status_srv("/set_epm_status", &set_epm_status_cb)
{

}

void pedro ::set_epm_status_cb(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    if(epm_status)
    {
        analogWrite(epm_pin, 50);
        delay(1100);
        analogWrite(epm_pin, 138);
        delay(1500);
        analogWrite(epm_pin, 0);
       // delay(8000);

        epm_status = false;
    }
    else
    {
       // digitalWrite(epm_pin, LOW);
        analogWrite(epm_pin, 250);
        delay(1000);
        analogWrite(epm_pin, 0);
      //  delay(1000);


        epm_status = true;
    }
}

void pedro::init(void)
{
    pinMode(epm_pin, OUTPUT);
    digitalWrite(epm_pin, LOW);
    nh.initNode();
    //nh.setSpinTimeout(900);
    nh.advertiseService(set_epm_status_srv);
}

void pedro::spin(void)
{
    nh.spinOnce();
}
