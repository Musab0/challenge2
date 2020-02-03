#ifndef _pickup_H_
#define _pickup_H_

#include <ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#define epm_pin 9
#define pb_pin  7

class pickup
{

public:

pickup();

static bool toggle_epm_cb(const std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
static bool pb_status_cb(const std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);


void init(void);
void spin(void);

private:

    static bool epm_status;
    ros::NodeHandle nh;
    ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> toggle_epm_srv;
    ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> pb_status_srv;

    
};

#endif
