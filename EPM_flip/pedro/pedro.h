#ifndef _PEDRO_H_
#define _PEDRO_H_

#include <ros.h>
#include <std_srvs/Empty.h>

#define epm_pin 9

class pedro
{

public:

pedro();

static void set_epm_status_cb(const std_srvs::Empty::Request&, std_srvs::Empty::Response&);

void init(void);
void spin(void);

private:

    static bool epm_status;
    ros::NodeHandle nh;
    ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> set_epm_status_srv;
};

#endif
