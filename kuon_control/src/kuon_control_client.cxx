//
//  TODO (dhp) - set header
//

#include <string>
#include "ros/ros.h"


#include "kuon_control/QueryVersion.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kuon_control_client");
  ros::NodeHandle n;

  ros::ServiceClient versionClient = 
       n.serviceClient<kuon_control::QueryVersion>("kuon_version");

  kuon_control::QueryVersion srv;
  if(versionClient.call(srv))
  {
    ROS_INFO("Kuon Version: %d.%d.%d-%s", (int)srv.response.v.maj, (int)srv.response.v.min, (int)srv.response.v.rev, srv.response.v.configuration.c_str());
  }

  return 0;
}
