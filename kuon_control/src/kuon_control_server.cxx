//
//  TODO (dhp) - set header
//

#include "ros/ros.h"

#include "kuon_control/QueryVersion.h"
#include "kuon_control/Version.h"

bool kuon_version(kuon_control::QueryVersion::Request &req,
                 kuon_control::QueryVersion::Response &rsp)
{
  ROS_INFO("Serving 'QueryVersion' request (DUMMY)");
  rsp.v.maj=0;
  rsp.v.min=0;
  rsp.v.rev=0;

  rsp.v.configuration="Deluxe";

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kuon_control_server");
  ros::NodeHandle n;

  ros::ServiceServer versionService = 
       n.advertiseService("kuon_version", kuon_version);

  ROS_INFO("Hekateros Server up and running - Get your arm on!");
  ros::spin();

  return 0;
}
