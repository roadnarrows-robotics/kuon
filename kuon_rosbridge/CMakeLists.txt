cmake_minimum_required(VERSION 2.8.3)
project(kuon_rosbridge)

catkin_package()

INSTALL(DIRECTORY www/img DESTINATION /opt/nginx/html/ros)
INSTALL(DIRECTORY www/js DESTINATION /opt/nginx/html/ros)
INSTALL(FILES www/html/ros/kuon/index.html DESTINATION /opt/nginx/html/ros/kuon)

