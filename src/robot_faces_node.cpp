
#include <fstream>
#include <string>
#include <iostream> // debug
#include <random>

#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <robot_faces/ParametersConfig.h>

#include <SFML/Graphics.hpp>
// #include <SFML/Graphics/CircleShape.hpp>


// debug variables
bool PRINT_DEBUG_MESS = true;

int g_window_width = 800;
int g_window_height = 600;


void dynamic_reconfigure_cb(robot_faces::ParametersConfig &config, uint32_t level) {


  if(PRINT_DEBUG_MESS) {
    ROS_INFO("Reconfigure Request");
  }


}



int main(int argc, char **argv) {

  ros::init(argc, argv, "robot_face");


  dynamic_reconfigure::Server<robot_faces::ParametersConfig> server;
  dynamic_reconfigure::Server<robot_faces::ParametersConfig>::CallbackType f;

  f = boost::bind(&dynamic_reconfigure_cb, _1, _2);
  server.setCallback(f);




  sf::RenderWindow renderWindow(sf::VideoMode(g_window_width, g_window_height), "robot_face");

  // set the framerate to be the same as the monitor's refresh rate to reduce the change of adverse visual artifacts - tearing.
  renderWindow.setVerticalSyncEnabled(true);

  // sometimes vertical synchronistion is forced off by the graphic card so fall back to limiting the framerate to a reasonable figure.
  // renderWindow.setFramerateLimit(30);




  sf::Event event;

  // main loop
  while(ros::ok() && renderWindow.isOpen()) {

    while(renderWindow.pollEvent(event)) {

        if(event.type == sf::Event::Closed) {
          renderWindow.close();
        }
    }


    renderWindow.display();

    ros::spinOnce();
  }

  return EXIT_SUCCESS;
}
