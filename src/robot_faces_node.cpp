#include <ros/ros.h>
#include <ros/package.h>

#include <fstream>
#include <string>
#include <iostream> // debug
#include <random>


#include <SFML/Graphics.hpp>
// #include <SFML/Graphics/CircleShape.hpp>

int g_window_width = 800;
int g_window_height = 600;


int main(int argc, char **argv) {

  ros::init(argc, argv, "robot_face");




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
