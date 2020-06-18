
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

#include "robot_faces/utility.h"

// debug variables
bool PRINT_DEBUG_MESS = true;
bool DRAW_REFERENCE_MARKERS = true;

int g_window_width = 800;
int g_window_height = 600;


// positioning
float eye_spacing = 0.5f;
float eye_height = 0.35f;
float eyebrow_spacing = 0.2f;
float nose_height = 0.5f;
float mouth_height = 0.75f;


// colours
sf::Color background_colour(255,255,255,255);
sf::Color nose_colour(41,41,41,255);
sf::Color pupil_colour(0,0,0,255);
sf::Color iris_colour(139,69,19,255);
sf::Color eyebrow_colour(34,27,7,255);

// debug markers
const sf::Color REFERENCE_MARKER_COLOUR(0, 255, 0, 255);
const int REFERENCE_MARKER_RADIUS = 5;
sf::CircleShape reference_marker;


void dynamic_reconfigure_cb(robot_faces::ParametersConfig &config, uint32_t level) {


  if(PRINT_DEBUG_MESS) {
    ROS_INFO("Reconfigure Request");
  }

  // positioning
  eye_spacing = config.eye_spacing;
  eye_height = config.eye_height;
  eyebrow_spacing = config.eyebrow_spacing;
  nose_height = config.nose_height;
  mouth_height = config.mouth_height;

  // colours
  updateColour(background_colour, config.background_colour);
  updateColour(nose_colour, config.nose_colour);
  updateColour(eyebrow_colour, config.eyebrow_colour);
  updateColour(iris_colour, config.iris_colour);
  updateColour(pupil_colour, config.pupil_colour);


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

  reference_marker.setRadius(REFERENCE_MARKER_RADIUS);
  reference_marker.setOrigin(REFERENCE_MARKER_RADIUS, REFERENCE_MARKER_RADIUS);
  reference_marker.setFillColor(REFERENCE_MARKER_COLOUR);


  sf::Event event;

  // main loop
  while(ros::ok() && renderWindow.isOpen()) {

    while(renderWindow.pollEvent(event)) {

        if(event.type == sf::Event::Closed) {
          renderWindow.close();
        }
    }


    // calculate reference points for positioning
    int left_eye_reference_x = int(0.5f*(g_window_width-eye_spacing*g_window_width));
    int right_eye_reference_x = int(g_window_width-0.5f*(g_window_width-eye_spacing*g_window_width));
    int eye_reference_y = int(eye_height*g_window_height);

    int nose_reference_x = int(0.5f*g_window_width);
    int nose_reference_y = int(nose_height*g_window_height);

    int eyebrow_reference_y = int(eye_height*g_window_height-eyebrow_spacing*g_window_height);

    int mouth_reference_x = int(0.5f*g_window_width);
    int mouth_reference_y = int(mouth_height*g_window_height);


    // colours
    renderWindow.clear(background_colour);
    // TODO FILL ELEMENTS WITH COLOUR HERE


    /*
    TODO
    RENDER FACE HERE
    */


    /*
    debug markers
    */
    if(DRAW_REFERENCE_MARKERS) {

      // left eye reference mark
      reference_marker.setPosition(left_eye_reference_x, eye_reference_y);
      renderWindow.draw(reference_marker);

      // right eye reference mark
      reference_marker.setPosition(right_eye_reference_x, eye_reference_y);
      renderWindow.draw(reference_marker);

      // left eyebrow reference mark
      reference_marker.setPosition(left_eye_reference_x, eyebrow_reference_y);
      renderWindow.draw(reference_marker);

      // right eyebrow reference mark
      reference_marker.setPosition(right_eye_reference_x, eyebrow_reference_y);
      renderWindow.draw(reference_marker);

      // nose reference mark
      reference_marker.setPosition(nose_reference_x, nose_reference_y);
      renderWindow.draw(reference_marker);

      // mouth reference mark
      reference_marker.setPosition(mouth_reference_x, mouth_reference_y);
      renderWindow.draw(reference_marker);

    }


    renderWindow.display();

    ros::spinOnce();
  }

  return EXIT_SUCCESS;
}
