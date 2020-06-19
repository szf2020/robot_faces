
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
#include "robot_faces/roundedrectangle.h"

/*
debug variables
*/
bool PRINT_DEBUG_MESS = true;
bool DRAW_REFERENCE_MARKERS = true;

/*
parameters
*/
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

// scaling
float nose_scale = 1.0f; //TODO ADD THIS TO PARAMETERS

/*
sf elements
*/

// nose
enum NoseShape { NO_NOSE, ANNULUS, BUTTON, CURVE, INVERTED_TRIANGLE } noseShape = BUTTON;

int nose_radius = int(g_window_width/15.0f); //TODO a better way of initialising this?

sf::CircleShape nose_annulus, left_nose_curve_fillet, right_nose_curve_fillet;
sf::VertexArray nose_curve_points(sf::TrianglesStrip);
sf::VertexArray nose_inverted_triangle_points(sf::TrianglesFan);





// debug markers
const sf::Color REFERENCE_MARKER_COLOUR(0, 255, 0, 255);
const int REFERENCE_MARKER_RADIUS = 5;
sf::CircleShape reference_marker;



void computeNoseInvertedTrianglePoints() {

  nose_inverted_triangle_points.clear();

  sf::Vector2f initial_position{int(0.5f*g_window_width), 0};
  std::string package_path = ros::package::getPath("robot_faces");

  std::ifstream file(package_path+"/res/inverted_triangle_nose.txt", std::ifstream::in);
  std::string line, x, y;
  if(!getline(file, line)) {
    ROS_ERROR("Could not open file");
  }
  std::stringstream liness(line);

  nose_inverted_triangle_points.append(sf::Vertex(sf::Vector2f(initial_position.x, initial_position.y), nose_colour));

  while (getline(file, line)) {
      std::stringstream liness(line);
      getline(liness, x, ',');
      getline(liness, y);

      // needs to be scaled
      float x_point = strtof(x.c_str(),0)*5.0f;
      float y_point = strtof(y.c_str(),0)*5.0f;

      x_point = (float) initial_position.x + x_point;
      y_point = (float) initial_position.y + y_point;
      nose_inverted_triangle_points.append(sf::Vertex(sf::Vector2f(x_point, y_point), nose_colour));
  }
  file.close();


}

void computeNoseCurvePoints() {

  nose_curve_points.clear();

  // duplication
  float thickness = 5.0f;
  sf::Vector2f initial_position{int(0.5f*g_window_width), 0};

  sf::Vector2f prev_vector = sf::Vector2f(initial_position.x+nose_radius*sin(degToRad(-30)), initial_position.y+nose_radius*cos(degToRad(-30)));
  sf::Vector2f curr_vector = sf::Vector2f(initial_position.x+nose_radius*sin(degToRad(-29)), initial_position.y+nose_radius*cos(degToRad(-29)));


  for(int ang=-29; ang<=30; ang+=1) {

    curr_vector = sf::Vector2f(initial_position.x+nose_radius*sin(degToRad(ang)), initial_position.y+nose_radius*cos(degToRad(ang)));

    sf::Vector2f line = prev_vector - curr_vector;
    sf::Vector2f normal = normalized(sf::Vector2f(-line.y, line.x));

    nose_curve_points.append(sf::Vertex(prev_vector - thickness * normal, nose_colour));
    nose_curve_points.append(sf::Vertex(prev_vector + thickness * normal, nose_colour));

    nose_curve_points.append(sf::Vertex(curr_vector - thickness * normal, nose_colour));
    nose_curve_points.append(sf::Vertex(curr_vector + thickness * normal, nose_colour));

    prev_vector = curr_vector;
  }
}



void dynamic_reconfigure_cb(robot_faces::ParametersConfig &config, uint32_t level) {


  if(PRINT_DEBUG_MESS) {
    ROS_INFO("Reconfigure Request");
  }


  noseShape = static_cast<NoseShape>(config.nose_shape);

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

  // TODO FILL ELEMENTS WITH COLOUR HERE
  nose_annulus.setFillColor(nose_colour);
  left_nose_curve_fillet.setFillColor(nose_colour);
  right_nose_curve_fillet.setFillColor(nose_colour);
  computeNoseCurvePoints();
  computeNoseInvertedTrianglePoints();


}


/*
main
*/
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


  /*
  sf elements
  */

  // nose
  nose_annulus.setRadius(nose_radius);
  nose_annulus.setOrigin(nose_radius, nose_radius);

  sf::Vector2f initial_position{int(0.5f*g_window_width), 0};

  float thickness = 5.0f;
	left_nose_curve_fillet.setRadius(thickness);
	left_nose_curve_fillet.setOrigin(thickness, thickness);
	left_nose_curve_fillet.setPosition(initial_position.x+nose_radius*sin(degToRad(-30)), initial_position.y+nose_radius*cos(degToRad(-30)));
	right_nose_curve_fillet.setRadius(thickness);
	right_nose_curve_fillet.setOrigin(thickness, thickness);
	right_nose_curve_fillet.setPosition(initial_position.x+nose_radius*sin(degToRad(30)), initial_position.y+nose_radius*cos(degToRad(30)));

  computeNoseCurvePoints();

  computeNoseInvertedTrianglePoints();




  // reference markers
  reference_marker.setRadius(REFERENCE_MARKER_RADIUS);
  reference_marker.setOrigin(REFERENCE_MARKER_RADIUS, REFERENCE_MARKER_RADIUS);
  reference_marker.setFillColor(REFERENCE_MARKER_COLOUR);






  // main loop
  while(ros::ok() && renderWindow.isOpen()) {

    sf::Event event;
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


    /*
    TODO
    RENDER FACE HERE
    */

    // nose

    switch(noseShape) {

      case NO_NOSE:
      default:
      break;

      case ANNULUS:
        nose_annulus.setPosition(nose_reference_x, nose_reference_y);
        nose_annulus.setScale(nose_scale, nose_scale);
        nose_annulus.setFillColor(nose_colour);
        renderWindow.draw(nose_annulus);

        nose_annulus.setScale(nose_scale*0.7f, nose_scale*0.7f);
        nose_annulus.setFillColor(background_colour);
        renderWindow.draw(nose_annulus);

      break;

      case BUTTON:
        nose_annulus.setPosition(nose_reference_x, nose_reference_y);
        nose_annulus.setFillColor(nose_colour);
        nose_annulus.setScale(nose_scale, nose_scale);
        renderWindow.draw(nose_annulus);

      break;


      case CURVE:
        {
          sf::Transform t;
          t.translate(0, nose_reference_y);

          //TODO SCALE HERE
          renderWindow.draw(nose_curve_points, t);
          renderWindow.draw(left_nose_curve_fillet, t);
          renderWindow.draw(right_nose_curve_fillet, t);
        }
      break;

      case INVERTED_TRIANGLE:
        sf::Transform t;
        t.translate(0, nose_reference_y);

        //TODO SCALE HERE
        renderWindow.draw(nose_inverted_triangle_points, t);
      break;
    }

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
