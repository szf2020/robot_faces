/*
Copyright 2020 Andrew Murtagh

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <fstream>
#include <string>
#include <iostream>
#include <random>

#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>

#include <robot_faces/ParametersConfig.h>
#include <robot_faces/Expression.h>
#include <robot_faces/Gaze.h>

#include <SFML/Graphics.hpp>

#include "robot_faces/utility.h"
#include "robot_faces/roundedrectangle.h"

#include "robot_faces/nose.h"
#include "robot_faces/mouth.h"
#include "robot_faces/eyebrow.h"

/*
debug variables - internal use by the author
*/
bool PRINT_DEBUG_MESSAGES = true;
bool DRAW_REFERENCE_MARKERS = true;


/*
consts
*/
int g_window_width = 800;
int g_window_height = 600;


//TODO a better way of initialising these?
const int DEFAULT_PUPIL_DIAMETER = int(g_window_width/15.0f);
const int DEFAULT_IRIS_DIAMETER = int(g_window_width/7.0f);
const float CLOSE_ENOUGH_THRESHOLD = 3.0f; // in px
const float BLINK_CLOSE_ENOUGH_THRESHOLD = 6.0f; // in px - more leeway is given to blinking because it moves much faster
const float SACCADE_SPEED = 90.0f; // px per second
const float BLINK_SPEED = 220.0f; // px per second
const float EXPRESSION_SPEED = 140.0f; //px per second
const float TEMP_EYELID_HEIGHT = 300.0f; //TODO CHANGE THIS LATER
const sf::Color REFERENCE_MARKER_COLOUR(0, 255, 0, 255);
const int REFERENCE_MARKER_RADIUS = 5;
const sf::Color BEZIER_MARKER_COLOUR(255, 0, 0, 255);


/*
parameters
*/
// positioning
float eye_spacing           = 0.5f;
float eye_height            = 0.35f;
float eyebrow_spacing       = 0.2f;
float nose_height           = 0.5f;
float mouth_height          = 0.75f;

float pupil_corner_radius   = 1.0f;
float iris_corner_radius    = 1.0f;
bool will_do_saccades       = true;
bool will_blink             = true;
int avr_blink_interval      = 3000; //ms

// colours
sf::Color background_colour(255,255,255,255);
sf::Color pupil_colour(0,0,0,255);
sf::Color iris_colour(139,69,19,255);

// display toggles
bool show_iris              = true;
bool show_pupil             = true;

// scaling
float eye_scaling_x         = 1.0f;
float eye_scaling_y         = 1.0f;

enum struct IrisShape {
  ROUNDED_RECTANGLE,
  THICK,
  OVAL,
  ALMOND,
  ARC
} irisShape                 = IrisShape::ROUNDED_RECTANGLE;



/*
sf elements
*/

Nose nose;

// pupil
sf::RoundedRectangle pupil_shape, pupil_highlight;
int gaze_offset_x, gaze_offset_y, saccade_offset_x, saccade_offset_y;
sf::Vector2f goal_pupil_offset, curr_pupil_offset; // relative to the reference point in px

// iris
sf::RoundedRectangle iris_shape;
sf::VertexArray iris_points(sf::TrianglesFan);
sf::Vector2f curr_iris_offset; // relative to the reference point in px, no need for a goal. The iris just trails the pupil goal position

// eyebrows
// sf::VertexArray eyebrow_points(sf::TrianglesFan);

Eyebrow left_eyebrow(LEFT);
Eyebrow right_eyebrow(RIGHT);

// mouth
Mouth mouth;

// eyelids
sf::RectangleShape top_eyelid, bottom_eyelid;

// debug markers
sf::CircleShape reference_marker, bezier_marker;


/*
others
*/

// TODO MOVE THESE TO REST OF CONSTS
float gaze_azimuth = 0.0f; // -1 to 1
float gaze_elevation = 0.0f; // -1 to 1
int gaze_radius = 60; // maximum radius of gaze


// state machine to manage blinking
enum struct BlinkState { OPEN, CLOSING, OPENING } currBlinkingState = BlinkState::OPEN;


sf::Clock frame_clock; // gets time from one frame to the next

std::mt19937 eng;
sf::Clock saccade_clock, blink_clock, gaze_clock; // measures time between saccades and blinks

int gaze_timeout = 0;

int saccade_interval;
std::uniform_int_distribution<int> saccade_time_dist(200, 400); // milliseconds between saccades - based on wiki article about saccadic frequency in humans
std::uniform_int_distribution<int> saccade_pos_dist(-10, 10); // position to move for a saccade in px TODO HOW TO INIT MAX

int blink_interval;
std::uniform_int_distribution<int> blink_time_dist(avr_blink_interval-avr_blink_interval/2.0f, avr_blink_interval-avr_blink_interval/2.0f); // milliseconds between blinks, once every 2.85 seconds on average for humans



/*
helper functions
//TODO MOVE THESE UTILITY FILE
//TODO REFACTOR TO USE generateLineWThickness FUNCTION
*/
void generateIrisPoints() {

  std::string filename = "";
  if(irisShape==IrisShape::THICK) {
    filename="iris_thick";
  } else if(irisShape==IrisShape::OVAL) {
    filename="iris_oval";
  } else if(irisShape==IrisShape::ALMOND) {
    filename="iris_almond";
  } else if(irisShape==IrisShape::ARC) {
    filename="iris_arc";
  } else {
    return;
  }
  iris_points.clear();
  sf::Vector2f initial_position{0, 0}; //TODO REMOVE
  std::string package_path = ros::package::getPath("robot_faces");

  std::ifstream file(package_path+"/res/"+filename+".txt", std::ifstream::in);

  std::string line, x, y;
  if(!getline(file, line)) {
    ROS_ERROR("Could not open file");
  }
  std::stringstream liness(line);

  iris_points.append(sf::Vertex(sf::Vector2f(initial_position.x, initial_position.y), iris_colour));

  while (getline(file, line)) {
      std::stringstream liness(line);
      getline(liness, x, ',');
      getline(liness, y);


      float x_point = strtof(x.c_str(),0);
      float y_point = strtof(y.c_str(),0);

      // each of the drawings is 100px wide so we scale it by whatever the default width of the eyes is
      x_point *= DEFAULT_IRIS_DIAMETER/100.f;
      y_point *= DEFAULT_IRIS_DIAMETER/100.f;

      x_point = (float) initial_position.x + x_point;
      y_point = (float) initial_position.y + y_point;
      iris_points.append(sf::Vertex(sf::Vector2f(x_point, y_point), iris_colour));
  }
  file.close();

}


/*
callback functions
*/
void dynamicReconfigureCb(robot_faces::ParametersConfig& config, uint32_t level) {


  if(PRINT_DEBUG_MESSAGES) {
    ROS_INFO("Reconfigure Request");
  }


  // misc

  irisShape = static_cast<IrisShape>(config.iris_shape);

  nose.setShape(static_cast<Nose::NoseShape>(config.nose_shape));

  left_eyebrow.setShape(static_cast<Eyebrow::EyebrowShape>(config.eyebrow_shape));
  right_eyebrow.setShape(static_cast<Eyebrow::EyebrowShape>(config.eyebrow_shape));

  pupil_corner_radius = config.pupil_corner_radius;
  pupil_shape.setCornersRadius(pupil_corner_radius*DEFAULT_PUPIL_DIAMETER/2.0f);

  iris_corner_radius = config.iris_corner_radius;
  iris_shape.setCornersRadius(iris_corner_radius*DEFAULT_IRIS_DIAMETER/2.0f);

  will_blink = config.will_blink;
  avr_blink_interval = config.avr_blink_interval;
  blink_time_dist = std::uniform_int_distribution<int>(avr_blink_interval-avr_blink_interval/2.0f, avr_blink_interval-avr_blink_interval/2.0f); // milliseconds

  will_do_saccades = config.will_do_saccades;

  // positioning
  eye_spacing = config.eye_spacing;
  eye_height = config.eye_height;
  eyebrow_spacing = config.eyebrow_spacing;
  nose_height = config.nose_height;
  mouth_height = config.mouth_height;

  nose.setPosition(int(0.5f*g_window_width), int(nose_height*g_window_height));

  mouth.setPosition(int(0.5f*g_window_width), int(mouth_height*g_window_height));

  left_eyebrow.setPosition(int(0.5f*(g_window_width-eye_spacing*g_window_width)), int(eye_height*g_window_height-eyebrow_spacing*g_window_height));
  right_eyebrow.setPosition(int(g_window_width-0.5f*(g_window_width-eye_spacing*g_window_width)), int(eye_height*g_window_height-eyebrow_spacing*g_window_height));



  // colours
  updateColour(background_colour, config.background_colour);
  top_eyelid.setFillColor(background_colour);
  bottom_eyelid.setFillColor(background_colour);

  nose.setColour(config.nose_colour);

  // updateColour(eyebrow_colour, config.eyebrow_colour);
  left_eyebrow.setColour(config.eyebrow_colour);
  right_eyebrow.setColour(config.eyebrow_colour);

  updateColour(iris_colour, config.iris_colour);
  iris_shape.setFillColor(iris_colour);

  updateColour(pupil_colour, config.pupil_colour);
  pupil_shape.setFillColor(pupil_colour);

  mouth.setColour(config.mouth_colour);

  // display toggles
  show_iris = config.show_iris;
  show_pupil = config.show_pupil;
  nose.setShow(config.show_nose);

  mouth.setShow(config.show_mouth);

  left_eyebrow.setShow(config.show_eybrows);
  right_eyebrow.setShow(config.show_eybrows);

  // scaling
  nose.setScaleX(config.nose_scaling);

  eye_scaling_x = config.eye_scaling_x;
  eye_scaling_y = config.eye_scaling_y;

  left_eyebrow.setScaleX(config.eyebrow_scaling);
  right_eyebrow.setScaleX(config.eyebrow_scaling);

  mouth.setScaleX(config.mouth_scaling_x);
  mouth.setScaleY(config.mouth_scaling_y);


  // recompute vertexarray points
  generateIrisPoints();
}


bool setExpressionCb(robot_faces::Expression::Request& req, robot_faces::Expression::Response& res) {

  if(PRINT_DEBUG_MESSAGES) {
    ROS_INFO_STREAM("Change Expression Request: " << req.expression << ", " << req.timeout);
  }

  std::string expr = req.expression;

  //convert to uppercase
  std::transform(expr.begin(), expr.end(), expr.begin(), [](unsigned char c){ return std::toupper(c); });

  if(expr == "NEUTRAL") {
    mouth.setGoalPoints(neutral_mouth_bezier_points);

  } else if(expr == "SADNESS") {
    mouth.setGoalPoints(sadness_mouth_bezier_points);

  } else if(expr == "FEAR") {
    mouth.setGoalPoints(fear_mouth_bezier_points);

  } else if(expr == "DISGUST") {
    mouth.setGoalPoints(disgust_mouth_bezier_points);

  } else if(expr == "ANGER") {
    mouth.setGoalPoints(anger_mouth_bezier_points);

  } else if(expr == "JOY") {
    mouth.setGoalPoints(joy_mouth_bezier_points);

  } else if(expr == "HAPPINESS") {
    mouth.setGoalPoints(happiness_mouth_bezier_points);

  } else if(expr == "AWE") {
    mouth.setGoalPoints(awe_mouth_bezier_points);

  } else if(expr == "SURPRISE") {
    mouth.setGoalPoints(surprise_mouth_bezier_points);

  } else {
    ROS_ERROR("Expression not recognised");
  }

  return true;
}


bool setGazeCb(robot_faces::Gaze::Request& req, robot_faces::Gaze::Response& res) {

  if(PRINT_DEBUG_MESSAGES) {
    ROS_INFO_STREAM("Change Gaze Request " << req.elevation << ", " << req.azimuth << ", " << req.timeout);
  }

  // handle timeout first and check for error to return prematurely
  if(req.timeout<0) {
    ROS_ERROR("Gaze timeout cannot be negative.");
    res.done = false;
    return true;
  }

  if(req.timeout>60000) {
    ROS_ERROR("Gaze timeout cannot be more than one minute.");
    res.done = false;
    return true;
  }

  gaze_timeout = req.timeout;
  gaze_clock.restart();
  res.done = true;

  gaze_elevation = req.elevation;

  // clamp to within bounds instead of raising error
  if(gaze_elevation<-1.0f) {
    gaze_elevation = -1.0f;
  }
  if(gaze_elevation>1.0f) {
    gaze_elevation = 1.0f;
  }

  gaze_azimuth = req.azimuth;

  // clamp to within bounds instead of raising error
  if(gaze_azimuth<-1.0f) {
    gaze_azimuth = -1.0f;
  }
  if(gaze_azimuth>1.0f) {
    gaze_azimuth = 1.0f;
  }

  gaze_offset_x = int(gaze_azimuth*gaze_radius);
  gaze_offset_y = int(-1.0f*gaze_elevation*gaze_radius);

  return true;
}




/*
main
*/
int main(int argc, char **argv) {

  ros::init(argc, argv, "robot_face");

  ros::NodeHandle node_handle;

  dynamic_reconfigure::Server<robot_faces::ParametersConfig> server;
  dynamic_reconfigure::Server<robot_faces::ParametersConfig>::CallbackType f;

  f = boost::bind(&dynamicReconfigureCb, _1, _2);
  server.setCallback(f);

  ros::ServiceServer expression_server = node_handle.advertiseService("expression", setExpressionCb);
  ros::ServiceServer gaze_server = node_handle.advertiseService("gaze", setGazeCb);



	std::random_device rd;
	eng.seed(rd());

	saccade_interval = saccade_time_dist(eng);
  blink_interval = blink_time_dist(eng);

  sf::RenderWindow renderWindow(sf::VideoMode(g_window_width, g_window_height), "robot_face");

  // set the framerate to be the same as the monitor's refresh rate to reduce the change of adverse visual artifacts - tearing.
  renderWindow.setVerticalSyncEnabled(true);

  // sometimes vertical synchronistion is forced off by the graphic card so fall back to limiting the framerate to a reasonable figure.
  // renderWindow.setFramerateLimit(30);




  /*
  init sf elements
  */

  // nose
  nose.setPosition(int(0.5f*g_window_width), int(nose_height*g_window_height));


  mouth.setPosition(int(0.5f*g_window_width), int(mouth_height*g_window_height));

  left_eyebrow.setPosition(int(0.5f*(g_window_width-eye_spacing*g_window_width)), int(eye_height*g_window_height-eyebrow_spacing*g_window_height));
  right_eyebrow.setPosition(int(g_window_width-0.5f*(g_window_width-eye_spacing*g_window_width)), int(eye_height*g_window_height-eyebrow_spacing*g_window_height));




  // pupil
  pupil_shape.setSize(sf::Vector2f(DEFAULT_PUPIL_DIAMETER, DEFAULT_PUPIL_DIAMETER));
  pupil_shape.setOrigin(DEFAULT_PUPIL_DIAMETER/2.0f, DEFAULT_PUPIL_DIAMETER/2.0f);
  pupil_shape.setCornersRadius(pupil_corner_radius* DEFAULT_PUPIL_DIAMETER/2.0f);
  pupil_shape.setCornerPointCount(20);
  pupil_shape.setFillColor(pupil_colour);

  pupil_highlight.setSize(sf::Vector2f(DEFAULT_PUPIL_DIAMETER/4.0f, DEFAULT_PUPIL_DIAMETER/4.0f));
  pupil_highlight.setOrigin(DEFAULT_PUPIL_DIAMETER/8.0f, DEFAULT_PUPIL_DIAMETER/8.0f);
  pupil_highlight.setCornersRadius(DEFAULT_PUPIL_DIAMETER/8.0f);
  pupil_highlight.setCornerPointCount(10);
  pupil_highlight.setFillColor(sf::Color(255,255,255,200));

  goal_pupil_offset = sf::Vector2f(0, 0);
  curr_pupil_offset = goal_pupil_offset;


  // iris
  iris_shape.setSize(sf::Vector2f(DEFAULT_IRIS_DIAMETER, DEFAULT_IRIS_DIAMETER));
  iris_shape.setOrigin(DEFAULT_IRIS_DIAMETER/2.0f, DEFAULT_IRIS_DIAMETER/2.0f);
  iris_shape.setCornersRadius(iris_corner_radius*DEFAULT_IRIS_DIAMETER/2.0f);
  iris_shape.setCornerPointCount(20);
  iris_shape.setFillColor(iris_colour);
  generateIrisPoints();

  curr_iris_offset = goal_pupil_offset;


  // eyebrows



  // eyelids
  top_eyelid.setSize(sf::Vector2f(g_window_width, TEMP_EYELID_HEIGHT));
  top_eyelid.setFillColor(background_colour);
  top_eyelid.setOrigin(g_window_width/2.0f, TEMP_EYELID_HEIGHT/2.0f);

  bottom_eyelid.setSize(sf::Vector2f(g_window_width, TEMP_EYELID_HEIGHT));
  bottom_eyelid.setFillColor(background_colour);
  bottom_eyelid.setOrigin(g_window_width/2.0f, TEMP_EYELID_HEIGHT/2.0f);


  // reference markers
  reference_marker.setRadius(REFERENCE_MARKER_RADIUS);
  reference_marker.setOrigin(REFERENCE_MARKER_RADIUS, REFERENCE_MARKER_RADIUS);
  reference_marker.setFillColor(REFERENCE_MARKER_COLOUR);





  /*
  main loop
  */
  while(ros::ok() && renderWindow.isOpen()) {

    sf::Event event;
    while(renderWindow.pollEvent(event)) {

        if(event.type == sf::Event::Closed) {
          renderWindow.close();
        }
    }


    // timings
    float frame_delta_time = frame_clock.getElapsedTime().asMilliseconds()/1000.0f; // time since last frame draw
    frame_clock.restart();


    if(will_blink && blink_clock.getElapsedTime().asMilliseconds() > blink_interval) {

      if(PRINT_DEBUG_MESSAGES) {
        ROS_INFO("Perform blink");
      }

      currBlinkingState = BlinkState::CLOSING;

      blink_interval = blink_time_dist(eng);
      blink_clock.restart();
    }


    if(will_do_saccades && saccade_clock.getElapsedTime().asMilliseconds() > saccade_interval) {

      if(PRINT_DEBUG_MESSAGES) {
        // ROS_INFO("Perform saccade");
      }

      saccade_offset_x = saccade_pos_dist(eng);
      saccade_offset_y = saccade_pos_dist(eng);

      saccade_interval = saccade_time_dist(eng);
      saccade_clock.restart();
    }

    // gaze timeout
    if(gaze_timeout!=0 && gaze_clock.getElapsedTime().asMilliseconds() > gaze_timeout) {

      if(PRINT_DEBUG_MESSAGES) {
        ROS_INFO("Gaze timed out");
      }

      gaze_offset_x = gaze_offset_y = gaze_timeout = 0;

    }


    // calculate reference points for positioning
    int left_eye_reference_x = int(0.5f*(g_window_width-eye_spacing*g_window_width));
    int right_eye_reference_x = int(g_window_width-0.5f*(g_window_width-eye_spacing*g_window_width));
    int eye_reference_y = int(eye_height*g_window_height);


    // int eyebrow_reference_y = int(eye_height*g_window_height-eyebrow_spacing*g_window_height);


    renderWindow.clear(background_colour);


    /*
    RENDER FACE
    */

    // interpolate between curr and goal positions
    if(will_do_saccades) {
      goal_pupil_offset = sf::Vector2f(gaze_offset_x+saccade_offset_x, gaze_offset_y+saccade_offset_y);
    } else {
      goal_pupil_offset = sf::Vector2f(gaze_offset_x, gaze_offset_y);
    }


    if(getDistance(goal_pupil_offset, curr_pupil_offset) < CLOSE_ENOUGH_THRESHOLD) {
      curr_pupil_offset = goal_pupil_offset;
      curr_iris_offset = goal_pupil_offset*0.6f;
    } else {
      sf::Vector2f pupil_direction = normalize(goal_pupil_offset - curr_pupil_offset);
      curr_pupil_offset += frame_delta_time*SACCADE_SPEED*pupil_direction;
      curr_iris_offset += frame_delta_time*SACCADE_SPEED*pupil_direction*0.6f;
    }



    // iris
    if(show_iris) {

      if(irisShape==IrisShape::THICK || irisShape==IrisShape::OVAL || irisShape==IrisShape::ALMOND || irisShape==IrisShape::ARC) {
        sf::Transform t(1.f*eye_scaling_x, 0.f, left_eye_reference_x+curr_iris_offset.x,
                         0.f,  eye_scaling_y, eye_reference_y+curr_iris_offset.y,
                         0.f,  0.f, 1.f);
        renderWindow.draw(iris_points, t);

        t = sf::Transform(-1.f*eye_scaling_x, 0.f, right_eye_reference_x+curr_iris_offset.x,
                         0.f,  eye_scaling_y, eye_reference_y+curr_iris_offset.y,
                         0.f,  0.f, 1.f);

        renderWindow.draw(iris_points, t);

      } else {
        iris_shape.setScale(eye_scaling_x, eye_scaling_y);
        iris_shape.setPosition(left_eye_reference_x+curr_iris_offset.x, eye_reference_y+curr_iris_offset.y);
        renderWindow.draw(iris_shape);
        iris_shape.setPosition(right_eye_reference_x+curr_iris_offset.x, eye_reference_y+curr_iris_offset.y);
        renderWindow.draw(iris_shape);
      }

    }


    // pupils
    if(show_pupil) {

      pupil_shape.setPosition(left_eye_reference_x+curr_pupil_offset.x, eye_reference_y+curr_pupil_offset.y);
      renderWindow.draw(pupil_shape);
      pupil_shape.setPosition(right_eye_reference_x+curr_pupil_offset.x, eye_reference_y+curr_pupil_offset.y);
      renderWindow.draw(pupil_shape);

    }


    // eyelids
    if(will_blink) {
      //TODO SCALE HERE
      //TODO IF BLINKING IS ON
      //TODO CHANGE THIS. HALF THE HEIGHT OF THE EYE BROW PLUS THE HEIGHT OF THE EYE
      int top_open_y = eye_reference_y+curr_pupil_offset.y-TEMP_EYELID_HEIGHT/2.0f-DEFAULT_IRIS_DIAMETER/2.0f*eye_scaling_y-20.0f;
      int bottom_open_y = eye_reference_y+curr_pupil_offset.y+TEMP_EYELID_HEIGHT/2.0f+DEFAULT_IRIS_DIAMETER/2.0f*eye_scaling_y+20.0f;
      int top_closed_y = eye_reference_y+curr_pupil_offset.y-TEMP_EYELID_HEIGHT/2.0f;
      int bottom_closed_y = eye_reference_y+curr_pupil_offset.y+TEMP_EYELID_HEIGHT/2.0f;
      int top_curr_y, bottom_curr_y;

      switch(currBlinkingState) {

          case BlinkState::OPEN:
            top_curr_y = top_open_y;
            bottom_curr_y = bottom_open_y;
          break;


          case BlinkState::CLOSING:
            { // scoped
              if(top_curr_y >= top_closed_y-BLINK_CLOSE_ENOUGH_THRESHOLD || bottom_curr_y <= bottom_closed_y+BLINK_CLOSE_ENOUGH_THRESHOLD) {
                top_curr_y = top_closed_y;
                  bottom_curr_y = bottom_closed_y;
                currBlinkingState = BlinkState::OPENING;
              } else {
                float this_frame_displacement = frame_delta_time*BLINK_SPEED;
                top_curr_y += this_frame_displacement;
                bottom_curr_y -= this_frame_displacement;
              }
            }
          break;

          case BlinkState::OPENING:
            { // scoped
              if(top_curr_y <= top_open_y+BLINK_CLOSE_ENOUGH_THRESHOLD || bottom_curr_y >= bottom_open_y-BLINK_CLOSE_ENOUGH_THRESHOLD) {
                top_curr_y = top_open_y;
                bottom_curr_y = bottom_open_y;
                currBlinkingState = BlinkState::OPEN;
              } else {
                float this_frame_displacement = frame_delta_time*BLINK_SPEED;
                top_curr_y -= this_frame_displacement;
                bottom_curr_y += this_frame_displacement;
              }
            }
          break;

      }

      top_eyelid.setPosition(g_window_width/2.0f, top_curr_y);
      renderWindow.draw(top_eyelid);

      bottom_eyelid.setPosition(g_window_width/2.0f, bottom_curr_y);
      renderWindow.draw(bottom_eyelid);
    }


    // eyebrows
    left_eyebrow.draw(renderWindow, frame_delta_time);
    right_eyebrow.draw(renderWindow, frame_delta_time);


    // mouth
    mouth.draw(renderWindow, frame_delta_time);


    // nose
    nose.draw(renderWindow, frame_delta_time);


    //debug markers
    if(DRAW_REFERENCE_MARKERS) {

      // left eye reference mark
      reference_marker.setPosition(left_eye_reference_x, eye_reference_y);
      renderWindow.draw(reference_marker);

      // right eye reference mark
      reference_marker.setPosition(right_eye_reference_x, eye_reference_y);
      renderWindow.draw(reference_marker);


    }


    renderWindow.display();

    ros::spinOnce();
  }

  return EXIT_SUCCESS;
}
