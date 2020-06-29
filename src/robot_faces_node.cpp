
#include <fstream>
#include <string>
#include <iostream>
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
bool PRINT_DEBUG_MESSAGES = true;
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
sf::Color mouth_colour(0,0,0,255);

// display toggles
bool show_eybrows = true;
bool show_iris = true;
bool show_pupil = true;
bool show_nose = false;
bool show_mouth = true;

// scaling
float nose_scaling = 1.0f;
float eye_scaling_x = 1.0f;
float eye_scaling_y = 1.0f;
float eyebrow_scaling = 1.0f;
float mouth_scaling_x = 1.0f;
float mouth_scaling_y = 1.0f;

/*
sf elements
*/

// nose
enum NoseShape { ANNULUS, BUTTON, CURVE, INVERTED_TRIANGLE } noseShape = BUTTON;

float nose_curve_thickness=5.0f;
int nose_radius = int(g_window_width/15.0f); //TODO a better way of initialising this?

sf::CircleShape nose_annulus, left_nose_curve_fillet, right_nose_curve_fillet;
sf::VertexArray nose_curve_points(sf::TrianglesStrip);
sf::VertexArray nose_inverted_triangle_points(sf::TrianglesFan);

// pupil
sf::RoundedRectangle pupil_shape;
int pupil_radius = int(g_window_width/15.0f);
float pupil_corner_radius = 1.0f;
sf::Vector2f goal_pupil_pos, curr_pupil_pos; // relative to the reference point in px

// iris
enum struct IrisShape { ROUNDED_RECTANGLE, THICK, OVAL, ALMOND, ARC } irisShape = IrisShape::ROUNDED_RECTANGLE;
sf::RoundedRectangle iris_shape;
int iris_diameter = int(g_window_width/7.0f);
float iris_corner_radius = 1.0f;
sf::VertexArray iris_points(sf::TrianglesFan);
sf::Vector2f curr_iris_pos; // relative to the reference point in px, no need for a goal. The iris just trails the pupil goal position

// eyebrows
enum struct EyebrowShape { CIRCULAR_ARC, RECTANGULAR, SQUARE, ROUNDED, STRAIGHT, HIGH_ARCH } eyebrowShape = EyebrowShape::CIRCULAR_ARC;
sf::VertexArray eyebrow_points(sf::TrianglesFan);


// mouth
const int MOUTH_THICKNESS = 8.0f; //note this is actually half the thickness, make a parameter
std::vector<sf::Vector2f> upper_mouth_vertices, lower_mouth_vertices;
sf::CircleShape mouth_fillet;

struct BezierPoints {
    sf::Vector2f upper_start, upper_end, upper_start_control, upper_end_control;
    sf::Vector2f lower_start, lower_end, lower_start_control, lower_end_control;

    bool operator==(const BezierPoints &) const;
    bool withinDelta(const BezierPoints &) const;
};

//TODO CHANGE THESE LATER
float center_x = (float) g_window_width/2.0f;
float quarter_x = (float) g_window_width/4.0f;
float eight_x = (float) g_window_width/8.0f;
float center_y = (float) g_window_height/2.0f;
float quarter_y = (float) g_window_height/4.0f;
float eight_y = (float) g_window_height/8.0f;

BezierPoints curr_mouth_points;

const float CLOSE_ENOUGH_THRESHOLD = 3.0f; // in px

//bezier points enum member functions
bool BezierPoints::operator== (const BezierPoints &other_point) const {
    if(upper_start == other_point.upper_start &&
        upper_end == other_point.upper_end &&
        upper_start_control == other_point.upper_start_control &&
        upper_end_control == other_point.upper_end_control &&

        lower_start == other_point.lower_start &&
        lower_end == other_point.lower_end &&
        lower_start_control == other_point.lower_start_control &&
        lower_end_control == other_point.lower_end_control) {

        return true;
    } else {
        return false;
    }
}

bool BezierPoints::withinDelta(const BezierPoints &other_point) const {

    if(getDistance(upper_start, other_point.upper_start) < CLOSE_ENOUGH_THRESHOLD &&
        getDistance(upper_end, other_point.upper_end) < CLOSE_ENOUGH_THRESHOLD &&
        getDistance(upper_start_control, other_point.upper_start_control) < CLOSE_ENOUGH_THRESHOLD &&
        getDistance(upper_end_control, other_point.upper_end_control) < CLOSE_ENOUGH_THRESHOLD &&

        getDistance(lower_start, other_point.lower_start) < CLOSE_ENOUGH_THRESHOLD &&
        getDistance(lower_end, other_point.lower_end) < CLOSE_ENOUGH_THRESHOLD &&
        getDistance(lower_start_control, other_point.lower_start_control) < CLOSE_ENOUGH_THRESHOLD &&
        getDistance(lower_end_control, other_point.lower_end_control) < CLOSE_ENOUGH_THRESHOLD) {
        return true;
    } else {
        return false;
    }
}


// eyelids
// state machine to manage blinking
enum struct BlinkState { OPEN, DOWN, UP } curr_blinking_state = BlinkState::OPEN;
sf::RectangleShape top_eyelid, bottom_eyelid;
// sf::Vector2f top_eyelid_curr_pos, bottom_eyelid_curr_pos;

// debug markers
const sf::Color REFERENCE_MARKER_COLOUR(0, 255, 0, 255);
const int REFERENCE_MARKER_RADIUS = 5;
sf::CircleShape reference_marker;


/*

behaviour

*/

const float SACCADE_SPEED = 90.0f; // px per second

sf::Clock frame_clock; // gets time from one frame to the next

std::mt19937 eng;
sf::Clock saccade_clock; // measures time between saccades

int saccade_interval;
std::uniform_int_distribution<int> saccade_time_dist(1000, 5000); // milliseconds between saccades
std::uniform_int_distribution<int> saccade_pos_dist(-60, 60); // position to move for a saccade in px TODO HOW TO INIT MAX




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
      x_point *= iris_diameter/100.f;
      y_point *= iris_diameter/100.f;

      x_point = (float) initial_position.x + x_point;
      y_point = (float) initial_position.y + y_point;
      iris_points.append(sf::Vertex(sf::Vector2f(x_point, y_point), iris_colour));
  }
  file.close();

}

void generateNoseInvertedTrianglePoints() {

  nose_inverted_triangle_points.clear();

  sf::Vector2f initial_position{0, 0}; //TODO REMOVE
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
      float x_point = strtof(x.c_str(),0)*5.0f*nose_scaling;
      float y_point = strtof(y.c_str(),0)*5.0f*nose_scaling;

      x_point = (float) initial_position.x + x_point;
      y_point = (float) initial_position.y + y_point;
      nose_inverted_triangle_points.append(sf::Vertex(sf::Vector2f(x_point, y_point), nose_colour));
  }
  file.close();

}

void generateNoseCurvePoints() {

  nose_curve_points.clear();

  float thickness = nose_scaling*nose_curve_thickness;
  float radius = nose_scaling*nose_radius;
  sf::Vector2f initial_position{0, -radius/2.0f};

  sf::Vector2f prev_vector = sf::Vector2f(initial_position.x+radius*sin(degToRad(-30)), initial_position.y+radius*cos(degToRad(-30)));
  sf::Vector2f curr_vector = sf::Vector2f(initial_position.x+radius*sin(degToRad(-29)), initial_position.y+radius*cos(degToRad(-29)));


  for(int ang=-29; ang<=30; ang+=1) {

    curr_vector = sf::Vector2f(initial_position.x+radius*sin(degToRad(ang)), initial_position.y+radius*cos(degToRad(ang)));

    sf::Vector2f line = prev_vector - curr_vector;
    sf::Vector2f normal = normalize(sf::Vector2f(-line.y, line.x));

    nose_curve_points.append(sf::Vertex(prev_vector - thickness * normal, nose_colour));
    nose_curve_points.append(sf::Vertex(prev_vector + thickness * normal, nose_colour));

    nose_curve_points.append(sf::Vertex(curr_vector - thickness * normal, nose_colour));
    nose_curve_points.append(sf::Vertex(curr_vector + thickness * normal, nose_colour));

    prev_vector = curr_vector;
  }


  	left_nose_curve_fillet.setRadius(thickness);
  	left_nose_curve_fillet.setOrigin(thickness, thickness);
  	left_nose_curve_fillet.setPosition(initial_position.x+radius*sin(degToRad(-30)), initial_position.y+radius*cos(degToRad(-30)));
  	right_nose_curve_fillet.setRadius(thickness);
  	right_nose_curve_fillet.setOrigin(thickness, thickness);
  	right_nose_curve_fillet.setPosition(initial_position.x+radius*sin(degToRad(30)), initial_position.y+radius*cos(degToRad(30)));
}

void generateEyebrowPoints() {

  std::string filename = "eyebrow_arc";

  switch (eyebrowShape) {
    case EyebrowShape::CIRCULAR_ARC:
    default:
      filename="eyebrow_arc";
    break;

    case EyebrowShape::RECTANGULAR:
      filename="eyebrow_rectangular";
    break;

    case EyebrowShape::SQUARE:
      filename="eyebrow_square";
    break;

    case EyebrowShape::ROUNDED:
      filename="eyebrow_rounded";
    break;

    case EyebrowShape::STRAIGHT:
      filename="eyebrow_straight";
    break;

    case EyebrowShape::HIGH_ARCH:
      filename="eyebrow_high_arch";
    break;

  }


  eyebrow_points.clear();
  sf::Vector2f initial_position{0, 0}; //TODO REMOVE
  std::string package_path = ros::package::getPath("robot_faces");

  std::ifstream file(package_path+"/res/"+filename+".txt", std::ifstream::in);

  std::string line, x, y;
  if(!getline(file, line)) {
    ROS_ERROR("Could not open file");
  }
  std::stringstream liness(line);

  eyebrow_points.append(sf::Vertex(sf::Vector2f(initial_position.x, initial_position.y), eyebrow_colour));

  while (getline(file, line)) {
      std::stringstream liness(line);
      getline(liness, x, ',');
      getline(liness, y);


      float x_point = strtof(x.c_str(),0);
      float y_point = strtof(y.c_str(),0);


      x_point = (float) initial_position.x + eyebrow_scaling*x_point;
      y_point = (float) initial_position.y + eyebrow_scaling*y_point;
      eyebrow_points.append(sf::Vertex(sf::Vector2f(x_point, y_point), eyebrow_colour));
  }
  file.close();

}



/*
callback functions
*/
void dynamicReconfigureCb(robot_faces::ParametersConfig &config, uint32_t level) {


  if(PRINT_DEBUG_MESSAGES) {
    ROS_INFO("Reconfigure Request");
  }


  irisShape = static_cast<IrisShape>(config.iris_shape);

  noseShape = static_cast<NoseShape>(config.nose_shape);

  eyebrowShape = static_cast<EyebrowShape>(config.eyebrow_shape);

  pupil_corner_radius = config.pupil_corner_radius;
  pupil_shape.setCornersRadius(pupil_corner_radius*pupil_radius/2.0f);

  iris_corner_radius = config.iris_corner_radius;
  iris_shape.setCornersRadius(iris_corner_radius*iris_diameter/2.0f);


  // positioning
  eye_spacing = config.eye_spacing;
  eye_height = config.eye_height;
  eyebrow_spacing = config.eyebrow_spacing;
  nose_height = config.nose_height;
  mouth_height = config.mouth_height;

  //NOTE REMOVE TEMPORARY DECLARATION
  int mouth_reference_x = int(0.5f*g_window_width);
  int mouth_reference_y = int(mouth_height*g_window_height);
  curr_mouth_points = {
     .upper_start = sf::Vector2f(mouth_reference_x-quarter_x, mouth_reference_y-10.0f),
     .upper_end = sf::Vector2f(mouth_reference_x+quarter_x, mouth_reference_y-10.0f),
     .upper_start_control = sf::Vector2f(mouth_reference_x-eight_x, mouth_reference_y-30.0f),
     .upper_end_control = sf::Vector2f(mouth_reference_x+eight_x, mouth_reference_y-30.0f),

     .lower_start = sf::Vector2f(mouth_reference_x-quarter_x, mouth_reference_y-10.0f),
     .lower_end = sf::Vector2f(mouth_reference_x+quarter_x, mouth_reference_y-10.0f),
     .lower_start_control = sf::Vector2f(mouth_reference_x-eight_x-20.0f, mouth_reference_y+quarter_y),
     .lower_end_control = sf::Vector2f(mouth_reference_x+eight_x+20.0f, mouth_reference_y+quarter_y)
 };

  // colours
  updateColour(background_colour, config.background_colour);

  updateColour(nose_colour, config.nose_colour);
  nose_annulus.setFillColor(nose_colour);
  left_nose_curve_fillet.setFillColor(nose_colour);
  right_nose_curve_fillet.setFillColor(nose_colour);

  updateColour(eyebrow_colour, config.eyebrow_colour);

  updateColour(iris_colour, config.iris_colour);
  iris_shape.setFillColor(iris_colour);

  updateColour(pupil_colour, config.pupil_colour);
  pupil_shape.setFillColor(pupil_colour);

  updateColour(mouth_colour, config.mouth_colour);
  mouth_fillet.setFillColor(mouth_colour);

  // display toggles
  show_eybrows = config.show_eybrows;
  show_iris = config.show_iris;
  show_pupil = config.show_pupil;
  show_nose = config.show_nose;
  show_mouth = config.show_mouth;

  // scaling
  nose_scaling = config.nose_scaling;
  eye_scaling_x = config.eye_scaling_x;
  eye_scaling_y = config.eye_scaling_y;
  eyebrow_scaling = config.eyebrow_scaling;
  mouth_scaling_x = config.mouth_scaling_x;
  mouth_scaling_y = config.mouth_scaling_y;

  // recompute vertexarray points
  generateNoseCurvePoints();
  generateNoseInvertedTrianglePoints();
  generateIrisPoints();
  generateEyebrowPoints();
}




/*
main
*/
int main(int argc, char **argv) {

  ros::init(argc, argv, "robot_face");


  dynamic_reconfigure::Server<robot_faces::ParametersConfig> server;
  dynamic_reconfigure::Server<robot_faces::ParametersConfig>::CallbackType f;

  f = boost::bind(&dynamicReconfigureCb, _1, _2);
  server.setCallback(f);


	std::random_device rd;
	eng.seed(rd());

	saccade_interval = saccade_time_dist(eng);

  sf::RenderWindow renderWindow(sf::VideoMode(g_window_width, g_window_height), "robot_face");

  // set the framerate to be the same as the monitor's refresh rate to reduce the change of adverse visual artifacts - tearing.
  renderWindow.setVerticalSyncEnabled(true);

  // sometimes vertical synchronistion is forced off by the graphic card so fall back to limiting the framerate to a reasonable figure.
  // renderWindow.setFramerateLimit(30);


  /*
  init
  */

  // nose
  nose_annulus.setRadius(nose_radius);
  nose_annulus.setOrigin(nose_radius, nose_radius);

  sf::Vector2f initial_position{int(0.5f*g_window_width), 0};

  generateNoseCurvePoints();

  generateNoseInvertedTrianglePoints();

  // pupil
  pupil_shape.setSize(sf::Vector2f(pupil_radius, pupil_radius));
  pupil_shape.setOrigin(pupil_radius/2.0f, pupil_radius/2.0f);
  pupil_shape.setCornersRadius(pupil_corner_radius* pupil_radius/2.0f);
  pupil_shape.setCornerPointCount(20);
  pupil_shape.setFillColor(pupil_colour);


  goal_pupil_pos = sf::Vector2f(0, 0);
  curr_pupil_pos = goal_pupil_pos;

  // iris
  iris_shape.setSize(sf::Vector2f(iris_diameter, iris_diameter));
  iris_shape.setOrigin(iris_diameter/2.0f, iris_diameter/2.0f);
  iris_shape.setCornersRadius(iris_corner_radius*iris_diameter/2.0f);
  iris_shape.setCornerPointCount(20);
  iris_shape.setFillColor(iris_colour);
  generateIrisPoints();

  curr_iris_pos = goal_pupil_pos;

  // eyebrows
  generateEyebrowPoints();

  // mouth
  mouth_fillet.setRadius(MOUTH_THICKNESS);
  mouth_fillet.setFillColor(mouth_colour);
  mouth_fillet.setOrigin(MOUTH_THICKNESS, MOUTH_THICKNESS);


  // eyelids
  top_eyelid.setSize(sf::Vector2f(g_window_width, 100*eye_scaling_y*2));
  top_eyelid.setFillColor(sf::Color(255,0,0,255));
  top_eyelid.setOrigin(g_window_width/2.0f, 100*eye_scaling_y);

  bottom_eyelid.setSize(sf::Vector2f(g_window_width, 100*eye_scaling_y*2));
  bottom_eyelid.setFillColor(sf::Color(255,255,0,255));
  bottom_eyelid.setOrigin(g_window_width/2.0f, 100*eye_scaling_y);


  //NOTE REMOVE TEMPORARY DECLARATION
  int mouth_reference_x = int(0.5f*g_window_width);
  int mouth_reference_y = int(mouth_height*g_window_height);
  curr_mouth_points = {
     .upper_start = sf::Vector2f(mouth_reference_x-quarter_x, mouth_reference_y-10.0f),
     .upper_end = sf::Vector2f(mouth_reference_x+quarter_x, mouth_reference_y-10.0f),
     .upper_start_control = sf::Vector2f(mouth_reference_x-eight_x, mouth_reference_y-30.0f),
     .upper_end_control = sf::Vector2f(mouth_reference_x+eight_x, mouth_reference_y-30.0f),

     .lower_start = sf::Vector2f(mouth_reference_x-quarter_x, mouth_reference_y-10.0f),
     .lower_end = sf::Vector2f(mouth_reference_x+quarter_x, mouth_reference_y-10.0f),
     .lower_start_control = sf::Vector2f(mouth_reference_x-eight_x-20.0f, mouth_reference_y+quarter_y),
     .lower_end_control = sf::Vector2f(mouth_reference_x+eight_x+20.0f, mouth_reference_y+quarter_y)
 };

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



    float frame_delta_time = frame_clock.getElapsedTime().asMilliseconds()/1000.0f; // time since last frame draw
    frame_clock.restart();


    if(saccade_clock.getElapsedTime().asMilliseconds() > saccade_interval) {

      if(PRINT_DEBUG_MESSAGES) {
        ROS_INFO("Perform saccade");
      }

      int delta_x = saccade_pos_dist(eng);
      int delta_y = saccade_pos_dist(eng);

      goal_pupil_pos = sf::Vector2f(delta_x, delta_y);


      saccade_interval = saccade_time_dist(eng);
      saccade_clock.restart();
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

    // interpolate between curr and goal positions
    if(getDistance(curr_pupil_pos, goal_pupil_pos) < CLOSE_ENOUGH_THRESHOLD) {
      curr_pupil_pos = goal_pupil_pos;
    } else {
      sf::Vector2f pupil_direction = normalize(goal_pupil_pos - curr_pupil_pos);
      curr_pupil_pos += frame_delta_time*SACCADE_SPEED*pupil_direction;
    }

    sf::Vector2f pupil_direction = normalize(goal_pupil_pos - curr_pupil_pos);
    curr_iris_pos += frame_delta_time*SACCADE_SPEED*pupil_direction*0.6f;




    renderWindow.clear(background_colour);


    /*
    RENDERING FACE
    */

    // iris
    if(show_iris) {

      if(irisShape==IrisShape::THICK || irisShape==IrisShape::OVAL || irisShape==IrisShape::ALMOND || irisShape==IrisShape::ARC) {
        sf::Transform t(1.f*eye_scaling_x, 0.f, left_eye_reference_x+curr_iris_pos.x,
                         0.f,  eye_scaling_y, eye_reference_y+curr_iris_pos.y,
                         0.f,  0.f, 1.f);
        renderWindow.draw(iris_points, t);

        t = sf::Transform(-1.f*eye_scaling_x, 0.f, right_eye_reference_x+curr_iris_pos.x,
                         0.f,  eye_scaling_y, eye_reference_y+curr_iris_pos.y,
                         0.f,  0.f, 1.f);

        renderWindow.draw(iris_points, t);

      } else {
        iris_shape.setScale(eye_scaling_x, eye_scaling_y);
        iris_shape.setPosition(left_eye_reference_x+curr_iris_pos.x, eye_reference_y+curr_iris_pos.y);
        renderWindow.draw(iris_shape);
        iris_shape.setPosition(right_eye_reference_x+curr_iris_pos.x, eye_reference_y+curr_iris_pos.y);
        renderWindow.draw(iris_shape);
      }

    }


    // pupils
    if(show_pupil) {

      pupil_shape.setPosition(left_eye_reference_x+curr_pupil_pos.x, eye_reference_y+curr_pupil_pos.y);
      renderWindow.draw(pupil_shape);
      pupil_shape.setPosition(right_eye_reference_x+curr_pupil_pos.x, eye_reference_y+curr_pupil_pos.y);
      renderWindow.draw(pupil_shape);
    }


    // eyelids
    top_eyelid.setPosition(g_window_width/2.0f, eye_reference_y+curr_pupil_pos.y-100-100*eye_scaling_y);
    renderWindow.draw(top_eyelid);

    bottom_eyelid.setPosition(g_window_width/2.0f, eye_reference_y+curr_pupil_pos.y+100+100*eye_scaling_y);
    renderWindow.draw(bottom_eyelid);



    // nose
    if(show_nose) {

      switch(noseShape) {

        case ANNULUS:
          nose_annulus.setPosition(nose_reference_x, nose_reference_y);
          nose_annulus.setScale(nose_scaling, nose_scaling);
          nose_annulus.setFillColor(nose_colour);
          renderWindow.draw(nose_annulus);

          nose_annulus.setScale(nose_scaling*0.7f, nose_scaling*0.7f);
          nose_annulus.setFillColor(background_colour);
          renderWindow.draw(nose_annulus);

        break;

        case BUTTON:
        default:
          nose_annulus.setPosition(nose_reference_x, nose_reference_y);
          nose_annulus.setFillColor(nose_colour);
          nose_annulus.setScale(nose_scaling, nose_scaling);
          renderWindow.draw(nose_annulus);

        break;


        case CURVE:
          {
            //TODO CHANGE SCALING TO HERE USING CUSTOM TRANSFORMATION MATRIX
            sf::Transform t;
            t.translate(nose_reference_x, nose_reference_y);
            renderWindow.draw(nose_curve_points, t);
            renderWindow.draw(left_nose_curve_fillet, t);
            renderWindow.draw(right_nose_curve_fillet, t);
          }
        break;

        case INVERTED_TRIANGLE:
          //TODO CHANGE SCALING TO HERE USING CUSTOM TRANSFORMATION MATRIX
          sf::Transform t;
          t.translate(nose_reference_x, nose_reference_y);
          renderWindow.draw(nose_inverted_triangle_points, t);
        break;
      }

    }


    if(show_mouth) {

      //NOTE THIS IS DONE EVERY FRAME, SHOULD FIND A WAY TO OPTIMISE
      upper_mouth_vertices = computeBezierCurve(curr_mouth_points.upper_start, curr_mouth_points.upper_end,
          curr_mouth_points.upper_start_control, curr_mouth_points.upper_end_control);

      lower_mouth_vertices = computeBezierCurve(curr_mouth_points.lower_start, curr_mouth_points.lower_end,
          curr_mouth_points.lower_start_control, curr_mouth_points.lower_end_control);


      renderWindow.draw(generateLineWThickness(upper_mouth_vertices, mouth_colour, MOUTH_THICKNESS));
      renderWindow.draw(generateLineWThickness(lower_mouth_vertices, mouth_colour, MOUTH_THICKNESS));
      mouth_fillet.setPosition(upper_mouth_vertices.front().x, upper_mouth_vertices.front().y);
      renderWindow.draw(mouth_fillet);
      mouth_fillet.setPosition(upper_mouth_vertices.back().x, upper_mouth_vertices.back().y);
      renderWindow.draw(mouth_fillet);
    }


    if(show_eybrows) {
      // offset for concave shapes
      float offset_x=0.0f; //90.0f for eyebrow_two
      if(eyebrowShape==EyebrowShape::RECTANGULAR) {
        offset_x=90.0f;
      }
      sf::Transform t(1.f, 0.f, left_eye_reference_x+offset_x,
                       0.f,  1.0f, eyebrow_reference_y,
                       0.f,  0.f, 1.f);
      renderWindow.draw(eyebrow_points, t);

      t = sf::Transform(-1.f, 0.f, right_eye_reference_x-offset_x,
                       0.f,  1.0f, eyebrow_reference_y,
                       0.f,  0.f, 1.f);

      renderWindow.draw(eyebrow_points, t);
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
