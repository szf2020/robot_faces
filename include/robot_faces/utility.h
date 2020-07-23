#ifndef UTILITY_H
#define UTILITY_H

/*
Copyright 2020 Andrew Murtagh

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <regex>

#include <ros/ros.h>

#include <SFML/Graphics.hpp>


std::regex temprgba_regex("^(25[0-5]|2[0-4][0-9]|1[0-9]?[0-9]?|[1-9][0-9]?|[0-9]),?(25[0-5]|2[0-4][0-9]|1[0-9]?[0-9]?|[1-9][0-9]?|[0-9]),?(25[0-5]|2[0-4][0-9]|1[0-9]?[0-9]?|[1-9][0-9]?|[0-9]),?(25[0-5]|2[0-4][0-9]|1[0-9]?[0-9]?|[1-9][0-9]?|[0-9])$");


inline float getDistance(const sf::Vector2f one, const sf::Vector2f two) {
 return sqrt(pow(two.x - one.x, 2) + pow(two.y - one.y, 2));
}

inline float getMagnitude(const sf::Vector2f v) {
	return (float) sqrt(v.x * v.x + v.y * v.y);
}

inline float dot(const sf::Vector2f v0, const sf::Vector2f v1) {
	return v0.x * v1.x + v0.y * v1.y;
}

inline sf::Vector2f getNormal(const sf::Vector2f v) {
	return sf::Vector2f(-v.y, v.y);
}

inline sf::Vector2f normalize(const sf::Vector2f v) {
	float mag = getMagnitude(v);
	return mag !=0 ? v / mag : sf::Vector2f(0,0);
}


inline float degToRad(const float deg) {
  return deg * M_PI / 180.0f;
}

sf::VertexArray generateLineWThickness(const std::vector<sf::Vector2f>& points, const sf::Color color, const float thickness) {
	sf::VertexArray array = sf::VertexArray(sf::TrianglesStrip);

  for (int i=0; i < points.size()-1; i++) {
    sf::Vector2f line = points[i] - points[i+1];
    sf::Vector2f normal = normalize(sf::Vector2f(-line.y, line.x));

		array.append(sf::Vertex(points[i] - thickness * normal, color));
		array.append(sf::Vertex(points[i] + thickness * normal, color));

		array.append(sf::Vertex(points[i+1] - thickness * normal, color));
		array.append(sf::Vertex(points[i+1] + thickness * normal, color));
	}

	return array;
}



void updateColour(sf::Color& colour, const std::string& new_colour) {

  if(std::regex_match(new_colour, temprgba_regex)) {

    std::stringstream ss(new_colour);

    std::string substr;
    int r=255, g=255, b=255, a=255;

    if(ss.good())
      std::getline(ss, substr, ',');
    r = std::stoi(substr);

    if(ss.good())
      std::getline(ss, substr, ',');
    g = std::stoi(substr);

    if(ss.good())
      std::getline(ss, substr, ',');
    b = std::stoi(substr);

    if(ss.good())
      std::getline(ss, substr, ',');
    a = std::stoi(substr);

    colour = sf::Color(r,g,b,a);

    // ROS_INFO((std::to_string(r)+", "+std::to_string(g)+", "+std::to_string(b)+", "+std::to_string(a)).c_str());

  } else {
    ROS_ERROR("colour parameter is not formatted incorrectly.");
  }
}


/*
data structures
*/
struct MouthBezierPoints {
    sf::Vector2f upper_start, upper_end, upper_start_control, upper_end_control;
    sf::Vector2f lower_start, lower_end, lower_start_control, lower_end_control;

    bool operator==(const MouthBezierPoints &) const;
    bool closeEnough(const MouthBezierPoints &, const float) const;
    MouthBezierPoints transform(const int, const int, const float, const float);
    void moveTowards(const MouthBezierPoints&, const float, const float);
    // void setPoint(const std::string& key, const std::string& x, const std::string& y);
    std::vector<sf::Vector2f> generateCurve(int) const;
};


std::vector<sf::Vector2f> MouthBezierPoints::generateCurve(int u) const {

  //TODO PUT TRANSFORM IN THIS FUNCTION
  // TODO MAKE THE SELECTOR AN ENUM INSTEAD OF INT
  //TODO MAKE THIS A STATIC VARIABLE
  const int NUM_SEGMENTS = 20;
  std::vector<sf::Vector2f> result;

  if(u==0) {
    result.push_back(upper_start);
    float p = 1.f / NUM_SEGMENTS;
    float q = p;
    for (size_t i = 1; i < NUM_SEGMENTS; i++, p += q) { // Generate all between
      result.push_back(p * p * p * (upper_end + 3.f * (upper_start_control - upper_end_control) - upper_start) + 3.f * p * p * (upper_start - 2.f * upper_start_control + upper_end_control) + 3.f * p * (upper_start_control - upper_start) + upper_start);
    }
    result.push_back(upper_end);
  } else {

    result.push_back(lower_start);
    float p = 1.f / NUM_SEGMENTS;
    float q = p;
    for (size_t i = 1; i < NUM_SEGMENTS; i++, p += q) { // Generate all between
      result.push_back(p * p * p * (lower_end + 3.f * (lower_start_control - lower_end_control) - lower_start) + 3.f * p * p * (lower_start - 2.f * lower_start_control + lower_end_control) + 3.f * p * (lower_start_control - lower_start) + lower_start);
    }
    result.push_back(lower_end);
  }
	return result;
}



bool MouthBezierPoints::operator== (const MouthBezierPoints &other_point) const {
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


/*
void MouthBezierPoints::setPoint(const std::string& key, const std::string& x, const std::string& y) {
    if (key == "upper_start") upper_start = sf::Vector2f(strtof(x.c_str(),0), strtof(y.c_str(),0)); return;
    if (key == "upper_end") upper_end = sf::Vector2f(strtof(x.c_str(),0), strtof(y.c_str(),0)); return;
    if (key == "upper_start_control") upper_start_control = sf::Vector2f(strtof(x.c_str(),0), strtof(y.c_str(),0)); return;
    if (key == "upper_end_control") upper_end_control = sf::Vector2f(strtof(x.c_str(),0), strtof(y.c_str(),0)); return;
    if (key == "lower_start") lower_start = sf::Vector2f(strtof(x.c_str(),0), strtof(y.c_str(),0)); return;
    if (key == "lower_end") lower_end = sf::Vector2f(strtof(x.c_str(),0), strtof(y.c_str(),0)); return;
    if (key == "lower_start_control") lower_start_control = sf::Vector2f(strtof(x.c_str(),0), strtof(y.c_str(),0)); return;
    if (key == "lower_end_control") lower_end_control = sf::Vector2f(strtof(x.c_str(),0), strtof(y.c_str(),0)); return;
}
*/


bool MouthBezierPoints::closeEnough(const MouthBezierPoints &other_point, const float threshold) const {

    if(getDistance(upper_start, other_point.upper_start) <= threshold &&
        getDistance(upper_end, other_point.upper_end) <= threshold &&
        getDistance(upper_start_control, other_point.upper_start_control) <= threshold &&
        getDistance(upper_end_control, other_point.upper_end_control) <= threshold &&

        getDistance(lower_start, other_point.lower_start) <= threshold &&
        getDistance(lower_end, other_point.lower_end) <= threshold &&
        getDistance(lower_start_control, other_point.lower_start_control) <= threshold &&
        getDistance(lower_end_control, other_point.lower_end_control) <= threshold) {
        return true;
    } else {
        return false;
    }
}

MouthBezierPoints MouthBezierPoints::transform(const int offset_x, const int offset_y, const float scale_x, const float scale_y) {
    return {
      .upper_start = sf::Vector2f(scale_x*upper_start.x+offset_x, scale_y*upper_start.y+offset_y),
      .upper_end = sf::Vector2f(scale_x*upper_end.x+offset_x, scale_y*upper_end.y+offset_y),
      .upper_start_control = sf::Vector2f(scale_x*upper_start_control.x+offset_x, scale_y*upper_start_control.y+offset_y),
      .upper_end_control = sf::Vector2f(scale_x*upper_end_control.x+offset_x, scale_y*upper_end_control.y+offset_y),

      .lower_start = sf::Vector2f(scale_x*lower_start.x+offset_x, scale_y*lower_start.y+offset_y),
      .lower_end = sf::Vector2f(scale_x*lower_end.x+offset_x, scale_y*lower_end.y+offset_y),
      .lower_start_control = sf::Vector2f(scale_x*lower_start_control.x+offset_x, scale_y*lower_start_control.y+offset_y),
      .lower_end_control = sf::Vector2f(scale_x*lower_end_control.x+offset_x, scale_y*lower_end_control.y+offset_y)
    };
}

void MouthBezierPoints::moveTowards(const MouthBezierPoints &goal_point, const float frame_delta_time, const float speed) {
  sf::Vector2f direction = normalize(goal_point.upper_start - upper_start);
  upper_start += frame_delta_time*speed*direction*0.2f;

  direction = normalize(goal_point.upper_end - upper_end);
  upper_end += frame_delta_time*speed*direction*0.2f;

  direction = normalize(goal_point.upper_start_control - upper_start_control);
  upper_start_control += frame_delta_time*speed*direction*0.2f;

  direction = normalize(goal_point.upper_end_control - upper_end_control);
  upper_end_control += frame_delta_time*speed*direction*0.2f;

  direction = normalize(goal_point.lower_start - lower_start);
  lower_start += frame_delta_time*speed*direction*0.2f;

  direction = normalize(goal_point.lower_end - lower_end);
  lower_end += frame_delta_time*speed*direction*0.2f;

  direction = normalize(goal_point.lower_start_control - lower_start_control);
  lower_start_control += frame_delta_time*speed*direction*0.2f;

  direction = normalize(goal_point.lower_end_control - lower_end_control);
  lower_end_control += frame_delta_time*speed*direction*0.2f;


}



MouthBezierPoints neutral_mouth_bezier_points = {
  .upper_start = sf::Vector2f(-100, 0),
  .upper_end = sf::Vector2f(100, 0),
  .upper_start_control = sf::Vector2f(0, 0),
  .upper_end_control = sf::Vector2f(0, 0),

  .lower_start = sf::Vector2f(-100, 0),
  .lower_end = sf::Vector2f(100, 0),
  .lower_start_control = sf::Vector2f(0, 0),
  .lower_end_control = sf::Vector2f(0, 0)
};


MouthBezierPoints sadness_mouth_bezier_points = {
  .upper_start = sf::Vector2f(-100, 40),
  .upper_end = sf::Vector2f(100, 40),
  .upper_start_control = sf::Vector2f(-30, 0),
  .upper_end_control = sf::Vector2f(30, 0),

  .lower_start = sf::Vector2f(-100, 40),
  .lower_end = sf::Vector2f(100, 40),
  .lower_start_control = sf::Vector2f(-30, 0),
  .lower_end_control = sf::Vector2f(30, 0)
};

MouthBezierPoints fear_mouth_bezier_points = {
  .upper_start = sf::Vector2f(-100, 40),
  .upper_end = sf::Vector2f(100, 40),
  .upper_start_control = sf::Vector2f(-30, -10),
  .upper_end_control = sf::Vector2f(30, -10),

  .lower_start = sf::Vector2f(-100, 40),
  .lower_end = sf::Vector2f(100, 40),
  .lower_start_control = sf::Vector2f(-30, 30),
  .lower_end_control = sf::Vector2f(30, 30)
};

MouthBezierPoints disgust_mouth_bezier_points = {
  .upper_start = sf::Vector2f(-100, 0),
  .upper_end = sf::Vector2f(100, 0),
  .upper_start_control = sf::Vector2f(-30, -30),
  .upper_end_control = sf::Vector2f(30, 30),

  .lower_start = sf::Vector2f(-100, 0),
  .lower_end = sf::Vector2f(100, 0),
  .lower_start_control = sf::Vector2f(-30, -30),
  .lower_end_control = sf::Vector2f(30, 30)
};

MouthBezierPoints anger_mouth_bezier_points = {
  .upper_start = sf::Vector2f(-100, 40),
  .upper_end = sf::Vector2f(100, 40),
  .upper_start_control = sf::Vector2f(-30, -10),
  .upper_end_control = sf::Vector2f(30, -10),

  .lower_start = sf::Vector2f(-100, 40),
  .lower_end = sf::Vector2f(100, 40),
  .lower_start_control = sf::Vector2f(-30, 30),
  .lower_end_control = sf::Vector2f(30, 30)
};

MouthBezierPoints joy_mouth_bezier_points = {
  .upper_start = sf::Vector2f(-100, 0),
  .upper_end = sf::Vector2f(100, 0),
  .upper_start_control = sf::Vector2f(-30, 20),
  .upper_end_control = sf::Vector2f(30, 20),

  .lower_start = sf::Vector2f(-100, 0),
  .lower_end = sf::Vector2f(100, 0),
  .lower_start_control = sf::Vector2f(-30, 20),
  .lower_end_control = sf::Vector2f(30, 20)
};

MouthBezierPoints happiness_mouth_bezier_points = {
  .upper_start = sf::Vector2f(-100, -30),
  .upper_end = sf::Vector2f(100, -30),
  .upper_start_control = sf::Vector2f(-20, -10),
  .upper_end_control = sf::Vector2f(20, -10),

  .lower_start = sf::Vector2f(-100, -30),
  .lower_end = sf::Vector2f(100, -30),
  .lower_start_control = sf::Vector2f(-20, 30),
  .lower_end_control = sf::Vector2f(20, 30)
};

MouthBezierPoints awe_mouth_bezier_points = {
  .upper_start = sf::Vector2f(-100, -10),
  .upper_end = sf::Vector2f(100, -10),
  .upper_start_control = sf::Vector2f(-20, -15),
  .upper_end_control = sf::Vector2f(20, -15),

  .lower_start = sf::Vector2f(-100, -10),
  .lower_end = sf::Vector2f(100, -10),
  .lower_start_control = sf::Vector2f(-20, 30),
  .lower_end_control = sf::Vector2f(20, 30)
};

MouthBezierPoints surprise_mouth_bezier_points = {
  .upper_start = sf::Vector2f(-60, 0),
  .upper_end = sf::Vector2f(60, 0),
  .upper_start_control = sf::Vector2f(-40, -80),
  .upper_end_control = sf::Vector2f(40, -80),

  .lower_start = sf::Vector2f(-60, 0),
  .lower_end = sf::Vector2f(60, 0),
  .lower_start_control = sf::Vector2f(-40, 80),
  .lower_end_control = sf::Vector2f(40, 80)
};

#endif // UTILITY_H
