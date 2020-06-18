#include <regex>

#include <ros/ros.h>

#include <SFML/Graphics.hpp>

std::regex rgba_regex("^(25[0-5]|2[0-4][0-9]|1[0-9]?[0-9]?|[1-9][0-9]?|[0-9]),?(25[0-5]|2[0-4][0-9]|1[0-9]?[0-9]?|[1-9][0-9]?|[0-9]),?(25[0-5]|2[0-4][0-9]|1[0-9]?[0-9]?|[1-9][0-9]?|[0-9]),?(25[0-5]|2[0-4][0-9]|1[0-9]?[0-9]?|[1-9][0-9]?|[0-9])$");

inline float getMagnitude(sf::Vector2f v) {
	return (float) sqrt(v.x * v.x + v.y * v.y);
}

inline float dot(sf::Vector2f v0, sf::Vector2f v1) {
	return v0.x * v1.x + v0.y * v1.y;
}

inline sf::Vector2f getNormal(sf::Vector2f v) {
	return sf::Vector2f(-v.y, v.y);
}

inline sf::Vector2f normalized(sf::Vector2f v) {
	return v / getMagnitude(v);
}

inline float degToRad(float deg) {
  return deg * M_PI / 180.0f;
}


void updateColour(sf::Color& colour, const std::string& new_colour) {

  if(std::regex_match(new_colour, rgba_regex)) {

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
