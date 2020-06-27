#include <regex>

#include <ros/ros.h>

#include <SFML/Graphics.hpp>

std::regex rgba_regex("^(25[0-5]|2[0-4][0-9]|1[0-9]?[0-9]?|[1-9][0-9]?|[0-9]),?(25[0-5]|2[0-4][0-9]|1[0-9]?[0-9]?|[1-9][0-9]?|[0-9]),?(25[0-5]|2[0-4][0-9]|1[0-9]?[0-9]?|[1-9][0-9]?|[0-9]),?(25[0-5]|2[0-4][0-9]|1[0-9]?[0-9]?|[1-9][0-9]?|[0-9])$");


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

inline sf::Vector2f normalized(const sf::Vector2f v) {
	return v / getMagnitude(v);
}

inline float degToRad(const float deg) {
  return deg * M_PI / 180.0f;
}


sf::VertexArray generateLineWThickness(const std::vector<sf::Vector2f>& points, const sf::Color color, const float thickness) {
	sf::VertexArray array = sf::VertexArray(sf::TrianglesStrip);

  for (int i=0; i < points.size()-1; i++) {
    sf::Vector2f line = points[i] - points[i+1];
    sf::Vector2f normal = normalized(sf::Vector2f(-line.y, line.x));

		array.append(sf::Vertex(points[i] - thickness * normal, color));
		array.append(sf::Vertex(points[i] + thickness * normal, color));

		array.append(sf::Vertex(points[i+1] - thickness * normal, color));
		array.append(sf::Vertex(points[i+1] + thickness * normal, color));
	}

	return array;
}

std::vector<sf::Vector2f> computeBezierCurve(
        const sf::Vector2f& start,
        const sf::Vector2f& end,
        const sf::Vector2f& startControl,
        const sf::Vector2f& endControl) {

	const int NUM_SEGMENTS = 20;
	std::vector<sf::Vector2f> result;

	result.push_back(start);
	float p = 1.f / NUM_SEGMENTS;
	float q = p;
	for (size_t i = 1; i < NUM_SEGMENTS; i++, p += q) { // Generate all between
		result.push_back(p * p * p * (end + 3.f * (startControl - endControl) - start) + 3.f * p * p * (start - 2.f * startControl + endControl) + 3.f * p * (startControl - start) + start);
	}
	result.push_back(end);
	return result;
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
