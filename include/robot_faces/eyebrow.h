#ifndef EYEBROW_H
#define EYEBROW_H

/*
Copyright 2020 Andrew Murtagh

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <SFML/Graphics.hpp>


#include "element.h"

class Eyebrow : public Element {

  public:
    Eyebrow(LeftOrRight lr) :
      left_or_right_(lr),
      eyebrow_points_(sf::TrianglesFan),
      eyebrow_shape_(EyebrowShape::CIRCULAR_ARC)
    {
      generateEyebrowPoints();
    }

    enum struct EyebrowShape {
      CIRCULAR_ARC,
      RECTANGULAR,
      SQUARE,
      ROUNDED,
      STRAIGHT,
      HIGH_ARCH
    };


    void draw(sf::RenderWindow& renderWindow, const float frame_delta_time) override {

      if(show_) {
        // offset for concave shapes
        float offset_x=0.0f;
        if(eyebrow_shape_==EyebrowShape::RECTANGULAR) {
          offset_x=90.0f;
        }

        //TODO
        float left_eye_reference_x = 200;
        float right_eye_reference_x = 400;
        float eyebrow_reference_y = 200;

        if(left_or_right_==LEFT) {
          sf::Transform t(1.f, 0.f, reference_x_+offset_x,
                           0.f,  1.0f, reference_y_,
                           0.f,  0.f, 1.f);
          renderWindow.draw(eyebrow_points_, t);
        } else {
          sf::Transform t(-1.f, 0.f, reference_x_-offset_x,
                           0.f,  1.0f, reference_y_,
                           0.f,  0.f, 1.f);

          renderWindow.draw(eyebrow_points_, t);
        }


      }

      //TODO ONLY IF DEBUG IS SET
      reference_marker_.setPosition(reference_x_, reference_y_);
      renderWindow.draw(reference_marker_);
    }


    void setShape(const EyebrowShape es) {
      eyebrow_shape_ = es;
      generateEyebrowPoints();
    }

    void setScaleX(const float sx) {
      Element::setScaleX(sx);
      generateEyebrowPoints();
    }


    void setColour(const std::string& c) {
      //TODO CODE DUPLICATION. FIX THS. OVERRIDE element.setColour with sf::Color and validate here then call base class with validated sf::Color
            if(std::regex_match(c, rgba_regex)) {

        std::stringstream ss(c);

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

        colour_ = sf::Color(r,g,b,a);

        // ROS_INFO((std::to_string(r)+", "+std::to_string(g)+", "+std::to_string(b)+", "+std::to_string(a)).c_str());

      } else {
        ROS_ERROR("colour parameter is not formatted incorrectly.");
      }

      Element::setColour(c);
      generateEyebrowPoints();

    }


  private:


    LeftOrRight left_or_right_;
    sf::VertexArray eyebrow_points_;
    EyebrowShape eyebrow_shape_;


    void generateEyebrowPoints() {

      std::string filename = "eyebrow_arc";

      switch(eyebrow_shape_) {
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


      eyebrow_points_.clear();
      sf::Vector2f initial_position{0, 0}; //TODO REMOVE
      std::string package_path = ros::package::getPath("robot_faces");

      std::ifstream file(package_path+"/res/"+filename+".txt", std::ifstream::in);

      std::string line, x, y;
      if(!getline(file, line)) {
        ROS_ERROR("Could not open file");
      }
      std::stringstream liness(line);

      eyebrow_points_.append(sf::Vertex(sf::Vector2f(initial_position.x, initial_position.y), colour_));

      while (getline(file, line)) {
          std::stringstream liness(line);
          getline(liness, x, ',');
          getline(liness, y);


          float x_point = strtof(x.c_str(),0);
          float y_point = strtof(y.c_str(),0);


          x_point = (float) initial_position.x + scale_x_*x_point;
          y_point = (float) initial_position.y + scale_x_*y_point;
          eyebrow_points_.append(sf::Vertex(sf::Vector2f(x_point, y_point), colour_));
      }
      file.close();

    }


};


#endif // EYEBROW_H
