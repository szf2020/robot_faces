#ifndef NOSE_H
#define NOSE_H

/*
Copyright 2020 Andrew Murtagh

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <SFML/Graphics.hpp>


#include "element.h"

class Nose : public Element {

  public:
    // Nose(QString n, QDate df) : Company(n, df) { }

    Nose() :
      nose_curve_points_(sf::TrianglesStrip),
      nose_inverted_triangle_points_(sf::TrianglesFan),
      nose_shape_(NoseShape::BUTTON)
    {

      nose_annulus_.setRadius(DEFAULT_NOSE_RADIUS);
      nose_annulus_.setOrigin(DEFAULT_NOSE_RADIUS, DEFAULT_NOSE_RADIUS);
      //
      generateNoseCurvePoints();
      //
      generateNoseInvertedTrianglePoints();
    }

    enum struct NoseShape {
      ANNULUS,
      BUTTON,
      CURVE,
      INVERTED_TRIANGLE
    };

    void draw(sf::RenderWindow& renderWindow) override {



      if(show_) {
        //TODO background_colour, scale_x_
        sf::Color background_colour = sf::Color(255,255,255,255);

        switch(nose_shape_) {

          case NoseShape::ANNULUS:
            nose_annulus_.setPosition(reference_x_, reference_y_);
            nose_annulus_.setScale(scale_x_, scale_x_);
            nose_annulus_.setFillColor(colour_);
            renderWindow.draw(nose_annulus_);

            nose_annulus_.setScale(scale_x_*0.7f, scale_x_*0.7f);
            nose_annulus_.setFillColor(background_colour);
            renderWindow.draw(nose_annulus_);
          break;

          case NoseShape::BUTTON:
          default:
            nose_annulus_.setPosition(reference_x_, reference_y_);
            nose_annulus_.setFillColor(colour_);
            nose_annulus_.setScale(scale_x_, scale_x_);
            renderWindow.draw(nose_annulus_);

          break;

          case NoseShape::CURVE:
            {
              //TODO CHANGE SCALING TO HERE USING CUSTOM TRANSFORMATION MATRIX
              sf::Transform t;
              t.translate(reference_x_, reference_y_);
              renderWindow.draw(nose_curve_points_, t);
              renderWindow.draw(left_nose_curve_fillet_, t);
              renderWindow.draw(right_nose_curve_fillet_, t);
            }
          break;

          case NoseShape::INVERTED_TRIANGLE:
            //TODO CHANGE SCALING TO HERE USING CUSTOM TRANSFORMATION MATRIX
            sf::Transform t;
            t.translate(reference_x_, reference_y_);
            renderWindow.draw(nose_inverted_triangle_points_, t);
          break;
        }
      }

      //TODO ONLY IF DEBUG IS SET
      reference_marker_.setPosition(reference_x_, reference_y_);
      renderWindow.draw(reference_marker_);
    }


    void setShape(const NoseShape ns) {
      nose_shape_ = ns;
    }

    //TODO OVERRIDE BASE CLASSES TO CALL GENERATE NOSE POINTS
    void setScaleX(const float sx) {
      Element::setScaleX(sx);
      generateNoseCurvePoints();
      generateNoseInvertedTrianglePoints();
    }


  private:

    sf::CircleShape nose_annulus_;
    sf::CircleShape left_nose_curve_fillet_;
    sf::CircleShape right_nose_curve_fillet_;
    sf::VertexArray nose_curve_points_;
    sf::VertexArray nose_inverted_triangle_points_;
    NoseShape nose_shape_;


    const float DEFAULT_NOSE_CURVE_THICKNESS=5.0f;
    //TODO
    int g_window_width = 800;
    const int DEFAULT_NOSE_RADIUS = int(g_window_width/15.0f);


    void generateNoseCurvePoints() {

      //TODO scale_x_

      nose_curve_points_.clear();

      float thickness = scale_x_*DEFAULT_NOSE_CURVE_THICKNESS;
      float radius = scale_x_*DEFAULT_NOSE_RADIUS;
      sf::Vector2f initial_position{0, -radius/2.0f};

      sf::Vector2f prev_vector = sf::Vector2f(initial_position.x+radius*sin(degToRad(-30)), initial_position.y+radius*cos(degToRad(-30)));
      sf::Vector2f curr_vector = sf::Vector2f(initial_position.x+radius*sin(degToRad(-29)), initial_position.y+radius*cos(degToRad(-29)));


      for(int ang=-29; ang<=30; ang+=1) {

        curr_vector = sf::Vector2f(initial_position.x+radius*sin(degToRad(ang)), initial_position.y+radius*cos(degToRad(ang)));

        sf::Vector2f line = prev_vector - curr_vector;
        sf::Vector2f normal = normalize(sf::Vector2f(-line.y, line.x));

        nose_curve_points_.append(sf::Vertex(prev_vector - thickness * normal, colour_));
        nose_curve_points_.append(sf::Vertex(prev_vector + thickness * normal, colour_));

        nose_curve_points_.append(sf::Vertex(curr_vector - thickness * normal, colour_));
        nose_curve_points_.append(sf::Vertex(curr_vector + thickness * normal, colour_));

        prev_vector = curr_vector;
      }


    	left_nose_curve_fillet_.setRadius(thickness);
    	left_nose_curve_fillet_.setFillColor(colour_);
    	left_nose_curve_fillet_.setOrigin(thickness, thickness);
    	left_nose_curve_fillet_.setPosition(initial_position.x+radius*sin(degToRad(-30)), initial_position.y+radius*cos(degToRad(-30)));
    	right_nose_curve_fillet_.setRadius(thickness);
    	right_nose_curve_fillet_.setFillColor(colour_);
    	right_nose_curve_fillet_.setOrigin(thickness, thickness);
    	right_nose_curve_fillet_.setPosition(initial_position.x+radius*sin(degToRad(30)), initial_position.y+radius*cos(degToRad(30)));
    }




    void generateNoseInvertedTrianglePoints() {


      //TODO scale_x_

      nose_inverted_triangle_points_.clear();

      sf::Vector2f initial_position{0, 0}; //TODO REMOVE
      std::string package_path = ros::package::getPath("robot_faces");

      std::ifstream file(package_path+"/res/inverted_triangle_nose.txt", std::ifstream::in);
      std::string line, x, y;
      if(!getline(file, line)) {
        ROS_ERROR("Could not open file");
      }
      std::stringstream liness(line);

      nose_inverted_triangle_points_.append(sf::Vertex(sf::Vector2f(initial_position.x, initial_position.y), colour_));

      while (getline(file, line)) {
          std::stringstream liness(line);
          getline(liness, x, ',');
          getline(liness, y);

          // needs to be scaled
          float x_point = strtof(x.c_str(),0)*5.0f*scale_x_;
          float y_point = strtof(y.c_str(),0)*5.0f*scale_x_;

          x_point = (float) initial_position.x + x_point;
          y_point = (float) initial_position.y + y_point;
          nose_inverted_triangle_points_.append(sf::Vertex(sf::Vector2f(x_point, y_point), colour_));
      }
      file.close();

    }

};


#endif // NOSE_H
