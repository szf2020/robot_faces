#ifndef MOUTH_H
#define MOUTH_H

/*
Copyright 2020 Andrew Murtagh

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <SFML/Graphics.hpp>


#include "element.h"

class Mouth : public Element {

  public:

    Mouth() {

      mouth_fillet_.setRadius(MOUTH_THICKNESS_);
      mouth_fillet_.setFillColor(colour_);
      mouth_fillet_.setOrigin(MOUTH_THICKNESS_, MOUTH_THICKNESS_);

      goal_mouth_points_ = neutral_mouth_bezier_points;
      curr_mouth_points_ = goal_mouth_points_;
    }


    void draw(sf::RenderWindow& renderWindow, const float frame_delta_time) override {


      MouthBezierPoints translated_points_ = curr_mouth_points_.transform(reference_x_, reference_y_, scale_x_, scale_y_);

      //TODO GLOBAL VARIABLE DUPLICATION.. FIX THIS
      const float EXPRESSION_SPEED = 140.0f; //px per second
      const float CLOSE_ENOUGH_THRESHOLD = 3.0f; // in px

      if(show_) {

        if(curr_mouth_points_.closeEnough(goal_mouth_points_, CLOSE_ENOUGH_THRESHOLD)) {
          curr_mouth_points_ = goal_mouth_points_;
        } else {
          curr_mouth_points_.moveTowards(goal_mouth_points_, frame_delta_time, EXPRESSION_SPEED);
        }



        //NOTE THIS IS DONE EVERY FRAME, SHOULD FIND A WAY TO OPTIMISE
        // upper_mouth_vertices = computeBezierCurve(translated_points.upper_start, translated_points.upper_end,
            // translated_points.upper_start_control, translated_points.upper_end_control);

        // lower_mouth_vertices = computeBezierCurve(translated_points.lower_start, translated_points.lower_end,
            // translated_points.lower_start_control, translated_points.lower_end_control);


        upper_mouth_vertices_ = translated_points_.generateCurve(0);
        lower_mouth_vertices_ = translated_points_.generateCurve(1);


        renderWindow.draw(generateLineWThickness(upper_mouth_vertices_, colour_, MOUTH_THICKNESS_));
        renderWindow.draw(generateLineWThickness(lower_mouth_vertices_, colour_, MOUTH_THICKNESS_));
        mouth_fillet_.setPosition(upper_mouth_vertices_.front().x, upper_mouth_vertices_.front().y);
        renderWindow.draw(mouth_fillet_);
        mouth_fillet_.setPosition(upper_mouth_vertices_.back().x, upper_mouth_vertices_.back().y);
        renderWindow.draw(mouth_fillet_);
      }

      //TODO ONLY IF DEBUG IS SET
      reference_marker_.setPosition(reference_x_, reference_y_);
      renderWindow.draw(reference_marker_);


      // MouthBezierPoints translated_points = curr_mouth_points.transform(reference_x_, mouth_reference_y, mouth_scaling_x, mouth_scaling_y);
      /*
      bezier_marker_.setPosition(translated_points_.upper_start.x, translated_points_.upper_start.y);
      renderWindow.draw(bezier_marker_);

      bezier_marker_.setPosition(translated_points_.upper_end.x, translated_points_.upper_end.y);
      renderWindow.draw(bezier_marker_);

      bezier_marker_.setPosition(translated_points_.upper_start_control.x, translated_points_.upper_start_control.y);
      renderWindow.draw(bezier_marker_);

      bezier_marker_.setPosition(translated_points_.upper_end_control.x, translated_points_.upper_end_control.y);
      renderWindow.draw(bezier_marker_);

      bezier_marker_.setPosition(translated_points_.lower_start.x, translated_points_.lower_start.y);
      renderWindow.draw(bezier_marker_);

      bezier_marker_.setPosition(translated_points_.lower_end.x, translated_points_.lower_end.y);
      renderWindow.draw(bezier_marker_);

      bezier_marker_.setPosition(translated_points_.lower_start_control.x, translated_points_.lower_start_control.y);
      renderWindow.draw(bezier_marker_);

      bezier_marker_.setPosition(translated_points_.lower_end_control.x, translated_points_.lower_end_control.y);
      renderWindow.draw(bezier_marker_);
      */
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
      mouth_fillet_.setFillColor(colour_);

    }

    void setGoalPoints(MouthBezierPoints mbp) {
      goal_mouth_points_ = mbp;
    }



  private:

    const int MOUTH_THICKNESS_ = 8.0f; //note this is actually half the thickness, make a parameter

    std::vector<sf::Vector2f> upper_mouth_vertices_;
    std::vector<sf::Vector2f>  lower_mouth_vertices_;
    sf::CircleShape mouth_fillet_;
    MouthBezierPoints goal_mouth_points_;
    MouthBezierPoints curr_mouth_points_;



    sf::CircleShape bezier_marker_;

};


#endif // MOUTH_H
