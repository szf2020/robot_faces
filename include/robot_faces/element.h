#ifndef ELEMENT_H
#define ELEMENT_H

/*
Copyright 2020 Andrew Murtagh

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <regex>
#include <SFML/Graphics.hpp>

std::regex rgba_regex("^(25[0-5]|2[0-4][0-9]|1[0-9]?[0-9]?|[1-9][0-9]?|[0-9]),?(25[0-5]|2[0-4][0-9]|1[0-9]?[0-9]?|[1-9][0-9]?|[0-9]),?(25[0-5]|2[0-4][0-9]|1[0-9]?[0-9]?|[1-9][0-9]?|[0-9]),?(25[0-5]|2[0-4][0-9]|1[0-9]?[0-9]?|[1-9][0-9]?|[0-9])$");


class Element {

  public:

    Element() :
      scale_x_(1.0f),
      scale_y_(1.0f),
      show_(true),
      reference_x_(0),
      reference_y_(0),
      colour_(0,0,0,255),
      REFERENCE_MARKER_COLOUR_(0, 0, 255, 255)
    {

      //TODO SET TO DEFAULT COLOUR
      reference_marker_.setRadius(REFERENCE_MARKER_RADIUS_);
      reference_marker_.setOrigin(REFERENCE_MARKER_RADIUS_, REFERENCE_MARKER_RADIUS_);
      reference_marker_.setFillColor(REFERENCE_MARKER_COLOUR_);
    }

    virtual void draw(sf::RenderWindow&, const float) = 0;

    void setScaleX(const float sx) {
      //TODO BOUNDS CHECKING
      scale_x_ = sx;
    }

    void setScaleY(const float sy) {
      //TODO BOUNDS CHECKING
      scale_y_ = sy;
    }

    void setShow(const bool s) {
      show_ = s;
    }

    void setReferenceX(const int x) {
      //TODO BOUNDS CHECKING
      reference_x_ = x;
    }

    void setReferenceY(const int y) {
      //TODO BOUNDS CHECKING
      reference_y_ = y;
    }

    void setPosition(const int x, const int y) {
      //TODO BOUNDS CHECKING
      reference_x_ = x;
      reference_y_ = y;
    }

    void setColour(const std::string& c) {
      // TODO CHECKING VALIDITY
      // colour_ = c;
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
    }

  protected:
    float scale_x_;
    float scale_y_;
    bool show_;
    int reference_x_;
    int reference_y_;
    sf::Color colour_;

    sf::CircleShape reference_marker_;
    const sf::Color REFERENCE_MARKER_COLOUR_;
    const int REFERENCE_MARKER_RADIUS_ = 5;
};


#endif // ELEMENT_H
