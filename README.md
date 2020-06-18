# robot_faces Robot Operating System package

robot_faces is a ROS package for rendering animated robot faces. Almost everything is parametised and can be changed through dynamic_recofigure to produce a variety of faces. All faces support several emotional expressions and actions such as speaking and blinking.

## Dependencies

We use SFML to render animations. It does not come with a FindSFML.cmake so I added one in the cmake/ directory.

## Parameters

The visual appearance of the face can be reconfigured and changed through parameters set through dynamic_recofigure. This can be done through the CLI, your own dynamic_recofigure client or the rqt_reconfigure GUI - which can be started by running `rosrun rqt_reconfigure rqt_reconfigure`.

Parameters are organised into groups such as positioning, scaling and behaviour.

### Positioning group


| Parameter name  | Description                                                    | default | min | max |
| --------------- | -------------------------------------------------------------- | ------- | --- | --- |
| eye_spacing     | Spacing between eyes as a percetange of width                  | 0.5     | 0.0 | 1.0 |
| eye_height      | Height of eyes from top as a percetange of height              | 0.25    | 0.0 | 1.0 |
| eyebrow_spacing | Spacing between eyes and eyebrows as a percetange of height    | 0.2     | 0.0 | 1.0 |
| nose_height     | Height of nose from top between eyes as a percetange of height | 0.5     | 0.0 | 1.0 |
| mouth_height    | Height of mouth from tops as a percetange of height            | 0.75    | 0.0 | 1.0 |

## API

The faces can be interacted with through ROS topics and services.

## TODO

* Github issue and feature request template.

* Anti-aliasing setting.

* Add image of possible faces to README.

## Ideas

* Include face parameter presets.

## Notes

Would like a way to parameterise the window dimensions but need to figure out a way to dynamically resize an SFML window.

## Authors

* Andrew Murtagh (murtagan@tcd.ie)

* More contributors are welcome.

## License

MIT licensed, see LICENSE file for more information.
