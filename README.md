# robot_faces Robot Operating System package

robot_faces is a ROS package for rendering animated robot faces. Almost everything is parametised and can be changed through dynamic_recofigure to produce a variety of faces. All faces support several emotional expressions and actions such as speaking and blinking.

## Dependencies

We use SFML to render animations. It does not come with a FindSFML.cmake so I added one in the camke/ directory.

# Parameters

The visual appearance of the face can be reconfigured and changed through parameters set through dynamic_recofigure. This can be done through the CLI, your own dynamic_recofigure client or the rqt_reconfigure GUI - which can be started by running `rosrun rqt_reconfigure rqt_reconfigure`.

Parameters are organised into groups such as positioning, scaling and behaviour.

| Parameter name | Description |
| -------------- | ----------- |
| iris_shape     | TODO |
| iris_colour    | TODO |
| eyebrow_shape  | TODO |
| mouth_shape    | TODO |
| mouth_scaling  | TODO |
| window_width   | TODO |
| window_height  | TODO |


# API

The faces can be interacted with through ROS topics and services.

# TODO

* Github issue and feature request template.

* Anti-aliasing setting.

## Ideas

* Include face parameter presets.

## Authors

* Andrew Murtagh (murtagan@tcd.ie)

* More contributors are welcome.

## License

MIT licensed, see LICENSE file for more information.
