# robot_faces Robot Operating System package

robot_faces is a ROS package for rendering animated robot faces. Almost everything is parametised and can be changed through dynamic_recofigure to produce a variety of faces. All faces support several emotional expressions and actions such as speaking and blinking.


## Dependencies

We use SFML to render animations. It does not come with a FindSFML.cmake so I added one in the cmake/ directory.

## Parameters

The visual appearance of the face can be reconfigured and changed through parameters set through dynamic_recofigure. This can be done through the CLI, your own dynamic_recofigure client or the rqt_reconfigure GUI - which can be started by running `rosrun rqt_reconfigure rqt_reconfigure`.

Parameters are organised into groups such as positioning, scaling and behaviour.

#### Misc

| Parameter name      | Description                                                       | default | min  | max   |
| ------------------- | ----------------------------------------------------------------- | ------- | ---- | ----- |
| pupil_corner_radius | Corner radius of pupil: 0.0 is a straight corner, 1.0 is a circle | 1.0     | 0.0  | 1.0   |
| iris_corner_radius  | Corner radius of iris: 0.0 is a straight corner, 1.0 is a circle  | 1.0     | 0.0  | 1.0   |
| eyebrow_shape       | Enum for the eyebrow shape                                        | arc     |      |       |
| iris_shape          | Enum for the iris shape (rounded rect, thick, oval, almond, arc)  | circle  |      |       |
| nose_shape          | Enum describing the nose (annulus, button, curve and dog)         | button  |      |       |
| avr_blink_interval  | Average ms between blinks                                         | 3000    | 1000 | 20000 |
| will_blink          | Whether or not to blink                                           | True    |      |       |
| will_do_saccades    | Whether or not the eyes will do saccades                          | True    |      |       |



#### Positioning parameter group

| Parameter name  | Description                                                    | default | min | max |
| --------------- | -------------------------------------------------------------- | ------- | --- | --- |
| eye_spacing     | Spacing between eyes as a percetange of width                  | 0.5     | 0.0 | 1.0 |
| eye_height      | Height of eyes from top as a percetange of height              | 0.25    | 0.0 | 1.0 |
| eyebrow_spacing | Spacing between eyes and eyebrows as a percetange of height    | 0.2     | 0.0 | 1.0 |
| nose_height     | Height of nose from top between eyes as a percetange of height | 0.5     | 0.0 | 1.0 |
| mouth_height    | Height of mouth from tops as a percetange of height            | 0.75    | 0.0 | 1.0 |


#### Colours parameter group

| Parameter name    | Description                                   | default           |
| ----------------- | --------------------------------------------- | ----------------- |
| background_colour | Comma delimited RGBA string of the background | "255,255,255,255" |
| nose_colour       | Comma delimited RGBA string of the nose       | "41,41,41,255"    |
| pupil_colour      | Comma delimited RGBA string of the pupil      | "0,0,0,255"       |
| iris_colour       | Comma delimited RGBA string of the iris       | "139,69,19,255"   |
| eyebrow_colour    | Comma delimited RGBA string of the eyebrow    | "34,27,7,255"     |
| mouth_colour      | Comma delimited RGBA string of the mouth      | "0,0,0,255"     |


#### Display toggles parameter group

| Parameter name | Description                    | default |
| -------------- | ------------------------------ | ------- |
| show_eybrows   | Whether to render the eyebrows | True    |
| show_iris      | Whether to render the iris     | True    |
| show_pupil     | Whether to render the pupil    | True    |
| show_nose      | Whether to render the nose     | False   |
| show_mouth     | Whether to render the mouth    | True    |

### Scaling parameter group

| Parameter name  | Description             | default | min | max  |
| --------------- | ----------------------- | ------- | --- | ---- |
| nose_scaling    | Scaling of the nose     | 1.0     | 0.0 | 10.0 |
| eye_scaling_x   | X-scaling of the eyes   | 1.0     | 0.0 | 10.0 |
| eye_scaling_y   | Y-scaling of the eyes   | 1.0     | 0.0 | 10.0 |
| eyebrow_scaling | Scaling of the eyebrows | 1.0     | 0.0 | 10.0 |
| mouth_scaling_x | X-scaling of the mouth  | 1.0     | 0.0 | 10.0 |
| mouth_scaling_y | Y-scaling of the mouth  | 1.0     | 0.0 | 10.0 |


## API

The faces can be interacted with through ROS topics and services.

## TODO

* Github issue and feature request template.

* Anti-aliasing setting.

* Add image of possible faces and explain bezier points to README.

* Add tests for regex validation of colour parameters.

* Add light highlight to nose and iris.

* Make nose a rounded rectangle.

* Add other functions to utility file.

* Mouth: scaling in x and y, closed or open mouth.

* Bugfix -pupils moving outside of iris

* Pupil scaling

* adjust corner radius of pupil highlight to match the iris.

## Ideas

* Include face parameter presets.

* Nonlinear easing and interpolation.

* Different eyelid shapes

## Notes

Would like a way to parameterise the window dimensions but need to figure out a way to dynamically resize an SFML window.

## Authors

* Andrew Murtagh (murtagan@tcd.ie)

* More contributors are welcome.

## License

MIT licensed, see LICENSE file for more information.
