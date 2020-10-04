# visy_detector_pkg

ROS package for vision system (visy) to detect conveyor system and metal chips.

![CI](https://github.com/deltarobotone/visy_detector_pkg/workflows/CI/badge.svg?branch=master) [![Codacy Badge](https://app.codacy.com/project/badge/Grade/f52da64a87c54a66a092bf198cd6c83d)](https://www.codacy.com/gh/deltarobotone/visy_detector_pkg?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=deltarobotone/visy_detector_pkg&amp;utm_campaign=Badge_Grade)

## Nodes

### conveyor_detector_node

ROS node detects conveyor system via image processing using openCV3.

#### Published topics

##### conveyor_system_rect (visy_detector_pkg/ConveyorSystem)

Publish detected rectangle of conveyor system. Corners of rectangle via points (x,y) in geometry message.

##### visy_image (sensor_msgs/CompressedImage)

Publish image at the end of a detection loop including rectange for conveor system if it is detected.

#### Advertised Action

##### detect_conveyor (visy_detector_pkg/DetectConveyor)

Action to start conveyor system detector. Non blocking solution for conveyor system detector (uses internal blocking while loop). Result could be true or false. Feedback while detecting is a loop counter. The rectangle with detected corners of conveyor system will be published after this action is finished.

#### Service Clients

##### /status_bar_node/pixel_ctrl (visy_neopixel_pkg/PixelCtrl)

Control statusbar of visy to inform user about the detection state. Green for detected. Yellow for not detected.

#### Subscribed topics

##### /raspicam_node/image (sensor_msgs/CompressedImage)

Get actual image from visy camera (raspicam) using raspicam node.

### metal_chip_detector_node

ROS node detects metal chips on conveyor system via image processing using openCV3. Detected metalchip data like colour, position and timestamps are used by other nodes like grasp planner.

#### Advertised Services

##### select_image (visy_detector_pkg/SelectImage)

Select image at processing state to provide insights at hsi, chroma and threshold states.

##### start_metalchip_detector (visy_detector_pkg/StartMetalChipDetector)

Starts metal chip detector. Requires detected conveyor system.

##### stop_metalchip_detector (visy_detector_pkg/StopMetalChipDetector)

Stops metal chip detector.

#### Published topics

##### metal_chip (visy_detector_pkg/MetalChip)

Publish date of detected metal chip including colour as hue value, position (pixel), image timestamp and timestamp after image processing.

##### visy_image (sensor_msgs/CompressedImage)

Publish image at the end of a detection loop. Image could be changed in diffenent image processing states via service.

#### Service Clients

##### /status_bar_node/light_ctrl (visy_neopixel_pkg/LightCtrl)

Control statusbar of visy to inform user about the detection state. Spinning light for an active detector or full light using detected colour.

#### Subscribed topics

##### /raspicam_node/image (sensor_msgs/CompressedImage)

Get actual image from visy camera (raspicam) using raspicam node.

##### conveyor_system_rect (visy_detector_pkg/ConveyorSystem)

Get detected rectangle of conveyor system from conveyor detector node.

