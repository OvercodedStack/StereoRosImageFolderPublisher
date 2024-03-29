# Stereo ROS Image Publisher

This node publishes all the images contained in an specified folder, in the specified topic.
The images will be published in BGR8 format.

## Setup

1. Clone this repo `git clone https://github.com/amc-nu/RosImageFolderPublisher`
1. Change to repo dir `cd StereoRosImageFolderPublisher`
1. Execute `catkin_make`
1. Source the workspace `source devel/setup.bash`

If you want to integrate this into another catkin workspace, just copy the `image_folder_publisher` into the `src` target workspace.

## How to Launch

Once in a sourced terminal, execute:   
`rosrun image_folder_publisher image_folder_publisher.py`   
or   
`roslaunch image_folder_publisher publisher.launch`   

## Params

|Param name    | Type   | Description                                             | Default Value |
|------------- |--------|-------------------------------------------------------  |---------------|
|`topic_name_left`  | String | Name of the topic to publish the left image stream.           | `image_raw_left`   |
|`topic_name_right`  | String | Name of the topic to publish the right image stream.           | `image_raw_right`   |
|`publish_rate`| Integer| Frame rate in Hz to publish the image.                  | `10`          |
|`sort_files`  | Boolean| Defines if the files will be sorted before publishing.   | `True`        |
|`simulate_camera`  | Boolean| Simulates the camera intrinsic values and camera info as a real stereo camera.   | `True`        |
|`stereo_baseline`  | Boolean| Sets the stereo baseline of the camera.   | `True`        |
|`simulated_camera_fov`  | float| Sets the FOV of the inputted images. Used to calculate camera intrisics.   | `90.0`        |
|`frame_id`    | String | Sets the frame_id_left contained in the Image message header | `/ispies/stereo_camera`      |
|`image_folder_left`| String | Path to the folder containing the images to be published|               | 
|`image_folder_right`| String | Path to the folder containing the images to be published|               | 
|`sleep`       | Int    | Sleep few seconds to make sure the images process nodes are started|`0` | 


Images are outputted to *<topic_name_left>/image_rect*.

Camera info are outputted to *<topic_name_left>/camera_info*.



### Example

`rosrun image_folder_publisher image_folder_publisher.py image_folder_left:=/PATH/Images_left image_folder_right:=/PATH/Images_right _topic_name_left:=/image_topic_left _topic_name_right:=/image_topic_right`
or modify these in `publisher.launch`

## Notes

* Invalid files will be skipped. The supported image formats will correspond to the ones supported by CvBridge and OpenCV.
* Publish Rate will depend on the speed of the Hard disk. 

## Acknowledgements 

This repository is a fork from the original repository at: https://github.com/amc-nu/RosImageFolderPublisher


