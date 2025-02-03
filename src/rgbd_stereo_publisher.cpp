/*
MIT License

Copyright (c) 2024 José Miguel Guerrero Hernández

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>

#include "camera_info_manager/camera_info_manager.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

// Includes DepthAI libraries needed to work with the OAK-D device and pipeline
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"

std::vector<std::string> usbStrings = {"UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"};

// Function to configure and create the DepthAI pipeline
std::tuple<dai::Pipeline, int, int, int, int> createPipeline(
  bool lrcheck, bool extended, bool subpixel, int confidence, int LRchecktresh, bool use_depth,
  bool use_disparity, bool use_lr_raw)
{
  // Creates the processing pipeline
  dai::Pipeline pipeline;

  // Disables chunk size for XLink transfer
  pipeline.setXLinkChunkSize(0);

  // Sets the resolution for the mono cameras and RGB camera
  dai::node::MonoCamera::Properties::SensorResolution monoResolution =
    dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
  dai::ColorCameraProperties::SensorResolution colorResolution =
    dai::ColorCameraProperties::SensorResolution::THE_1080_P;
  int stereoWidth = 640, stereoHeight = 400, rgbWidth = 1920, rgbHeight = 1080;


  // ------------------------------
  // RGB Camera
  // ------------------------------

  // Creates node for RGB camera and XLink output connection
  std::shared_ptr<dai::node::ColorCamera> rgbCam = pipeline.create<dai::node::ColorCamera>();
  std::shared_ptr<dai::node::XLinkOut> xoutRgb = pipeline.create<dai::node::XLinkOut>();

  // Configures stream names for RGB output
  xoutRgb->setStreamName("rgb");

  // RGB camera configuration
  rgbCam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
  rgbCam->setResolution(colorResolution);
  rgbCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
  rgbCam->setFps(35);

  // Link RGB camera to its XLink output
  rgbCam->video.link(xoutRgb->input);


  // ------------------------------
  // Mono Cameras and Stereo Depth
  // ------------------------------

  std::shared_ptr<dai::node::MonoCamera> monoLeft, monoRight;
  std::shared_ptr<dai::node::XLinkOut> xoutLeft, xoutRight, xoutLeftRect, xoutRightRect;

  std::shared_ptr<dai::node::StereoDepth> stereo;
  std::shared_ptr<dai::node::XLinkOut> xoutDepth, xoutDepthDisp;

  if (use_depth || use_disparity || use_lr_raw) {
    // Creates nodes for left and right mono cameras and stereo depth
    monoLeft = pipeline.create<dai::node::MonoCamera>();  // Left mono camera
    monoRight = pipeline.create<dai::node::MonoCamera>(); // Right mono camera
    stereo = pipeline.create<dai::node::StereoDepth>();   // Stereo depth node

    // Creates XLink output connections for left and right rectified images
    xoutLeftRect = pipeline.create<dai::node::XLinkOut>();
    xoutRightRect = pipeline.create<dai::node::XLinkOut>();

    // Configures stream names for output connections
    xoutLeftRect->setStreamName("left_rect");
    xoutRightRect->setStreamName("right_rect");

    // Mono camera configuration (left and right)
    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B); // Sets camera socket
    monoLeft->setFps(120);   // Sets FPS
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C);
    monoRight->setFps(120);

    // Stereo depth node configuration
    //stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_5x5);
    //stereo->initialConfig.setConfidenceThreshold(confidence); // Confidence threshold
    stereo->setRectifyEdgeFillColor(0); // Black edge fill color for better cropping view
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setLeftRightCheck(lrcheck); // Enable/disable left-right check
    stereo->setExtendedDisparity(extended); // Enable extended disparity
    stereo->setSubpixel(subpixel); // Enable subpixel mode
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A); // Align depth to RGB camera

    // Link mono cameras to stereo depth node
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    // Link mono cameras to rectified outputs
    stereo->rectifiedLeft.link(xoutLeftRect->input);
    stereo->rectifiedRight.link(xoutRightRect->input);
  }

  // If using left and right raw images
  if (use_lr_raw) {
    // Creates XLink output connections for left and right raw images
    xoutLeft = pipeline.create<dai::node::XLinkOut>();
    xoutRight = pipeline.create<dai::node::XLinkOut>();

    // Configures stream names for output connections
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");

    // Link mono cameras to unrectified outputs (synced)
    stereo->syncedLeft.link(xoutLeft->input);
    stereo->syncedRight.link(xoutRight->input);

    std::cout << "Using left and right raw images" << std::endl;
  }

  // If using depth images
  if (use_depth) {
    // Creates XLink output connection for depth images
    xoutDepth = pipeline.create<dai::node::XLinkOut>();
    xoutDepth->setStreamName("depth");

    // Links stereo depth node to depth output
    stereo->depth.link(xoutDepth->input);
    std::cout << "Using depth images" << std::endl;
  }

  // If using disparity images
  if (use_disparity) {
    // Creates XLink output connection for disparity images
    xoutDepthDisp = pipeline.create<dai::node::XLinkOut>();
    xoutDepthDisp->setStreamName("disparity");

    // Links stereo depth node to disparity output
    stereo->disparity.link(xoutDepthDisp->input);

    std::cout << "Using disparity images" << std::endl;
  }

  // Returns the pipeline and configured dimensions
  return std::make_tuple(pipeline, stereoWidth, stereoHeight, rgbWidth, rgbHeight);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); // Initializes ROS
  auto node = rclcpp::Node::make_shared("rgbd_stereo_node"); // Creates a ROS node

  // Declare and get ROS parameters
  std::string tfPrefix;
  bool lrcheck, extended, subpixel, use_depth, use_disparity, use_lr_raw, pc_color, only_rgb,
    use_pointcloud;
  int confidence, LRchecktresh;
  int monoWidth, monoHeight, colorWidth, colorHeight;

  node->declare_parameter("tf_prefix", "oak");
  node->declare_parameter("lrcheck", true);
  node->declare_parameter("extended", false);
  node->declare_parameter("subpixel", false);
  node->declare_parameter("confidence", 200);
  node->declare_parameter("LRchecktresh", 5);
  node->declare_parameter("use_depth", true);
  node->declare_parameter("use_disparity", true);
  node->declare_parameter("use_lr_raw", true);
  node->declare_parameter("pc_color", true);
  node->declare_parameter("only_rgb", false);
  node->declare_parameter("use_pointcloud", true);

  node->get_parameter("tf_prefix", tfPrefix);
  node->get_parameter("lrcheck", lrcheck);
  node->get_parameter("extended", extended);
  node->get_parameter("subpixel", subpixel);
  node->get_parameter("confidence", confidence);
  node->get_parameter("LRchecktresh", LRchecktresh);
  node->get_parameter("use_depth", use_depth);
  node->get_parameter("use_disparity", use_disparity);
  node->get_parameter("use_lr_raw", use_lr_raw);
  node->get_parameter("pc_color", pc_color);
  node->get_parameter("only_rgb", only_rgb);
  node->get_parameter("use_pointcloud", use_pointcloud);

  if (only_rgb) {
    use_depth = false;
    use_disparity = false;
    use_lr_raw = false;
    use_pointcloud = false;
  }

  // Creates the pipeline with specified parameters
  dai::Pipeline pipeline;
  std::tie(pipeline, monoWidth, monoHeight, colorWidth, colorHeight) = createPipeline(
        lrcheck, extended, subpixel, confidence, LRchecktresh, use_depth, use_disparity,
    use_lr_raw);

  // Initialize DepthAI devices with configured pipeline
  auto deviceInfoVec = dai::Device::getAnyAvailableDevice();
  RCLCPP_INFO(node->get_logger(), "Camera detected: %d", std::get<0>(deviceInfoVec));
  
  // TODO: check if get<0> = true
  dai::Device device(pipeline, std::get<1>(deviceInfoVec));

  // Reads calibration data from the device
  auto calibrationHandler = device.readCalibration();

  // Show configuration
  RCLCPP_INFO(node->get_logger(), "-------------------------------");
  RCLCPP_INFO(node->get_logger(), "System Information:");
  RCLCPP_INFO(node->get_logger(), "- Device MxID : %s", device.getMxId().c_str());
  RCLCPP_INFO(node->get_logger(), "- Device USB status: %s",
      usbStrings[static_cast<int32_t>(device.getUsbSpeed())].c_str());

  RCLCPP_INFO(node->get_logger(), "- Color resolution: %dx%d", colorWidth, colorHeight);
  RCLCPP_INFO(node->get_logger(), "- RGB camera activated");

  if (use_lr_raw || use_depth || use_disparity) {
    RCLCPP_INFO(node->get_logger(), "- Mono resolution: %dx%d", monoWidth, monoHeight);
    RCLCPP_INFO(node->get_logger(), "- Stereo cameras activated");
  }

  if (use_lr_raw) {
    RCLCPP_INFO(node->get_logger(), "- Left and right raw images activated");
  }
  if (use_depth) {
    RCLCPP_INFO(node->get_logger(), "- Depth images activated");
  }
  if (use_disparity) {
    RCLCPP_INFO(node->get_logger(), "- Disparity images activated");
  }
  if (use_pointcloud) {
    if (pc_color) {
      RCLCPP_INFO(node->get_logger(), "- Pointcloud activated with color");
    } else {
      RCLCPP_INFO(node->get_logger(), "- Pointcloud activated with intensity");
    }
  }
  RCLCPP_INFO(node->get_logger(), "-------------------------------");

  // Creates publishers for RGB, left, right, left rectified, right rectified, depth, and disparity images
  std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image,
    dai::ImgFrame>> rgbPublish, leftPublish, rightPublish, leftRectPublish, rightRectPublish,
    depthPublish;
  std::unique_ptr<dai::rosBridge::BridgePublisher<stereo_msgs::msg::DisparityImage,
    dai::ImgFrame>> dispPublish;

  // Creates image converters for RGB, left, right, and depth images
  dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
  dai::rosBridge::ImageConverter leftConverter(tfPrefix + "_left_camera_optical_frame", true);
  dai::rosBridge::ImageConverter rightConverter(tfPrefix + "_right_camera_optical_frame", true);
  dai::rosBridge::ImageConverter depthConverter = pc_color ? rgbConverter : rightConverter;
  dai::rosBridge::DisparityConverter dispConverter(tfPrefix + "_right_camera_optical_frame", 880,
    7.5, 20, 2000);

  // Creates camera info for RGB, left, right, and depth images
  auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler,
    dai::CameraBoardSocket::CAM_A, colorWidth, colorHeight);
  auto leftCameraInfo = leftConverter.calibrationToCameraInfo(calibrationHandler,
    dai::CameraBoardSocket::CAM_B, monoWidth, monoHeight);
  auto rightCameraInfo = leftConverter.calibrationToCameraInfo(calibrationHandler,
    dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);
  auto depthCameraInfo = pc_color ? rgbCameraInfo : rightCameraInfo;

  // Converts and publishes RGB images to ROS
  auto rgbQueue = device.getOutputQueue("rgb", 30, false);
  rgbPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image,
      dai::ImgFrame>>(
    rgbQueue,
    node,
    std::string("rgb/image"),
    std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                    &rgbConverter,  // since the converter has the same frame name
                                    // and image type is also same we can reuse it
                    std::placeholders::_1,
                    std::placeholders::_2),
    30,
    rgbCameraInfo,
    "rgb");

  rgbPublish->addPublisherCallback(); // addPublisherCallback works only when the dataqueue is non blocking.

  if (use_lr_raw) {
    // Converts and publishes left and right raw images to ROS
    auto leftQueue = device.getOutputQueue("left", 30, false);
    leftPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image,
        dai::ImgFrame>>(
      leftQueue,
      node,
      std::string("left/image"),
      std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &leftConverter, std::placeholders::_1,
                std::placeholders::_2),
      30,
      leftCameraInfo,
      "left");

    leftPublish->addPublisherCallback();

    auto rightQueue = device.getOutputQueue("right", 30, false);
    rightPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image,
        dai::ImgFrame>>(
      rightQueue,
      node,
      std::string("right/image"),
      std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rightConverter, std::placeholders::_1,
                std::placeholders::_2),
      30,
      rightCameraInfo,
      "right");

    rightPublish->addPublisherCallback();
  }

  if (use_depth || use_disparity || use_lr_raw) { // Note: It is necessary to publish rectified images when publishing raw images
    // Converts and publishes rectified left and right images to ROS
    auto leftRectQueue = device.getOutputQueue("left_rect", 30, false);
    leftRectPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image,
        dai::ImgFrame>>(
      leftRectQueue,
      node,
      std::string("left_rect/image"),
      std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &leftConverter, std::placeholders::_1,
                std::placeholders::_2),
      30,
      leftCameraInfo,
      "left_rect");

    leftRectPublish->addPublisherCallback();

    auto rightRectQueue = device.getOutputQueue("right_rect", 30, false);
    rightRectPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image,
        dai::ImgFrame>>(
      rightRectQueue,
      node,
      std::string("right_rect/image"),
      std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rightConverter, std::placeholders::_1,
                std::placeholders::_2),
      30,
      rightCameraInfo,
      "right_rect");

    rightRectPublish->addPublisherCallback();
  }

  if (use_depth) {
    // Converts and publishes depth image to ROS
    auto stereoQueue = device.getOutputQueue("depth", 1000, false);
    depthPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image,
        dai::ImgFrame>>(
      stereoQueue,
      node,
      std::string("stereo/depth"),
      std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                      &depthConverter,  // since the converter has the same frame name
                                        // and image type is also same we can reuse it
                      std::placeholders::_1,
                      std::placeholders::_2),
      1000,
      depthCameraInfo,
      "stereo");

    depthPublish->addPublisherCallback();
  }

  if (use_disparity) {
    // Converts and publishes disparity image to ROS
    auto stereoQueueDisp = device.getOutputQueue("disparity", 1000, false);
    dispPublish = std::make_unique<dai::rosBridge::BridgePublisher<stereo_msgs::msg::DisparityImage,
        dai::ImgFrame>>(
      stereoQueueDisp,
      node,
      std::string("stereo/disparity"),
      std::bind(&dai::rosBridge::DisparityConverter::toRosMsg, &dispConverter,
                std::placeholders::_1, std::placeholders::_2),
      30,
      rightCameraInfo,
      "stereo");

    dispPublish->addPublisherCallback();
  }

  // Spins the node
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
  
}