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

// Function to configure and create the DepthAI pipeline
std::tuple<dai::Pipeline, int, int, int, int> createPipeline(
  bool withDepth, bool lrcheck, bool extended, bool subpixel, int confidence, int LRchecktresh)
{

  dai::Pipeline pipeline; // Creates the processing pipeline
  pipeline.setXLinkChunkSize(0); // Disables chunk size for XLink transfer
  dai::node::MonoCamera::Properties::SensorResolution monoResolution =
    dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
  dai::ColorCameraProperties::SensorResolution colorResolution =
    dai::ColorCameraProperties::SensorResolution::THE_1080_P;
  int stereoWidth = 640, stereoHeight = 480, rgbWidth = 1920, rgbHeight = 1080;

// Creates nodes for mono cameras, RGB, and XLink output connections
  auto monoLeft = pipeline.create<dai::node::MonoCamera>();
  auto monoRight = pipeline.create<dai::node::MonoCamera>();
  auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
  auto xoutRight = pipeline.create<dai::node::XLinkOut>();
  auto xoutLeftRect = pipeline.create<dai::node::XLinkOut>();
  auto xoutRightRect = pipeline.create<dai::node::XLinkOut>();
  auto stereo = pipeline.create<dai::node::StereoDepth>();
  auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
  auto rgbCam = pipeline.create<dai::node::ColorCamera>();
  auto xoutRgb = pipeline.create<dai::node::XLinkOut>();

// Configures stream names for outputs
  xoutLeft->setStreamName("left");
  xoutRight->setStreamName("right");
  xoutRgb->setStreamName("rgb");
  xoutLeftRect->setStreamName("left_rect");
  xoutRightRect->setStreamName("right_rect");

  if(withDepth) {
    xoutDepth->setStreamName("depth"); // Sets depth output stream
  } else {
    xoutDepth->setStreamName("disparity"); // Sets disparity output stream
  }

  // Mono camera configuration (left and right)
  monoLeft->setResolution(monoResolution);
  monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B); // Sets camera socket
  monoLeft->setFps(40);   // Sets FPS
  monoRight->setResolution(monoResolution);
  monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C);
  monoRight->setFps(40);

  // Stereo depth node configuration
  stereo->initialConfig.setConfidenceThreshold(confidence); // Confidence threshold
  stereo->setRectifyEdgeFillColor(0); // Black edge fill color for better cropping view
  stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
  stereo->setLeftRightCheck(lrcheck); // Enable/disable left-right check
  stereo->setExtendedDisparity(extended); // Enable extended disparity
  stereo->setSubpixel(subpixel); // Enable subpixel mode
  if(withDepth) {stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);}

  // RGB camera configuration
  rgbCam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
  rgbCam->setResolution(colorResolution);
  rgbCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
  rgbCam->setFps(35);

  // Link mono cameras to stereo depth node
  monoLeft->out.link(stereo->left);
  monoRight->out.link(stereo->right);

  // Link RGB camera to its XLink output
  rgbCam->video.link(xoutRgb->input);

  // Link mono cameras to both rectified and unrectified outputs
  stereo->syncedLeft.link(xoutLeft->input);
  stereo->syncedRight.link(xoutRight->input);
  stereo->rectifiedLeft.link(xoutLeftRect->input);
  stereo->rectifiedRight.link(xoutRightRect->input);

  // Set depth or disparity output depending on mode
  if(withDepth) {
    stereo->depth.link(xoutDepth->input);
  } else {
    stereo->disparity.link(xoutDepth->input);
  }

  // Returns the pipeline and configured dimensions
  return std::make_tuple(pipeline, stereoWidth, stereoHeight, rgbWidth, rgbHeight);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); // Initializes ROS
  auto node = rclcpp::Node::make_shared("rgbd_stereo_node"); // Creates a ROS node

  // Declare and get ROS parameters
  std::string tfPrefix, mode;
  bool lrcheck, extended, subpixel, enableDepth;
  int confidence, LRchecktresh;
  int monoWidth, monoHeight, colorWidth, colorHeight;
  dai::Pipeline pipeline;

  node->declare_parameter("tf_prefix", "oak");
  node->declare_parameter("mode", "depth");
  node->declare_parameter("lrcheck", true);
  node->declare_parameter("extended", false);
  node->declare_parameter("subpixel", true);
  node->declare_parameter("confidence", 200);
  node->declare_parameter("LRchecktresh", 5);

  node->get_parameter("tf_prefix", tfPrefix);
  node->get_parameter("mode", mode);
  node->get_parameter("lrcheck", lrcheck);
  node->get_parameter("extended", extended);
  node->get_parameter("subpixel", subpixel);
  node->get_parameter("confidence", confidence);
  node->get_parameter("LRchecktresh", LRchecktresh);

  // Enable depth or disparity mode based on `mode` parameter
  if(mode == "depth") {
    enableDepth = true;
  } else {
    enableDepth = false;
  }

  // Creates the pipeline with specified parameters
  std::tie(pipeline, monoWidth, monoHeight, colorWidth, colorHeight) = createPipeline(
        enableDepth, lrcheck, extended, subpixel, confidence, LRchecktresh);

  // Initialize DepthAI device with configured pipeline
  dai::Device device(pipeline);
  auto leftQueue = device.getOutputQueue("left", 30, false);
  auto rightQueue = device.getOutputQueue("right", 30, false);
  auto rgbQueue = device.getOutputQueue("rgb", 30, false);
  auto leftRectQueue = device.getOutputQueue("left_rect", 30, false);
  auto rightRectQueue = device.getOutputQueue("right_rect", 30, false);

  // Output queue for depth or disparity, depending on mode
  std::shared_ptr<dai::DataOutputQueue> stereoQueue;
  if(enableDepth) {
    stereoQueue = device.getOutputQueue("depth", 30, false);
  } else {
    stereoQueue = device.getOutputQueue("disparity", 30, false);
  }

  // Converts and publishes image data to ROS
  auto calibrationHandler = device.readCalibration();
  dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
  auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler,
    dai::CameraBoardSocket::CAM_A, colorWidth, colorHeight);
  dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(
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

  rgbPublish.addPublisherCallback(); // addPublisherCallback works only when the dataqueue is non blocking.

  dai::rosBridge::ImageConverter leftconverter(tfPrefix + "_left_camera_optical_frame", true);
  auto leftCameraInfo = leftconverter.calibrationToCameraInfo(calibrationHandler,
    dai::CameraBoardSocket::CAM_B, monoWidth, monoHeight);
  dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> leftPublish(
    leftQueue,
    node,
    std::string("left/image"),
    std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &leftconverter, std::placeholders::_1,
              std::placeholders::_2),
    30,
    leftCameraInfo,
    "left");

  leftPublish.addPublisherCallback();

  dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
  auto rightCameraInfo = leftconverter.calibrationToCameraInfo(calibrationHandler,
    dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);

  dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(
    rightQueue,
    node,
    std::string("right/image"),
    std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rightconverter, std::placeholders::_1,
              std::placeholders::_2),
    30,
    rightCameraInfo,
    "right");

  rightPublish.addPublisherCallback();

  dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> leftRectPublish(
    leftRectQueue,
    node,
    std::string("left_rect/image"),
    std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &leftconverter, std::placeholders::_1,
              std::placeholders::_2),
    30,
    leftCameraInfo,
    "left_rect");

  leftRectPublish.addPublisherCallback();

  dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightRectPublish(
    rightRectQueue,
    node,
    std::string("right_rect/image"),
    std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rightconverter, std::placeholders::_1,
              std::placeholders::_2),
    30,
    rightCameraInfo,
    "right_rect");

  rightRectPublish.addPublisherCallback();

  // Converts and publishes depth or disparity images to ROS
  if(mode == "depth") {
    auto depthCameraInfo = enableDepth ? rgbCameraInfo : rightCameraInfo;
    auto depthconverter = enableDepth ? rgbConverter : rightconverter;
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(
      stereoQueue,
      node,
      std::string("stereo/depth"),
      std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                      &depthconverter,  // since the converter has the same frame name
                                        // and image type is also same we can reuse it
                      std::placeholders::_1,
                      std::placeholders::_2),
      30,
      depthCameraInfo,
      "stereo");
    depthPublish.addPublisherCallback();
    rclcpp::spin(node);
  } else {
    dai::rosBridge::DisparityConverter dispConverter(tfPrefix + "_right_camera_optical_frame", 880,
      7.5, 20, 2000);
    dai::rosBridge::BridgePublisher<stereo_msgs::msg::DisparityImage, dai::ImgFrame> dispPublish(
      stereoQueue,
      node,
      std::string("stereo/disparity"),
      std::bind(&dai::rosBridge::DisparityConverter::toRosMsg, &dispConverter,
                std::placeholders::_1, std::placeholders::_2),
      30,
      rightCameraInfo,
      "stereo");
    dispPublish.addPublisherCallback();
    rclcpp::spin(node);
  }

  return 0;
}
