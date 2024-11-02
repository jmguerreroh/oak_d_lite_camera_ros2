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
  bool lrcheck, bool extended, bool subpixel, int confidence, int LRchecktresh)
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
  auto xoutDepthDisp = pipeline.create<dai::node::XLinkOut>();
  auto rgbCam = pipeline.create<dai::node::ColorCamera>();
  auto xoutRgb = pipeline.create<dai::node::XLinkOut>();

  // Configures stream names for outputs
  xoutLeft->setStreamName("left");
  xoutRight->setStreamName("right");
  xoutRgb->setStreamName("rgb");
  xoutLeftRect->setStreamName("left_rect");
  xoutRightRect->setStreamName("right_rect");
  xoutDepth->setStreamName("depth");
  xoutDepthDisp->setStreamName("disparity");

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
  stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A); // Align depth to RGB camera

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
  stereo->depth.link(xoutDepth->input);
  stereo->disparity.link(xoutDepthDisp->input);

  // Returns the pipeline and configured dimensions
  return std::make_tuple(pipeline, stereoWidth, stereoHeight, rgbWidth, rgbHeight);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); // Initializes ROS
  auto node = rclcpp::Node::make_shared("rgbd_stereo_node"); // Creates a ROS node

  // Declare and get ROS parameters
  std::string tfPrefix;
  bool lrcheck, extended, subpixel, use_depth, use_disparity, use_lr_raw, pc_color;
  int confidence, LRchecktresh;
  int monoWidth, monoHeight, colorWidth, colorHeight;
  dai::Pipeline pipeline;

  node->declare_parameter("tf_prefix", "oak");
  node->declare_parameter("lrcheck", true);
  node->declare_parameter("extended", false);
  node->declare_parameter("subpixel", true);
  node->declare_parameter("confidence", 200);
  node->declare_parameter("LRchecktresh", 5);
  node->declare_parameter("use_depth", true);
  node->declare_parameter("use_disparity", true);
  node->declare_parameter("use_lr_raw", true);
  node->declare_parameter("pc_color", true);

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

  // Creates the pipeline with specified parameters
  std::tie(pipeline, monoWidth, monoHeight, colorWidth, colorHeight) = createPipeline(
        lrcheck, extended, subpixel, confidence, LRchecktresh);

  // Initialize DepthAI device with configured pipeline
  dai::Device device(pipeline);
  auto calibrationHandler = device.readCalibration();
  RCLCPP_INFO(node->get_logger(), "Device USB status: %s",
    usbStrings[static_cast<int32_t>(device.getUsbSpeed())].c_str());

  // Creates output queues for left, right, left rectified, right rectified, depth, and disparity images
  auto leftQueue = device.getOutputQueue("left", 30, false);
  auto rightQueue = device.getOutputQueue("right", 30, false);
  auto leftRectQueue = device.getOutputQueue("left_rect", 30, false);
  auto rightRectQueue = device.getOutputQueue("right_rect", 30, false);
  auto stereoQueue = device.getOutputQueue("depth", 30, false);
  auto stereoQueueDisp = device.getOutputQueue("disparity", 30, false);

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
    // Converts and publishes left and right images to ROS
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

  if (use_depth || use_disparity) {
    // Converts and publishes rectified left and right images to ROS
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
      30,
      depthCameraInfo,
      "stereo");

    depthPublish->addPublisherCallback();
  }

  if (use_disparity) {
    // Converts and publishes disparity image to ROS
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
