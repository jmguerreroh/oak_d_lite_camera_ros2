#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>

#include "camera_info_manager/camera_info_manager.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"

std::vector<std::string> usbStrings = {"UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"};

std::tuple<dai::Pipeline, int, int> createPipeline(
    bool withDepth, bool lrcheck, bool extended, bool subpixel, int confidence, int LRchecktresh, std::string resolution) {
    dai::Pipeline pipeline;
    pipeline.setXLinkChunkSize(0);
    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();

    // XLinkOut

    if(withDepth) {
        xoutDepth->setStreamName("depth");
    } else {
        xoutDepth->setStreamName("disparity");
    }

    int width, height;
    if(resolution == "720p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
        width = 1280;
        height = 720;
    } else if(resolution == "400p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
        width = 640;
        height = 400;
    } else if(resolution == "800p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
        width = 1280;
        height = 800;
    } else if(resolution == "480p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
        width = 640;
        height = 480;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid parameter. -> monoResolution: %s", resolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    // MonoCamera
    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C);

    // StereoDepth
    stereo->initialConfig.setConfidenceThreshold(confidence);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);
    monoLeft->setFps(120);
    monoRight->setFps(120);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    if(withDepth) {
        stereo->depth.link(xoutDepth->input);
    } else {
        stereo->disparity.link(xoutDepth->input);
    }

    return std::make_tuple(pipeline, width, height);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("stereo_node");

    std::string tfPrefix, mode, monoResolution, cam_id;
    bool lrcheck, extended, subpixel, enableDepth;
    int confidence, LRchecktresh;
    int monoWidth, monoHeight;
    dai::Pipeline pipeline;

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("mode", "depth");
    node->declare_parameter("lrcheck", true);
    node->declare_parameter("extended", false);
    node->declare_parameter("subpixel", true);
    node->declare_parameter("confidence", 200);
    node->declare_parameter("LRchecktresh", 5);
    node->declare_parameter("monoResolution", "400p");
    node->declare_parameter("cam_id", "False");


    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("mode", mode);
    node->get_parameter("lrcheck", lrcheck);
    node->get_parameter("extended", extended);
    node->get_parameter("subpixel", subpixel);
    node->get_parameter("confidence", confidence);
    node->get_parameter("LRchecktresh", LRchecktresh);
    node->get_parameter("monoResolution", monoResolution);
    node->get_parameter("cam_id", cam_id);

    if(mode == "depth") {
        enableDepth = true;
    } else {
        enableDepth = false;
    }

    std::tie(pipeline, monoWidth, monoHeight) = createPipeline(enableDepth, lrcheck, extended, subpixel, confidence, LRchecktresh, monoResolution);
    std::tuple<bool, dai::DeviceInfo> deviceInfoVec;
    
    if(cam_id == std::string("f")){
        deviceInfoVec = dai::Device::getAnyAvailableDevice();
    }else{
        deviceInfoVec = dai::Device::getDeviceByMxId(cam_id) ;
    }
    dai::Device device(pipeline, std::get<1>(deviceInfoVec));

    // Show configuration
    RCLCPP_INFO(node->get_logger(), "-------------------------------");
    RCLCPP_INFO(node->get_logger(), "System Information:");
    RCLCPP_INFO(node->get_logger(), "- Device MxID : %s", device.getMxId().c_str());
    RCLCPP_INFO(node->get_logger(), "- Device USB status: %s",
        usbStrings[static_cast<int32_t>(device.getUsbSpeed())].c_str());

    RCLCPP_INFO(node->get_logger(), "- Color resolution: %dx%d", monoWidth, monoHeight);
    RCLCPP_INFO(node->get_logger(), "- RGB camera activated");
    RCLCPP_INFO(node->get_logger(), "-------------------------------");

    std::shared_ptr<dai::DataOutputQueue> stereoQueue;
    if(enableDepth) {
        stereoQueue = device.getOutputQueue("depth", 240, false);
    } else {
        stereoQueue = device.getOutputQueue("disparity", 240, false);
    }

    auto calibrationHandler = device.readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    if(monoHeight > 480 && boardName == "OAK-D-LITE") {
        monoWidth = 640;
        monoHeight = 480;
    }

    dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
    auto rightCameraInfo = rightconverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);

    if(mode == "depth") {
        dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(
            stereoQueue,
            node,
            std::string("stereo/depth"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                      &rightconverter,  // since the converter has the same frame name
                                        // and image type is also same we can reuse it
                      std::placeholders::_1,
                      std::placeholders::_2),
            240,
            rightCameraInfo,
            "stereo");
        depthPublish.addPublisherCallback();
        rclcpp::spin(node);
    } else {
        dai::rosBridge::DisparityConverter dispConverter(tfPrefix + "_right_camera_optical_frame", 880, 7.5, 20, 2000);
        dai::rosBridge::BridgePublisher<stereo_msgs::msg::DisparityImage, dai::ImgFrame> dispPublish(
            stereoQueue,
            node,
            std::string("stereo/disparity"),
            std::bind(&dai::rosBridge::DisparityConverter::toRosMsg, &dispConverter, std::placeholders::_1, std::placeholders::_2),
            240,
            rightCameraInfo,
            "stereo");
        dispPublish.addPublisherCallback();
        
        rclcpp::Rate rate(240);
        while (rclcpp::ok()) {
            rclcpp::spin_some(node->get_node_base_interface());
            rate.sleep();
        }
    }

    return 0;
}