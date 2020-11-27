#include <iostream>

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#include "spdlog/spdlog.h"

#include "Renderer/Renderer.h"

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

int main(int argc, char* argv[]) {
    spdlog::set_level(spdlog::level::debug);

    ros::init(argc, argv, "Simulation");
    ros::NodeHandle n;

    unsigned int count = 0;
    ros::Publisher cameraInfoPublisher = n.advertise<sensor_msgs::CameraInfo>("/camera/color/camera_info", 1000);
    ros::Publisher rgbPublisher = n.advertise<sensor_msgs::Image>("/camera/color/image_raw", 1000);
    ros::Publisher depthPublisher = n.advertise<sensor_msgs::Image>("/camera/aligned_depth_to_color/image_raw", 1000);

    Renderer renderer("Simulation", 500, 500);
    Shader shader("basic");
    renderer.loadObject("living_room");


    while(renderer.isRunning() && ros::ok()) {
        auto [rgb, depth] = renderer.render(shader);
        std::transform(depth.begin(), depth.end(), depth.begin(), [&renderer](float depth) {
            // Use ROS REP 118 special meanings
            if(depth == 1.0) return INFINITY;
            if(depth == 0.0) return -INFINITY;
            // Convert nonlinear depth buffer to distance values
            float zNear = renderer.getZNear();
            float zFar = renderer.getZFar();
            return (zNear * zFar) / (zFar - depth * (zFar - zNear));
        });

        std_msgs::Header header;
        header.seq = count;
        header.stamp = ros::Time::now();
        header.frame_id = "camera_link";

        sensor_msgs::CameraInfo cameraInfo;
        cameraInfo.header = header;
        cameraInfo.width = renderer.getWidth();
        cameraInfo.height = renderer.getHeight();
        cameraInfo.D = {0, 0, 0, 0, 0};
        cameraInfo.K = {
            renderer.getWidth() / 2.0 / glm::tan(renderer.getFOV() / 2.0), 0, renderer.getWidth() / 2.0,
            0, renderer.getHeight() / 2.0 / glm::tan(renderer.getFOV() / 2.0), renderer.getHeight() / 2.0,
            0, 0, 1,
        };
        cameraInfo.R = {
            1, 0, 0,
            0, 1, 0,
            0, 0, 1,
        };
        cameraInfo.P = {
            cameraInfo.K[0], cameraInfo.K[1], cameraInfo.K[2], 0,
            cameraInfo.K[3], cameraInfo.K[4], cameraInfo.K[5], 0,
            cameraInfo.K[6], cameraInfo.K[7], cameraInfo.K[8], 0,
        };
        cameraInfoPublisher.publish(cameraInfo);

        sensor_msgs::Image rgbImage;
        rgbImage.header = header;
        rgbImage.width = renderer.getWidth();
        rgbImage.height = renderer.getHeight();
        rgbImage.encoding = "rgb8";
        rgbImage.is_bigendian = true;
        rgbImage.step = 3 * sizeof(std::uint8_t) * renderer.getWidth();
        rgbImage.data = rgb;
        rgbPublisher.publish(rgbImage);

        sensor_msgs::Image depthImage;
        depthImage.header = header;
        depthImage.width = renderer.getWidth();
        depthImage.height = renderer.getHeight();
        depthImage.encoding = "32FC1";
        depthImage.is_bigendian = false;
        depthImage.step = sizeof(float) * renderer.getWidth();
        depthImage.data = std::vector<std::uint8_t>(
            reinterpret_cast<std::uint8_t*>(depth.data()),
            reinterpret_cast<std::uint8_t*>(depth.data() + depth.size())
        );
        depthPublisher.publish(depthImage);

        count++;

        renderer.update();
    }
    return 0;
}
