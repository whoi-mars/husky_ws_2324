/*
 * MIT License
 * 
 * Copyright (c) 2020 Stereolabs
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "zed_rgb_convert_component.hpp"

#include <opencv2/opencv.hpp>

#include <sensor_msgs/image_encodings.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace stereolabs {


ZedRgbCvtComponent::ZedRgbCvtComponent(const rclcpp::NodeOptions &options)
    : Node("zed_cvt_node", options)
    , mVideoQos(1) {

    RCLCPP_INFO(get_logger(), "********************************");
    RCLCPP_INFO(get_logger(), " ZED BGRA2BGA Convert Component ");
    RCLCPP_INFO(get_logger(), "********************************");
    RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
    RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
    RCLCPP_INFO(get_logger(), "********************************");

    /* Note: it is very important to use a QOS profile for the subscriber that is compatible
     * with the QOS profile of the publisher.
     * The ZED component node uses a default QoS profile with reliability set as "RELIABLE"
     * and durability set as "VOLATILE".
     * To be able to receive the subscribed topic the subscriber must use compatible
     * parameters.
     */

    // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings

    mVideoQos.keep_last(10);
    mVideoQos.reliable();
    mVideoQos.durability_volatile();

    // Create camera pusblisher for converted image topic
    mPubBgr = image_transport::create_camera_publisher( this, "~/zed_image_3ch", mVideoQos.get_rmw_qos_profile() );
    RCLCPP_INFO_STREAM( get_logger(), "Advertised on topic: " << mPubBgr.getTopic());

    // Create camera subscriber
    mSubBgra = image_transport::create_camera_subscription( this,
                                                 "zed_image_4ch",
                                                 std::bind(&ZedRgbCvtComponent::camera_callback,this,_1,_2),
                                                 "raw",
                                                 mVideoQos.get_rmw_qos_profile());

    RCLCPP_INFO_STREAM( get_logger(), "Subscribed on topic: " << mSubBgra.getTopic());
    RCLCPP_INFO_STREAM( get_logger(), "Subscribed on topic: " << mSubBgra.getInfoTopic());
}

void ZedRgbCvtComponent::camera_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                                         const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info)
{
    // Check for correct input image encoding
    if(img->encoding!=sensor_msgs::image_encodings::BGRA8) {
        RCLCPP_ERROR(get_logger(), "The input topic image requires 'BGRA8' encoding");
        exit(EXIT_FAILURE);
    }

    // Convert BGRA to BGR using OpenCV
    cv::Mat bgra(img->height,img->width,CV_8UC4,(void*)(&img->data[0]));
    cv::Mat bgr;

    cv::cvtColor(bgra,bgr,cv::COLOR_BGRA2BGR);

    // Create the output message and copy coverted data
    std::shared_ptr<sensor_msgs::msg::Image> out_bgr = std::make_shared<sensor_msgs::msg::Image>();

    out_bgr->header.stamp = img->header.stamp;
    out_bgr->header.frame_id = img->header.frame_id;
    out_bgr->height = bgr.rows;
    out_bgr->width = bgr.cols;

    int num = 1; // for endianness detection
    out_bgr->is_bigendian = !(*(char*)&num == 1);

    out_bgr->step = bgr.step;

    size_t size = out_bgr->step * out_bgr->height;
    out_bgr->data.resize(size);

    out_bgr->encoding = sensor_msgs::image_encodings::BGR8;
    memcpy((char*)(&out_bgr->data[0]), &bgr.data[0], size);

    // Publish the new image message coupled with camera info from the original message
    mPubBgr.publish( out_bgr, cam_info);
}

} // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedRgbCvtComponent)
