#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

class ImageConverter {
public:
    ImageConverter(ros::NodeHandle& nh, const std::string& direction,
                   const std::string& input_topic, const std::string& output_topic)
        : nh_(nh), it_(nh_), direction_(direction) {
        if (direction_ == "comp2raw") {
            comp_sub_ = nh_.subscribe(input_topic, 1, &ImageConverter::compressedCallback, this);
            raw_pub_ = it_.advertise(output_topic, 1);
        } else if (direction_ == "raw2comp") {
            raw_sub_ = it_.subscribe(input_topic, 1, &ImageConverter::rawCallback, this);
            comp_pub_ = nh_.advertise<sensor_msgs::CompressedImage>(output_topic, 1);
        } else {
            ROS_ERROR("Invalid direction: %s. Use 'comp2raw' or 'raw2comp'", direction_.c_str());
        }
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    std::string direction_;

    ros::Subscriber comp_sub_;
    image_transport::Subscriber raw_sub_;
    image_transport::Publisher raw_pub_;
    ros::Publisher comp_pub_;

    void compressedCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
        try {
            cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            if (image.empty()) {
                ROS_WARN("Decoded image is empty!");
                return;
            }
            auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();
            raw_pub_.publish(out_msg);
        } catch (cv::Exception& e) {
            ROS_ERROR("cv::Exception in compressedCallback: %s", e.what());
        }
    }

    void rawCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
            std::vector<uchar> buf;
            cv::imencode(".jpg", image, buf);

            sensor_msgs::CompressedImage comp_msg;
            comp_msg.header = msg->header;
            comp_msg.format = "jpeg";
            comp_msg.data = buf;
            comp_pub_.publish(comp_msg);
        } catch (cv::Exception& e) {
            ROS_ERROR("cv::Exception in rawCallback: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "img_converter");
    ros::NodeHandle nh("~");

    std::string direction, input_topic, output_topic;
    nh.param<std::string>("direction", direction, "comp2raw");
    nh.param<std::string>("input_topic", input_topic, "/camera/image/compressed");
    nh.param<std::string>("output_topic", output_topic, "/camera/image_raw");

    ImageConverter converter(nh, direction, input_topic, output_topic);
    ros::spin();
    return 0;
}
