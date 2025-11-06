#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

class RansacGroundNode : public rclcpp::Node
{
public:
    RansacGroundNode()
    : Node("ransac_ground_node")
    {
        // --- Parameter deklarieren ---
        this->declare_parameter<std::string>("input_topic", "/points_voxel");
        this->declare_parameter<std::string>("output_topic", "/obstacle_points");
        this->declare_parameter<double>("distance_threshold", 0.15);
        this->declare_parameter<int>("max_iterations", 500);

        auto input_topic = this->get_parameter("input_topic").as_string();
        auto output_topic = this->get_parameter("output_topic").as_string();

        // --- Subscriber für Punktwolken ---
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, rclcpp::SensorDataQoS(),
            std::bind(&RansacGroundNode::callback, this, std::placeholders::_1));

        // --- Publisher für Hindernis-Punktwolken (Boden entfernt) ---
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Eingehende ROS2-Cloud -> PCL-Cloud konvertieren
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->points.empty())
            return;

        // --- RANSAC-Planarsegmentierung konfigurieren ---
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(this->get_parameter("distance_threshold").as_double());
        seg.setMaxIterations(this->get_parameter("max_iterations").as_int());

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
            return;

        // --- Extrahiere Hindernisse: alle Punkte, die NICHT zum Bodenplan gehören ---
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZI>);
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*obstacles);

        // --- Veröffentlichung der Hindernis-Punktwolke ---
        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(*obstacles, out_msg);
        out_msg.header = msg->header;
        pub_->publish(out_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RansacGroundNode>());
    rclcpp::shutdown();
    return 0;
}
