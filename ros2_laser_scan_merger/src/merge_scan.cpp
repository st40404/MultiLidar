#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LaserScanToPointCloudNode : public rclcpp::Node
{
public:
  LaserScanToPointCloudNode() : Node("laser_scan_to_point_cloud")
  {
    // Create a publisher for the PointCloud2 message
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merge", 10);
    // publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merge", rclcpp::SensorDataQoS());

    // Create a subscriber for the laser scan messages
    scan1_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan_1st", 10, std::bind(&LaserScanToPointCloudNode::scan1_callback, this, std::placeholders::_1));

    scan2_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan_2nd", 10, std::bind(&LaserScanToPointCloudNode::scan2_callback, this, std::placeholders::_1));

    // Create a transform listener to get the transform between base and lidars
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a timer to publish the merged PointCloud2 message
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&LaserScanToPointCloudNode::publish_point_cloud, this));
  }

private:
  void scan1_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Store the laser scan message
    scan1_ = msg;
  }

  void scan2_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Store the laser scan message
    scan2_ = msg;
  }

  void publish_point_cloud()
  {
    if (scan1_ && scan2_)
    {
      // Get the transform between base and lidar_1
      geometry_msgs::msg::TransformStamped transform1;
      try
      {
        transform1 = tf_buffer_->lookupTransform(
            "mobile_base_link", "mobile_lidar_link", tf2::TimePointZero);
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_WARN(this->get_logger(), "Failed to lookup transform from mobile_base_link to lidar_1");
        return;
      }

      // Get the transform between base and lidar_2
      geometry_msgs::msg::TransformStamped transform2;
      try
      {
        transform2 = tf_buffer_->lookupTransform(
            "mobile_base_link", "mobile_lidar_link2", tf2::TimePointZero);
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_WARN(this->get_logger(), "Failed to lookup transform from mobile_base_link to lidar_2");
        return;
      }

      // Convert the laser scan data to PointCloud2 format
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      convert_scan_to_point_cloud(scan1_, transform1, cloud);
      convert_scan_to_point_cloud(scan2_, transform2, cloud);

      // Convert the pcl::PointCloud to sensor_msgs::msg::PointCloud2
      sensor_msgs::msg::PointCloud2::Ptr cloud_msg(new sensor_msgs::msg::PointCloud2);
      pcl::toROSMsg(*cloud, *cloud_msg);
      cloud_msg->header.stamp = this->now();
      cloud_msg->header.frame_id = "mobile_base_link";

      // Publish the merged PointCloud2 message
      publisher_->publish(*cloud_msg);

      // Clear the stored laser scan messages
      scan1_.reset();
      scan2_.reset();
    }
  }

  void convert_scan_to_point_cloud(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                                   const geometry_msgs::msg::TransformStamped &transform,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
      float r = scan->ranges[i];
      if (std::isfinite(r))
      {
        // Convert from polar coordinates to Cartesian coordinates
        float angle = scan->angle_min + i * scan->angle_increment;
        float x = r * std::cos(angle);
        float y = r * std::sin(angle);
        float z = 0.0; // Assuming 2D laser scan

        // Apply the transform between base and lidar
        geometry_msgs::msg::PointStamped p_in, p_out;
        p_in.header = scan->header;
        p_in.point.x = x;
        p_in.point.y = y;
        p_in.point.z = z;
        tf2::doTransform(p_in, p_out, transform);

        pcl::PointXYZ point;
        point.x = p_out.point.x;
        point.y = p_out.point.y;
        point.z = p_out.point.z;
        cloud->push_back(point);
      }
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan1_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan2_subscription_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::LaserScan::SharedPtr scan1_;
  sensor_msgs::msg::LaserScan::SharedPtr scan2_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanToPointCloudNode>());
  rclcpp::shutdown();
  return 0;
}