// ============================ self_filter.cpp ============================
#include <chrono>
#include <sstream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "robot_self_filter/self_see_filter.h"
#include "robot_self_filter/point_ouster.h"
#include "robot_self_filter/point_hesai.h"
#include "robot_self_filter/point_pandar.h"
#include "robot_self_filter/point_robosense.h"

#include <yaml-cpp/yaml.h>
#include <robot_self_filter/bodies.h>
#include <robot_self_filter/shapes.h>

namespace robot_self_filter
{

  enum class SensorType : int
  {
    XYZSensor = 0,
    XYZRGBSensor = 1,
    OusterSensor = 2,
    HesaiSensor = 3,
    RobosenseSensor = 4,
    PandarSensor = 5,
  };

  class SelfFilterNode : public rclcpp::Node
  {
  public:
    SelfFilterNode()
        : Node("self_filter")
    {
      try
      {
        this->declare_parameter<bool>("use_sim_time", true);
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
      }
      catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &)
      {
      }

      this->declare_parameter<std::string>("sensor_frame", "Lidar"); // Default value
      // this->set_parameter(rclcpp::Parameter("sensor_frame", "Lidar")); // Removed explicit set
      this->declare_parameter<bool>("use_rgb", false);
      this->declare_parameter<int>("max_queue_size", 10);
      this->declare_parameter<int>("lidar_sensor_type", 0);
      this->declare_parameter<std::string>("robot_description", "");
      this->declare_parameter<std::string>("in_pointcloud_topic", "/cloud_in");

      sensor_frame_ = this->get_parameter("sensor_frame").as_string();
      use_rgb_ = this->get_parameter("use_rgb").as_bool();
      max_queue_size_ = this->get_parameter("max_queue_size").as_int();
      int temp_sensor_type = this->get_parameter("lidar_sensor_type").as_int();
      sensor_type_ = static_cast<SensorType>(temp_sensor_type);
      in_topic_ = this->get_parameter("in_pointcloud_topic").as_string();

      RCLCPP_INFO(this->get_logger(), "Parameters:");
      RCLCPP_INFO(this->get_logger(), "  sensor_frame: %s", sensor_frame_.c_str());
      RCLCPP_INFO(this->get_logger(), "  use_rgb: %s", use_rgb_ ? "true" : "false");
      RCLCPP_INFO(this->get_logger(), "  max_queue_size: %d", max_queue_size_);
      RCLCPP_INFO(this->get_logger(), "  lidar_sensor_type: %d", temp_sensor_type);
      RCLCPP_INFO(this->get_logger(), "  in_pointcloud_topic: %s", in_topic_.c_str());

      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_buffer_->setCreateTimerInterface(
          std::make_shared<tf2_ros::CreateTimerROS>(
              this->get_node_base_interface(),
              this->get_node_timers_interface()));
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      pointCloudPublisher_ =
          this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out", 1);

      marker_pub_ =
          this->create_publisher<visualization_msgs::msg::MarkerArray>("collision_shapes", 1);
    }

    void initSelfFilter()
    {
      std::string robot_description_xml = this->get_parameter("robot_description").as_string();

      switch (sensor_type_)
      {
      case SensorType::XYZSensor:
        self_filter_ = std::make_shared<filters::SelfFilter<pcl::PointXYZ>>(this->shared_from_this());
        break;
      case SensorType::XYZRGBSensor:
        self_filter_ = std::make_shared<filters::SelfFilter<pcl::PointXYZRGB>>(this->shared_from_this());
        break;
      case SensorType::OusterSensor:
        self_filter_ = std::make_shared<filters::SelfFilter<PointOuster>>(this->shared_from_this());
        break;
      case SensorType::HesaiSensor:
        self_filter_ = std::make_shared<filters::SelfFilter<PointHesai>>(this->shared_from_this());
        break;
      case SensorType::RobosenseSensor:
        self_filter_ = std::make_shared<filters::SelfFilter<PointRobosense>>(this->shared_from_this());
        break;
      case SensorType::PandarSensor:
        self_filter_ = std::make_shared<filters::SelfFilter<PointPandar>>(this->shared_from_this());
        break;
      default:
        self_filter_ = std::make_shared<filters::SelfFilter<pcl::PointXYZ>>(this->shared_from_this());
        break;
      }

      self_filter_->getLinkNames(frames_);

      sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          in_topic_,
          rclcpp::SensorDataQoS(),
          std::bind(&SelfFilterNode::cloudCallback, this, std::placeholders::_1));
    }

  private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud)
    {
      RCLCPP_INFO(this->get_logger(), "Received cloud message with timestamp %.6f",
                  rclcpp::Time(cloud->header.stamp).seconds());

      RCLCPP_INFO(this->get_logger(), "Point cloud size: width = %d, height = %d, total points = %d",
                  cloud->width, cloud->height, cloud->width * cloud->height);

      sensor_msgs::msg::PointCloud2 out2;
      int input_size = 0;
      int output_size = 0;

      self_filter_->fillPointCloud2(cloud, sensor_frame_, out2, input_size, output_size);
      pointCloudPublisher_->publish(out2);

      switch (sensor_type_)
      {
      case SensorType::XYZSensor:
      {
        auto sf_xyz = std::dynamic_pointer_cast<filters::SelfFilter<pcl::PointXYZ>>(self_filter_);
        if (!sf_xyz)
          return;
        auto mask = sf_xyz->getSelfMaskPtr();
        publishShapesFromMask(mask, cloud->header.frame_id);
        break;
      }
      case SensorType::OusterSensor:
      {
        auto sf_ouster = std::dynamic_pointer_cast<filters::SelfFilter<PointOuster>>(self_filter_);
        if (!sf_ouster)
          return;
        auto mask = sf_ouster->getSelfMaskPtr();
        publishShapesFromMask(mask, cloud->header.frame_id);
        break;
      }
      default:
        RCLCPP_ERROR(this->get_logger(), "Sensor type not handled for shape publishing");
        return;
      }
    }

    template <typename PointT>
    void publishShapesFromMask(robot_self_filter::SelfMask<PointT> *mask, const std::string &pointcloud_frame)
    {
      if (!mask)
        return;
      const auto &bodies = mask->getBodies();
      if (bodies.empty())
      {
        RCLCPP_ERROR(this->get_logger(), "No bodies found in SelfMask");
        return;
      }

      visualization_msgs::msg::MarkerArray marker_array;
      marker_array.markers.reserve(bodies.size());

      std::string shapes_frame = pointcloud_frame;
      for (size_t i = 0; i < bodies.size(); ++i)
      {
        const auto &see_link = bodies[i];
        const bodies::Body *body = see_link.body;
        if (!body)
          continue;

        visualization_msgs::msg::Marker mk;
        mk.header.frame_id = shapes_frame;
        mk.header.stamp = this->get_clock()->now();
        mk.ns = "self_filter_shapes";
        mk.id = static_cast<int>(i);
        mk.action = visualization_msgs::msg::Marker::ADD;
        mk.lifetime = rclcpp::Duration(0, 0);
        mk.color.a = 0.5f;
        mk.color.r = 1.0f;
        mk.color.g = 0.0f;
        mk.color.b = 0.0f;

        const tf2::Transform &tf = body->getPose();
        mk.pose.position.x = tf.getOrigin().x();
        mk.pose.position.y = tf.getOrigin().y();
        mk.pose.position.z = tf.getOrigin().z();
        tf2::Quaternion q = tf.getRotation();
        mk.pose.orientation.x = q.x();
        mk.pose.orientation.y = q.y();
        mk.pose.orientation.z = q.z();
        mk.pose.orientation.w = q.w();

        switch (body->getType())
        {
        case shapes::SPHERE:
        {
          auto sphere_body = dynamic_cast<const robot_self_filter::bodies::Sphere *>(body);
          if (sphere_body)
          {
            mk.type = visualization_msgs::msg::Marker::SPHERE;
            float d = static_cast<float>(2.0 * sphere_body->getScaledRadius());
            mk.scale.x = d;
            mk.scale.y = d;
            mk.scale.z = d;
          }
          break;
        }
        case shapes::BOX:
        {
          auto box_body = dynamic_cast<const robot_self_filter::bodies::Box *>(body);
          if (box_body)
          {
            mk.type = visualization_msgs::msg::Marker::CUBE;
            float sx = static_cast<float>(2.0 * box_body->getScaledHalfLength());
            float sy = static_cast<float>(2.0 * box_body->getScaledHalfWidth());
            float sz = static_cast<float>(2.0 * box_body->getScaledHalfHeight());
            mk.scale.x = sx;
            mk.scale.y = sy;
            mk.scale.z = sz;
          }
          break;
        }
        case shapes::CYLINDER:
        {
          auto cyl_body = dynamic_cast<const robot_self_filter::bodies::Cylinder *>(body);
          if (cyl_body)
          {
            mk.type = visualization_msgs::msg::Marker::CYLINDER;
            float radius = static_cast<float>(cyl_body->getScaledRadius());
            float length = static_cast<float>(2.0 * cyl_body->getScaledHalfLength());
            mk.scale.x = radius * 2.0f;
            mk.scale.y = radius * 2.0f;
            mk.scale.z = length;
          }
          break;
        }
        case shapes::MESH:
        {
          auto mesh_body = dynamic_cast<const robot_self_filter::bodies::ConvexMesh *>(body);
          if (mesh_body)
          {
            mk.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            mk.scale.x = mk.scale.y = mk.scale.z = 1.0f;

            const auto &verts = mesh_body->getScaledVertices();
            const auto &tris = mesh_body->getTriangles();
            mk.points.reserve(tris.size());
            for (size_t t_i = 0; t_i < tris.size(); ++t_i)
            {
              geometry_msgs::msg::Point p;
              p.x = verts[tris[t_i]].x();
              p.y = verts[tris[t_i]].y();
              p.z = verts[tris[t_i]].z();
              mk.points.push_back(p);
            }
          }
          break;
        }
        default:
          break;
        }
        marker_array.markers.push_back(mk);
      }

      marker_pub_->publish(marker_array);
      RCLCPP_INFO(this->get_logger(), "Published %zu collision shapes", marker_array.markers.size());
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<filters::SelfFilterInterface> self_filter_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPublisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    std::string sensor_frame_;
    bool use_rgb_;
    SensorType sensor_type_;
    int max_queue_size_;
    std::vector<std::string> frames_;
    std::string in_topic_;
  };

} // namespace robot_self_filter

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_self_filter::SelfFilterNode>();
  node->initSelfFilter();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
