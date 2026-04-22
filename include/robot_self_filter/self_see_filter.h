// ============================ self_see_filter.h (Updated) ============================
#ifndef FILTERS_SELF_SEE_H_
#define FILTERS_SELF_SEE_H_

#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <filters/filter_base.hpp>
#include <robot_self_filter/self_mask.h>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>  // For explicit constructor

namespace filters
{

class SelfFilterInterface
{
public:
  virtual ~SelfFilterInterface() = default;

  virtual void getLinkNames(std::vector<std::string> &frames) = 0;

  virtual bool fillPointCloud2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud2,
                               const std::string &sensor_frame,
                               sensor_msgs::msg::PointCloud2 &out2,
                               int &input_size,
                               int &output_size) = 0;
};

template <typename PointT>
class SelfFilter : public FilterBase<pcl::PointCloud<PointT>>, public SelfFilterInterface
{
public:
  using PointCloud = pcl::PointCloud<PointT>;

  template <class NodeT>
  explicit SelfFilter(const std::shared_ptr<NodeT> &node,
                      std::shared_ptr<tf2_ros::Buffer> existing_tf_buffer = nullptr)
    : tf_buffer_(existing_tf_buffer
                 ? existing_tf_buffer
                 : std::make_shared<tf2_ros::Buffer>(node->get_clock()))
    , tf_listener_(existing_tf_buffer
                   ? nullptr
                   : std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    node->template declare_parameter<double>("min_sensor_dist", 0.01, rcl_interfaces::msg::ParameterDescriptor());
    node->template declare_parameter<bool>("keep_organized", false, rcl_interfaces::msg::ParameterDescriptor());
    node->template declare_parameter<bool>("zero_for_removed_points", false, rcl_interfaces::msg::ParameterDescriptor());
    node->template declare_parameter<bool>("invert", false, rcl_interfaces::msg::ParameterDescriptor());

    node->template declare_parameter<std::vector<double>>("default_box_scale",
      {1.0, 1.0, 1.0}, rcl_interfaces::msg::ParameterDescriptor());
    node->template declare_parameter<std::vector<double>>("default_box_padding",
      {0.01, 0.01, 0.01}, rcl_interfaces::msg::ParameterDescriptor());
    node->template declare_parameter<std::vector<double>>("default_cylinder_scale",
      {1.0, 1.0}, rcl_interfaces::msg::ParameterDescriptor());
    node->template declare_parameter<std::vector<double>>("default_cylinder_padding",
      {0.01, 0.01}, rcl_interfaces::msg::ParameterDescriptor());
    node->template declare_parameter<double>("default_sphere_scale", 1.0, rcl_interfaces::msg::ParameterDescriptor());
    node->template declare_parameter<double>("default_sphere_padding", 0.01, rcl_interfaces::msg::ParameterDescriptor());

    node->get_parameter("min_sensor_dist", min_sensor_dist_);
    node->get_parameter("keep_organized", keep_organized_);
    node->get_parameter("zero_for_removed_points", zero_for_removed_points_);
    node->get_parameter("invert", invert_);

    node->get_parameter("default_box_scale", default_box_scale_);
    node->get_parameter("default_box_padding", default_box_pad_);
    node->get_parameter("default_cylinder_scale", default_cyl_scale_);
    node->get_parameter("default_cylinder_padding", default_cyl_pad_);
    node->get_parameter("default_sphere_scale", default_sphere_scale_);
    node->get_parameter("default_sphere_padding", default_sphere_pad_);

    node->template declare_parameter<std::vector<std::string>>(
      "self_see_links.names",
      std::vector<std::string>(),
      rcl_interfaces::msg::ParameterDescriptor()
    );
    std::vector<std::string> link_names;
    node->get_parameter("self_see_links.names", link_names);

    std::vector<robot_self_filter::LinkInfo> links;
    for (auto &lname : link_names)
    {
      robot_self_filter::LinkInfo li;
      li.name = lname;
      li.scale   = default_sphere_scale_;
      li.padding = default_sphere_pad_;

      std::string box_scale_key   = "self_see_links." + lname + ".box_scale";
      std::string box_padding_key = "self_see_links." + lname + ".box_padding";
      std::string cyl_scale_key   = "self_see_links." + lname + ".cylinder_scale";
      std::string cyl_padding_key = "self_see_links." + lname + ".cylinder_padding";
      std::string padding_key     = "self_see_links." + lname + ".padding";
      std::string scale_key       = "self_see_links." + lname + ".scale";

      node->template declare_parameter<std::vector<double>>(box_scale_key,
        std::vector<double>(), rcl_interfaces::msg::ParameterDescriptor());
      node->template declare_parameter<std::vector<double>>(box_padding_key,
        std::vector<double>(), rcl_interfaces::msg::ParameterDescriptor());
      node->get_parameter(box_scale_key, li.box_scale);
      node->get_parameter(box_padding_key, li.box_padding);

      node->template declare_parameter<std::vector<double>>(cyl_scale_key,
        std::vector<double>(), rcl_interfaces::msg::ParameterDescriptor());
      node->template declare_parameter<std::vector<double>>(cyl_padding_key,
        std::vector<double>(), rcl_interfaces::msg::ParameterDescriptor());
      node->get_parameter(cyl_scale_key, li.cylinder_scale);
      node->get_parameter(cyl_padding_key, li.cylinder_padding);

      node->template declare_parameter<double>(padding_key, default_sphere_pad_, rcl_interfaces::msg::ParameterDescriptor());
      node->template declare_parameter<double>(scale_key, default_sphere_scale_, rcl_interfaces::msg::ParameterDescriptor());
      double link_pad = default_sphere_pad_;
      double link_scl = default_sphere_scale_;
      node->get_parameter(padding_key, link_pad);
      node->get_parameter(scale_key, link_scl);
      li.padding = link_pad;
      li.scale   = link_scl;

      links.push_back(li);
    }

    sm_ = std::make_shared<robot_self_filter::SelfMask<PointT>>(node, *tf_buffer_, links);
  }

  ~SelfFilter() override = default;

  bool configure() override
  {
    return true;
  }

  void getLinkNames(std::vector<std::string> &frames) override
  {
    if (sm_) sm_->getLinkNames(frames);
  }

  bool fillPointCloud2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud2,
                       const std::string &sensor_frame,
                       sensor_msgs::msg::PointCloud2 &out2,
                       int &input_size,
                       int &output_size) override
  {
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*cloud2, *cloud);

    pcl::PointCloud<PointT> out;
    sensor_frame_ = sensor_frame;
    update(*cloud, out);

    pcl::toROSMsg(out, out2);
    out2.header = cloud2->header;

    input_size  = static_cast<int>(cloud->points.size());
    output_size = static_cast<int>(out.points.size());
    return true;
  }

  robot_self_filter::SelfMask<PointT>* getSelfMaskPtr() const
  {
    return sm_.get();
  }

protected:
  bool update(const PointCloud &data_in, PointCloud &data_out) override
  {
    std::vector<int> keep(data_in.points.size(), 0);
    if (sensor_frame_.empty())
      sm_->maskContainment(data_in, keep);
    else
      sm_->maskIntersection(data_in, sensor_frame_, min_sensor_dist_, keep);
    fillResult(data_in, keep, data_out);
    return true;
  }

  void fillResult(const PointCloud &data_in, const std::vector<int> &keep, PointCloud &data_out)
  {
    data_out.header = data_in.header;
    data_out.points.clear();
    data_out.points.reserve(data_in.points.size());

    PointT blank_pt;
    if (zero_for_removed_points_)
    {
      blank_pt.x = 0.0f;
      blank_pt.y = 0.0f;
      blank_pt.z = 0.0f;
    }
    else
    {
      blank_pt.x = std::numeric_limits<float>::quiet_NaN();
      blank_pt.y = std::numeric_limits<float>::quiet_NaN();
      blank_pt.z = std::numeric_limits<float>::quiet_NaN();
    }

    for (size_t i = 0; i < data_in.points.size(); ++i)
    {
      bool outside = (keep[i] == robot_self_filter::OUTSIDE);
      if (outside && !invert_)
      {
        data_out.points.push_back(data_in.points[i]);
      }
      else if (!outside && invert_)
      {
        data_out.points.push_back(data_in.points[i]);
      }
      else if (keep_organized_)
      {
        data_out.points.push_back(blank_pt);
      }
    }

    if (keep_organized_)
    {
      data_out.width  = data_in.width;
      data_out.height = data_in.height;
    }
    else
    {
      data_out.width  = static_cast<uint32_t>(data_out.points.size());
      data_out.height = 1;
    }
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<robot_self_filter::SelfMask<PointT>> sm_;

  bool keep_organized_ = false;
  bool zero_for_removed_points_ = false;
  bool invert_ = false;
  double min_sensor_dist_ = 0.01;

  std::vector<double> default_box_scale_;
  std::vector<double> default_box_pad_;
  std::vector<double> default_cyl_scale_;
  std::vector<double> default_cyl_pad_;
  double default_sphere_scale_ = 1.0;
  double default_sphere_pad_ = 0.01;

  std::string sensor_frame_;
};

}  // namespace filters

#endif  // FILTERS_SELF_SEE_H_
