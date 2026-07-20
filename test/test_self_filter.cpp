#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>

#include "robot_self_filter/point_hesai.h"
#include "robot_self_filter/point_ouster.h"
#include "robot_self_filter/point_pandar.h"
#include "robot_self_filter/point_robosense.h"
#include "robot_self_filter/self_mask.h"
#include "robot_self_filter/self_see_filter.h"

namespace
{

const char kRobotDescription[] = R"(
<robot name="self_filter_test_robot">
  <link name="robot">
    <collision name="sphere">
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><sphere radius="1.0"/></geometry>
    </collision>
    <collision name="box">
      <origin xyz="3 0 0" rpy="0 0 0"/>
      <geometry><box size="1 2 3"/></geometry>
    </collision>
    <collision name="cylinder">
      <origin xyz="-3 0 0" rpy="0 0 0"/>
      <geometry><cylinder radius="0.75" length="2"/></geometry>
    </collision>
  </link>
</robot>)";

std::string uniqueNodeName(const std::string &prefix)
{
  static std::uint64_t sequence = 0;
  return prefix + "_" + std::to_string(++sequence);
}

rclcpp::Node::SharedPtr makeNode(const std::string &prefix)
{
  auto node = std::make_shared<rclcpp::Node>(uniqueNodeName(prefix));
  node->declare_parameter<std::string>("robot_description", kRobotDescription);
  return node;
}

template<typename PointT>
class TestableSelfFilter : public filters::SelfFilter<PointT>
{
public:
  explicit TestableSelfFilter(const rclcpp::Node::SharedPtr &node)
  : filters::SelfFilter<PointT>(node)
  {
  }

  pcl::PointCloud<PointT> render(
    const pcl::PointCloud<PointT> &input,
    const std::vector<int> &mask,
    bool keep_organized,
    bool zero_for_removed_points,
    bool invert)
  {
    this->keep_organized_ = keep_organized;
    this->zero_for_removed_points_ = zero_for_removed_points;
    this->invert_ = invert;
    pcl::PointCloud<PointT> output;
    this->fillResult(input, mask, output);
    return output;
  }
};

template<typename PointT>
PointT makePoint(float x, float y, float z, float intensity)
{
  PointT point{};
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  return point;
}

class RclcppEnvironment : public testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    if (rclcpp::ok())
      rclcpp::shutdown();
  }
};

testing::Environment * const rclcpp_environment =
  testing::AddGlobalTestEnvironment(new RclcppEnvironment());

TEST(SelfMask, ClassifiesContainmentShadowOutsideMinimumDistanceNaNAndEmptyCloud)
{
  auto node = makeNode("self_mask_classification");
  tf2_ros::Buffer buffer(node->get_clock());
  buffer.setUsingDedicatedThread(true);
  robot_self_filter::LinkInfo link;
  link.name = "robot";
  link.scale = 1.0;
  link.padding = 0.0;
  link.box_scale = {1.0, 1.0, 1.0};
  link.box_padding = {0.0, 0.0, 0.0};
  link.cylinder_scale = {1.0, 1.0};
  link.cylinder_padding = {0.0, 0.0};
  robot_self_filter::SelfMask<pcl::PointXYZ> mask(node, buffer, {link});

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = "robot";
  cloud.points = {
    pcl::PointXYZ(0.0F, 0.0F, 0.0F),
    pcl::PointXYZ(3.0F, 0.0F, 0.0F),
    pcl::PointXYZ(-3.0F, 0.0F, 0.0F),
    pcl::PointXYZ(5.0F, 0.0F, 0.0F),
    pcl::PointXYZ(0.0F, 5.0F, 0.0F),
    pcl::PointXYZ(0.0F, 0.0F, 2.05F),
    pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F)};
  cloud.width = static_cast<std::uint32_t>(cloud.points.size());
  cloud.height = 1;

  std::vector<int> result;
  mask.maskIntersection(cloud, tf2::Vector3(0.0, 0.0, 2.0), 0.1, result);
  ASSERT_EQ(result.size(), cloud.points.size());
  EXPECT_EQ(result[0], robot_self_filter::INSIDE);
  EXPECT_EQ(result[1], robot_self_filter::INSIDE);
  EXPECT_EQ(result[2], robot_self_filter::INSIDE);
  EXPECT_EQ(result[3], robot_self_filter::SHADOW);
  EXPECT_EQ(result[4], robot_self_filter::OUTSIDE);
  EXPECT_EQ(result[5], robot_self_filter::INSIDE);
  EXPECT_EQ(result[6], robot_self_filter::OUTSIDE);

  pcl::PointCloud<pcl::PointXYZ> empty;
  empty.header.frame_id = "robot";
  mask.maskIntersection(empty, tf2::Vector3(0.0, 0.0, 2.0), 0.1, result);
  EXPECT_TRUE(result.empty());
}

TEST(SelfMask, ContainmentUsesCollisionOriginsAndPerShapeScale)
{
  auto node = makeNode("self_mask_containment");
  tf2_ros::Buffer buffer(node->get_clock());
  buffer.setUsingDedicatedThread(true);
  robot_self_filter::LinkInfo link;
  link.name = "robot";
  link.scale = 1.5;
  link.padding = 0.1;
  link.box_scale = {2.0, 1.0, 0.5};
  link.box_padding = {0.0, 0.1, 0.2};
  link.cylinder_scale = {1.25, 0.5};
  link.cylinder_padding = {0.05, 0.1};
  robot_self_filter::SelfMask<pcl::PointXYZ> mask(node, buffer, {link});

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = "robot";
  cloud.points = {
    pcl::PointXYZ(1.59F, 0.0F, 0.0F),
    pcl::PointXYZ(4.0F, 0.0F, 0.0F),
    pcl::PointXYZ(3.0F, 1.09F, 0.0F),
    pcl::PointXYZ(-3.98F, 0.0F, 0.0F),
    pcl::PointXYZ(-3.0F, 0.0F, 0.59F),
    pcl::PointXYZ(0.0F, 4.0F, 0.0F)};
  cloud.width = static_cast<std::uint32_t>(cloud.points.size());
  cloud.height = 1;

  std::vector<int> result;
  mask.maskContainment(cloud, result);
  ASSERT_EQ(result.size(), cloud.points.size());
  EXPECT_EQ(result[0], robot_self_filter::INSIDE);
  EXPECT_EQ(result[1], robot_self_filter::INSIDE);
  EXPECT_EQ(result[2], robot_self_filter::INSIDE);
  EXPECT_EQ(result[3], robot_self_filter::INSIDE);
  EXPECT_EQ(result[4], robot_self_filter::INSIDE);
  EXPECT_EQ(result[5], robot_self_filter::OUTSIDE);
}

TEST(SelfMask, UsesPrefixedNameForTfAndSuffixForUrdfLookup)
{
  auto node = makeNode("self_mask_prefixed_link");
  tf2_ros::Buffer buffer(node->get_clock());
  buffer.setUsingDedicatedThread(true);
  robot_self_filter::LinkInfo link;
  link.name = "agent_7/robot";
  link.scale = 1.0;
  link.padding = 0.0;
  link.box_scale = {1.0, 1.0, 1.0};
  link.box_padding = {0.0, 0.0, 0.0};
  link.cylinder_scale = {1.0, 1.0};
  link.cylinder_padding = {0.0, 0.0};
  robot_self_filter::SelfMask<pcl::PointXYZ> mask(node, buffer, {link});

  std::vector<std::string> link_names;
  mask.getLinkNames(link_names);
  ASSERT_EQ(link_names.size(), 3U);
  EXPECT_EQ(link_names[0], "agent_7/robot");

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = "agent_7/robot";
  cloud.points = {pcl::PointXYZ(0.0F, 0.0F, 0.0F)};
  cloud.width = 1;
  cloud.height = 1;
  std::vector<int> result;
  mask.maskContainment(cloud, result);
  ASSERT_EQ(result.size(), 1U);
  EXPECT_EQ(result[0], robot_self_filter::INSIDE);
}

TEST(SelfFilterOutput, UnorganizedNormalAndInvertedModesPreserveSelectedPoints)
{
  auto node = makeNode("filter_output_unorganized");
  TestableSelfFilter<pcl::PointXYZ> filter(node);
  pcl::PointCloud<pcl::PointXYZ> input;
  input.points = {
    pcl::PointXYZ(1.0F, 2.0F, 3.0F),
    pcl::PointXYZ(4.0F, 5.0F, 6.0F),
    pcl::PointXYZ(7.0F, 8.0F, 9.0F)};
  input.width = 3;
  input.height = 1;
  const std::vector<int> mask = {
    robot_self_filter::OUTSIDE, robot_self_filter::INSIDE, robot_self_filter::SHADOW};

  const auto normal = filter.render(input, mask, false, false, false);
  ASSERT_EQ(normal.size(), 1U);
  EXPECT_FLOAT_EQ(normal[0].x, 1.0F);
  EXPECT_EQ(normal.width, 1U);
  EXPECT_EQ(normal.height, 1U);

  const auto inverted = filter.render(input, mask, false, false, true);
  ASSERT_EQ(inverted.size(), 2U);
  EXPECT_FLOAT_EQ(inverted[0].x, 4.0F);
  EXPECT_FLOAT_EQ(inverted[1].x, 7.0F);
}

TEST(SelfFilterOutput, OrganizedModeUsesConfiguredRemovedPointRepresentation)
{
  auto node = makeNode("filter_output_organized");
  TestableSelfFilter<pcl::PointXYZ> filter(node);
  pcl::PointCloud<pcl::PointXYZ> input;
  input.points = {
    pcl::PointXYZ(1.0F, 2.0F, 3.0F), pcl::PointXYZ(4.0F, 5.0F, 6.0F),
    pcl::PointXYZ(7.0F, 8.0F, 9.0F), pcl::PointXYZ(10.0F, 11.0F, 12.0F)};
  input.width = 2;
  input.height = 2;
  input.header.frame_id = "test_cloud";
  input.is_dense = true;
  input.sensor_origin_ = Eigen::Vector4f(1.0F, 2.0F, 3.0F, 1.0F);
  input.sensor_orientation_ = Eigen::Quaternionf(1.0F, 0.0F, 0.0F, 0.0F);
  const std::vector<int> mask = {
    robot_self_filter::OUTSIDE, robot_self_filter::INSIDE,
    robot_self_filter::SHADOW, robot_self_filter::OUTSIDE};

  const auto nan_output = filter.render(input, mask, true, false, false);
  ASSERT_EQ(nan_output.size(), 4U);
  EXPECT_EQ(nan_output.width, 2U);
  EXPECT_EQ(nan_output.height, 2U);
  EXPECT_EQ(nan_output.header.frame_id, "test_cloud");
  EXPECT_FALSE(nan_output.is_dense);
  EXPECT_TRUE(nan_output.sensor_origin_.isApprox(input.sensor_origin_));
  EXPECT_TRUE(nan_output.sensor_orientation_.isApprox(input.sensor_orientation_));
  EXPECT_FLOAT_EQ(nan_output[0].x, 1.0F);
  EXPECT_TRUE(std::isnan(nan_output[1].x));
  EXPECT_TRUE(std::isnan(nan_output[2].y));
  EXPECT_FLOAT_EQ(nan_output[3].z, 12.0F);

  const auto zero_output = filter.render(input, mask, true, true, false);
  EXPECT_FLOAT_EQ(zero_output[1].x, 0.0F);
  EXPECT_FLOAT_EQ(zero_output[1].y, 0.0F);
  EXPECT_FLOAT_EQ(zero_output[1].z, 0.0F);
  EXPECT_FLOAT_EQ(zero_output[2].x, 0.0F);
  EXPECT_TRUE(zero_output.is_dense);
}

TEST(SelfFilterOutput, RejectsMaskSizeMismatch)
{
  auto node = makeNode("filter_output_mask_mismatch");
  TestableSelfFilter<pcl::PointXYZ> filter(node);
  pcl::PointCloud<pcl::PointXYZ> input;
  input.points = {pcl::PointXYZ(1.0F, 2.0F, 3.0F)};
  input.width = 1;
  input.height = 1;
  EXPECT_THROW(filter.render(input, {}, false, false, false), std::invalid_argument);
}

TEST(SelfFilterOutput, InitializesRemovedCustomPointFieldsDeterministically)
{
  auto node = makeNode("filter_output_custom_blank");
  TestableSelfFilter<PointOuster> filter(node);
  pcl::PointCloud<PointOuster> input;
  PointOuster point = makePoint<PointOuster>(1.0F, 2.0F, 3.0F, 42.0F);
  point.t = 1234U;
  point.reflectivity = 55U;
  point.ring = 7U;
  point.ambient = 88U;
  point.range = 900U;
  input.points = {point};
  input.width = 1;
  input.height = 1;

  const auto output = filter.render(
    input, {robot_self_filter::INSIDE}, true, false, false);
  ASSERT_EQ(output.size(), 1U);
  EXPECT_TRUE(std::isnan(output[0].x));
  EXPECT_FLOAT_EQ(output[0].intensity, 0.0F);
  EXPECT_EQ(output[0].t, 0U);
  EXPECT_EQ(output[0].reflectivity, 0U);
  EXPECT_EQ(output[0].ring, 0U);
  EXPECT_EQ(output[0].ambient, 0U);
  EXPECT_EQ(output[0].range, 0U);
}

template<typename PointT>
void expectCustomPointLayoutIsPreserved()
{
  auto node = makeNode("custom_point_layout");
  TestableSelfFilter<PointT> filter(node);
  pcl::PointCloud<PointT> input;
  input.points = {
    makePoint<PointT>(1.0F, 2.0F, 3.0F, 11.0F),
    makePoint<PointT>(4.0F, 5.0F, 6.0F, 22.0F)};
  input.width = 2;
  input.height = 1;
  const auto output = filter.render(
    input, {robot_self_filter::OUTSIDE, robot_self_filter::INSIDE}, false, false, false);
  ASSERT_EQ(output.size(), 1U);
  EXPECT_FLOAT_EQ(output[0].x, 1.0F);
  EXPECT_FLOAT_EQ(output[0].intensity, 11.0F);
}

TEST(SelfFilterOutput, PreservesOusterPointLayout)
{
  expectCustomPointLayoutIsPreserved<PointOuster>();
}

TEST(SelfFilterOutput, PreservesHesaiPointLayout)
{
  expectCustomPointLayoutIsPreserved<PointHesai>();
}

TEST(SelfFilterOutput, PreservesRobosensePointLayout)
{
  expectCustomPointLayoutIsPreserved<PointRobosense>();
}

TEST(SelfFilterOutput, PreservesPandarPointLayout)
{
  expectCustomPointLayoutIsPreserved<PointPandar>();
}

}  // namespace
