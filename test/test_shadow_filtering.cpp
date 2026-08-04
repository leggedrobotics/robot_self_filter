#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>

#include "robot_self_filter/self_mask.h"

namespace
{

constexpr char kRobotDescription[] = R"(
<robot name="shadow_test">
  <link name="box">
    <collision>
      <geometry><box size="2 2 2"/></geometry>
    </collision>
  </link>
</robot>
)";

class ShadowFilteringTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  std::vector<int> classify(const bool shadow, const std::vector<pcl::PointXYZ> &points)
  {
    auto node = std::make_shared<rclcpp::Node>(shadow ? "shadow_enabled" : "shadow_disabled");
    node->declare_parameter<std::string>("robot_description", kRobotDescription);
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf_buffer.setUsingDedicatedThread(true);

    robot_self_filter::LinkInfo link;
    link.name = "box";
    link.shadow = shadow;
    link.box_scale = {1.0, 1.0, 1.0};
    link.box_padding = {0.0, 0.0, 0.0};
    robot_self_filter::SelfMask<pcl::PointXYZ> mask(node, tf_buffer, {link});

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.header.frame_id = "box";
    cloud.points.assign(points.begin(), points.end());
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.height = 1;

    std::vector<int> result;
    mask.maskIntersection(cloud, tf2::Vector3(2.0, 0.0, 0.0), 0.0, result);
    return result;
  }
};

TEST_F(ShadowFilteringTest, DefaultBehaviorRejectsPointBehindBoxAsShadow)
{
  EXPECT_EQ(classify(true, {{-2.0F, 0.0F, 0.0F}}), std::vector<int>({robot_self_filter::SHADOW}));
}

TEST_F(ShadowFilteringTest, DisabledShadowLetsPointBehindBoxPass)
{
  EXPECT_EQ(classify(false, {{-2.0F, 0.0F, 0.0F}}), std::vector<int>({robot_self_filter::OUTSIDE}));
}

TEST_F(ShadowFilteringTest, DisabledShadowStillRejectsContainedPoint)
{
  EXPECT_EQ(classify(false, {{0.0F, 0.0F, 0.0F}}), std::vector<int>({robot_self_filter::INSIDE}));
}

}  // namespace
