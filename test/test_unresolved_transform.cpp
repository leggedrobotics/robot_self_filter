#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include "robot_self_filter/self_mask.h"

namespace robot_self_filter
{
namespace
{

constexpr char kRobotDescription[] = R"(
<robot name="unresolved_transform_test">
  <link name="BASE"/>
  <link name="UNMEASURED">
    <collision>
      <geometry>
        <box size="2 2 2"/>
      </geometry>
    </collision>
  </link>
  <joint name="J_UNMEASURED" type="revolute">
    <parent link="BASE"/>
    <child link="UNMEASURED"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <link name="TIP">
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>
  <joint name="J_TIP" type="fixed">
    <parent link="BASE"/>
    <child link="TIP"/>
  </joint>
</robot>
)";

class UnresolvedTransformTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok())
      rclcpp::shutdown();
  }
};

TEST_F(UnresolvedTransformTest, NeverResolvedBodyDoesNotFilterAtIdentity)
{
  auto node = std::make_shared<rclcpp::Node>("unresolved_transform_test");
  node->declare_parameter<std::string>("robot_description", kRobotDescription);
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf_buffer.setUsingDedicatedThread(true);

  LinkInfo link;
  link.name = "UNMEASURED";
  SelfMask<pcl::PointXYZ> mask(node, tf_buffer, {link});

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = "sensor";
  cloud.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));

  std::vector<int> result;
  mask.maskContainment(cloud, result);

  ASSERT_EQ(result.size(), 1U);
  EXPECT_EQ(result.front(), OUTSIDE);
}

TEST_F(UnresolvedTransformTest, ResolvedBodyStillFiltersAtItsTransform)
{
  auto node = std::make_shared<rclcpp::Node>("resolved_transform_test");
  node->declare_parameter<std::string>("robot_description", kRobotDescription);
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf_buffer.setUsingDedicatedThread(true);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "sensor";
  transform.child_frame_id = "UNMEASURED";
  transform.transform.translation.x = 2.0;
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_buffer.setTransform(transform, "test", true));

  LinkInfo link;
  link.name = "UNMEASURED";
  SelfMask<pcl::PointXYZ> mask(node, tf_buffer, {link});

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = "sensor";
  cloud.push_back(pcl::PointXYZ(2.0F, 0.0F, 0.0F));

  std::vector<int> result;
  mask.maskContainment(cloud, result);

  ASSERT_EQ(result.size(), 1U);
  EXPECT_EQ(result.front(), INSIDE);
}

TEST_F(UnresolvedTransformTest, BoxPoseOverrideOnlyChangesSelfFilterGeometry)
{
  auto node = std::make_shared<rclcpp::Node>("box_pose_override_test");
  node->declare_parameter<std::string>("robot_description", kRobotDescription);
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf_buffer.setUsingDedicatedThread(true);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "sensor";
  transform.child_frame_id = "UNMEASURED";
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_buffer.setTransform(transform, "test", true));

  LinkInfo link;
  link.name = "UNMEASURED";
  link.box_pose_override = {2.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  SelfMask<pcl::PointXYZ> mask(node, tf_buffer, {link});

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = "sensor";
  cloud.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  cloud.push_back(pcl::PointXYZ(2.0F, 0.0F, 0.0F));

  std::vector<int> result;
  mask.maskContainment(cloud, result);

  ASSERT_EQ(result.size(), 2U);
  EXPECT_EQ(result[0], OUTSIDE);
  EXPECT_EQ(result[1], INSIDE);
}

TEST_F(UnresolvedTransformTest, SpherePoseOverrideOnlyChangesSelfFilterGeometry)
{
  auto node = std::make_shared<rclcpp::Node>("sphere_pose_override_test");
  node->declare_parameter<std::string>("robot_description", kRobotDescription);
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf_buffer.setUsingDedicatedThread(true);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "sensor";
  transform.child_frame_id = "TIP";
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_buffer.setTransform(transform, "test", true));

  LinkInfo link;
  link.name = "TIP";
  link.sphere_pose_override = {3.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  SelfMask<pcl::PointXYZ> mask(node, tf_buffer, {link});

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = "sensor";
  cloud.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  cloud.push_back(pcl::PointXYZ(3.0F, 0.0F, 0.0F));

  std::vector<int> result;
  mask.maskContainment(cloud, result);

  ASSERT_EQ(result.size(), 2U);
  EXPECT_EQ(result[0], OUTSIDE);
  EXPECT_EQ(result[1], INSIDE);
}

}  // namespace
}  // namespace robot_self_filter
