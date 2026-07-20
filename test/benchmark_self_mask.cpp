#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>

#include "robot_self_filter/self_mask.h"

namespace
{

const char kBenchmarkRobot[] = R"(
<robot name="self_filter_benchmark_robot">
  <link name="robot">
    <collision name="base"><origin xyz="0 0 0.4"/><geometry><box size="3.2 2.4 0.8"/></geometry></collision>
    <collision name="cabin"><origin xyz="-0.6 0 1.8"/><geometry><box size="1.8 2.0 2.0"/></geometry></collision>
    <collision name="boom"><origin xyz="1.2 0 2.4" rpy="0 0.45 0"/><geometry><box size="3.6 0.55 0.55"/></geometry></collision>
    <collision name="stick"><origin xyz="3.4 0 1.7" rpy="0 -0.65 0"/><geometry><box size="2.8 0.45 0.45"/></geometry></collision>
    <collision name="bucket"><origin xyz="4.7 0 0.5"/><geometry><box size="1.2 1.4 0.8"/></geometry></collision>
    <collision name="wheel_lf"><origin xyz="1.0 1.45 0" rpy="1.57079632679 0 0"/><geometry><cylinder radius="0.75" length="0.45"/></geometry></collision>
    <collision name="wheel_lr"><origin xyz="-1.0 1.45 0" rpy="1.57079632679 0 0"/><geometry><cylinder radius="0.75" length="0.45"/></geometry></collision>
    <collision name="wheel_rf"><origin xyz="1.0 -1.45 0" rpy="1.57079632679 0 0"/><geometry><cylinder radius="0.75" length="0.45"/></geometry></collision>
    <collision name="wheel_rr"><origin xyz="-1.0 -1.45 0" rpy="1.57079632679 0 0"/><geometry><cylinder radius="0.75" length="0.45"/></geometry></collision>
    <collision name="joint_1"><origin xyz="0.7 0 1.5"/><geometry><sphere radius="0.45"/></geometry></collision>
    <collision name="joint_2"><origin xyz="2.6 0 2.2"/><geometry><sphere radius="0.38"/></geometry></collision>
  </link>
</robot>)";

struct BenchmarkCase
{
  const char *name;
  std::size_t point_count;
};

std::uint32_t nextRandom(std::uint32_t &state)
{
  state = state * 1664525U + 1013904223U;
  return state;
}

double sample(std::uint32_t &state, double minimum, double maximum)
{
  const double unit = static_cast<double>(nextRandom(state)) /
    static_cast<double>(std::numeric_limits<std::uint32_t>::max());
  return minimum + unit * (maximum - minimum);
}

pcl::PointCloud<pcl::PointXYZ> makeCloud(std::size_t point_count)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = "robot";
  cloud.points.resize(point_count);
  cloud.width = static_cast<std::uint32_t>(point_count);
  cloud.height = 1;

  std::uint32_t random_state = 0x5eed1234U;
  for (std::size_t i = 0; i < point_count; ++i)
  {
    // Mix a broad environment with dense samples around the machine so every
    // classification branch remains represented at every workload size.
    if ((i % 4U) == 0U)
    {
      cloud.points[i].x = static_cast<float>(sample(random_state, -2.5, 6.0));
      cloud.points[i].y = static_cast<float>(sample(random_state, -2.2, 2.2));
      cloud.points[i].z = static_cast<float>(sample(random_state, -0.8, 3.8));
    }
    else
    {
      cloud.points[i].x = static_cast<float>(sample(random_state, -15.0, 20.0));
      cloud.points[i].y = static_cast<float>(sample(random_state, -12.0, 12.0));
      cloud.points[i].z = static_cast<float>(sample(random_state, -2.0, 8.0));
    }
  }
  return cloud;
}

std::uint64_t checksum(const std::vector<int> &mask)
{
  std::uint64_t value = 1469598103934665603ULL;
  for (const int classification : mask)
  {
    value ^= static_cast<std::uint64_t>(classification + 1);
    value *= 1099511628211ULL;
  }
  return value;
}

double median(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2U;
  if ((values.size() % 2U) != 0U)
    return values[middle];
  return (values[middle - 1U] + values[middle]) / 2.0;
}

int parsePositiveArgument(int argc, char **argv, const std::string &name, int default_value)
{
  for (int i = 1; i + 1 < argc; ++i)
  {
    if (argv[i] == name)
    {
      const int value = std::atoi(argv[i + 1]);
      if (value <= 0)
        throw std::runtime_error(name + " must be positive");
      return value;
    }
  }
  return default_value;
}

std::string parseStringArgument(int argc, char **argv, const std::string &name)
{
  for (int i = 1; i + 1 < argc; ++i)
  {
    if (argv[i] == name)
      return argv[i + 1];
  }
  return {};
}

}  // namespace

int main(int argc, char **argv)
{
  const int warmup_count = parsePositiveArgument(argc, argv, "--warmup", 3);
  const int iteration_count = parsePositiveArgument(argc, argv, "--iterations", 10);
  const std::string output_path = parseStringArgument(argc, argv, "--output");

  std::ofstream output_file;
  if (!output_path.empty())
  {
    output_file.open(output_path);
    if (!output_file)
      throw std::runtime_error("failed to open benchmark output: " + output_path);
  }
  std::ostream &output = output_path.empty() ? std::cout : output_file;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("robot_self_filter_benchmark");
  node->declare_parameter<std::string>("robot_description", kBenchmarkRobot);
  tf2_ros::Buffer buffer(node->get_clock());
  buffer.setUsingDedicatedThread(true);

  robot_self_filter::LinkInfo link;
  link.name = "robot";
  link.scale = 1.0;
  link.padding = 0.03;
  link.box_scale = {1.0, 1.0, 1.0};
  link.box_padding = {0.03, 0.03, 0.03};
  link.cylinder_scale = {1.0, 1.0};
  link.cylinder_padding = {0.03, 0.03};
  robot_self_filter::SelfMask<pcl::PointXYZ> mask(node, buffer, {link});

  const std::vector<BenchmarkCase> cases = {
    {"small", 20000U}, {"medium", 100000U}, {"large", 400000U}};
  const tf2::Vector3 sensor_position(0.0, 0.0, 3.2);

  output << "format,robot_self_filter_benchmark_v1\n";
  output << "warmup," << warmup_count << "\n";
  output << "iterations," << iteration_count << "\n";
  output << "case,points,iteration,elapsed_ns,points_per_second,checksum\n";
  output << std::fixed << std::setprecision(3);

  for (const BenchmarkCase &benchmark_case : cases)
  {
    const auto cloud = makeCloud(benchmark_case.point_count);
    std::vector<int> classifications;

    for (int i = 0; i < warmup_count; ++i)
      mask.maskIntersection(cloud, sensor_position, 0.1, classifications);

    const std::uint64_t expected_checksum = checksum(classifications);
    std::vector<double> durations_ns;
    durations_ns.reserve(static_cast<std::size_t>(iteration_count));
    for (int i = 0; i < iteration_count; ++i)
    {
      const auto start = std::chrono::steady_clock::now();
      mask.maskIntersection(cloud, sensor_position, 0.1, classifications);
      const auto stop = std::chrono::steady_clock::now();
      const double elapsed_ns = static_cast<double>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());
      if (checksum(classifications) != expected_checksum)
        throw std::runtime_error("classification checksum changed between iterations");
      durations_ns.push_back(elapsed_ns);
      const double points_per_second =
        static_cast<double>(benchmark_case.point_count) * 1.0e9 / elapsed_ns;
      output << benchmark_case.name << ',' << benchmark_case.point_count << ',' << i << ','
             << elapsed_ns << ',' << points_per_second << ',' << expected_checksum << '\n';
    }

    const double median_ns = median(durations_ns);
    output << "summary_" << benchmark_case.name << ',' << benchmark_case.point_count
           << ",-1," << median_ns << ','
           << static_cast<double>(benchmark_case.point_count) * 1.0e9 / median_ns << ','
           << expected_checksum << '\n';
  }

  rclcpp::shutdown();
  return 0;
}
