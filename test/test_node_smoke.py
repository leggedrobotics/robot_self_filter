#!/usr/bin/env python3

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import pytest
import rclpy
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def test_installed_node_filters_cloud_and_respects_wall_time() -> None:
    prefix = Path(get_package_prefix('robot_self_filter'))
    executable = prefix / 'lib' / 'robot_self_filter' / 'self_filter'
    params_file = Path(__file__).with_name('smoke_params.yaml')
    assert executable.is_file()

    suffix = str(os.getpid())
    input_topic = f'/robot_self_filter_smoke_{suffix}/cloud_in'
    output_topic = f'/robot_self_filter_smoke_{suffix}/cloud_out'
    process = subprocess.Popen(
        [
            str(executable),
            '--ros-args',
            '--params-file',
            str(params_file),
            '-p',
            f'in_pointcloud_topic:={input_topic}',
            '-r',
            f'cloud_out:={output_topic}',
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    rclpy.init()
    node = rclpy.create_node(f'robot_self_filter_smoke_client_{suffix}')
    received: list[PointCloud2] = []
    subscription = node.create_subscription(
        PointCloud2, output_topic, received.append, qos_profile_sensor_data
    )
    publisher = node.create_publisher(PointCloud2, input_topic, qos_profile_sensor_data)

    try:
        parameter_client = AsyncParameterClient(node, '/self_filter')
        assert parameter_client.wait_for_services(timeout_sec=10.0)
        parameter_future = parameter_client.get_parameters(['use_sim_time'])
        rclpy.spin_until_future_complete(node, parameter_future, timeout_sec=5.0)
        assert parameter_future.done()
        parameter_result = parameter_future.result()
        assert parameter_result is not None
        assert parameter_result.values[0].bool_value is False

        deadline = time.monotonic() + 10.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            assert process.poll() is None
            rclpy.spin_once(node, timeout_sec=0.05)
        assert publisher.get_subscription_count() > 0

        header = Header()
        header.frame_id = 'robot'
        header.stamp = node.get_clock().now().to_msg()
        cloud = point_cloud2.create_cloud_xyz32(
            header,
            [(0.0, 0.0, 0.0), (3.0, 0.0, 0.0)],
        )
        while not received and time.monotonic() < deadline:
            assert process.poll() is None
            publisher.publish(cloud)
            rclpy.spin_once(node, timeout_sec=0.1)

        assert received
        output = received[-1]
        assert output.header.frame_id == 'robot'
        assert output.header.stamp == cloud.header.stamp
        assert output.width == 1
        assert output.height == 1
        points = list(point_cloud2.read_points(output, field_names=('x', 'y', 'z')))
        assert len(points) == 1
        assert tuple(float(value) for value in points[0]) == pytest.approx(
            (3.0, 0.0, 0.0)
        )
    finally:
        node.destroy_subscription(subscription)
        node.destroy_publisher(publisher)
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate(timeout=5.0)
            pytest.fail(f'self_filter did not shut down cleanly:\n{output}')
        assert process.returncode == 0, output
