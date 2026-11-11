#!/usr/bin/env python3
"""Republish points whose intensity equals a semantic class ID (default: stairs=1).

Example, inside the running simulator container:
  python3 /opt/dfl/scripts/filter_semantic_cloud.py --label 1
  ros2 param set /semantic_cloud_filter label 2
"""
import argparse
import copy

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import PointCloud2, PointField


def filter_cloud(cloud, label):
    """Keep complete point records, including extra fields, endianness and padding."""
    field = next((field for field in cloud.fields if field.name == 'intensity'), None)
    formats = {
        PointField.INT8: 'i1', PointField.UINT8: 'u1',
        PointField.INT16: 'i2', PointField.UINT16: 'u2',
        PointField.INT32: 'i4', PointField.UINT32: 'u4',
        PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8',
    }
    if field is None or field.count != 1 or field.datatype not in formats:
        raise ValueError('Expected one numeric intensity value per point')
    dtype = np.dtype(('>' if cloud.is_bigendian else '<') + formats[field.datatype])
    if field.offset + dtype.itemsize > cloud.point_step:
        raise ValueError('Intensity field exceeds point_step')
    if cloud.row_step < cloud.width * cloud.point_step:
        raise ValueError('row_step is smaller than the row data')
    if len(cloud.data) < cloud.height * cloud.row_step:
        raise ValueError('PointCloud2 data is shorter than height * row_step')

    result = PointCloud2()
    result.header = copy.deepcopy(cloud.header)
    result.fields = copy.deepcopy(cloud.fields)
    result.height = 1
    result.is_bigendian = cloud.is_bigendian
    result.point_step = cloud.point_step
    result.is_dense = cloud.is_dense
    if cloud.width == 0 or cloud.height == 0:
        return result

    values = np.ndarray((cloud.height, cloud.width), dtype=dtype,
                        buffer=cloud.data, offset=field.offset,
                        strides=(cloud.row_step, cloud.point_step))
    records = np.ndarray((cloud.height, cloud.width, cloud.point_step), dtype=np.uint8,
                         buffer=cloud.data, strides=(cloud.row_step, cloud.point_step, 1))
    selected = records[values == label]
    result.width = len(selected)
    result.row_step = result.width * result.point_step
    result.data = selected.tobytes()
    return result


class SemanticCloudFilter(Node):
    def __init__(self, args):
        super().__init__('semantic_cloud_filter')
        self.declare_parameter('label', args.label)
        self.add_on_set_parameters_callback(self.validate_parameters)
        if not 0 <= self.get_parameter('label').value <= 65534:
            raise ValueError('label must be an integer in 0..65534')
        if self.resolve_topic_name(args.input) == self.resolve_topic_name(args.output):
            raise ValueError('Input and output topics must differ')
        # A reliable publisher also serves RViz's default reliable subscriber.
        self.publisher = self.create_publisher(PointCloud2, args.output, 2)
        # This subscription accepts both reliable and best-effort LiDAR sources.
        self.subscription = self.create_subscription(
            PointCloud2, args.input, self.on_cloud, qos_profile_sensor_data)
        self.get_logger().info(
            f'{args.input} -> {args.output}; intensity label={self.get_parameter("label").value}')

    @staticmethod
    def validate_parameters(parameters):
        for parameter in parameters:
            if parameter.name == 'label' and (
                    parameter.type_ != Parameter.Type.INTEGER or
                    not 0 <= parameter.value <= 65534):
                return SetParametersResult(successful=False,
                                           reason='label must be an integer in 0..65534')
        return SetParametersResult(successful=True)

    def on_cloud(self, cloud):
        try:
            self.publisher.publish(filter_cloud(cloud, self.get_parameter('label').value))
        except (ValueError, TypeError) as error:
            self.get_logger().error(str(error), throttle_duration_sec=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', default='/dome_lidar/points')
    parser.add_argument('--output', default='/dome_lidar/points_selected')
    parser.add_argument('--label', type=int, default=1,
                        help='Semantic class to retain (0..65534; stairs=1)')
    args = parser.parse_args(remove_ros_args()[1:])
    if not 0 <= args.label <= 65534:
        parser.error('--label must be 0..65534')
    rclpy.init()
    node = None
    try:
        node = SemanticCloudFilter(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
