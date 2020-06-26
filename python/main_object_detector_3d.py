#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys
sys.path.append('/usr/lib/python2.7/dist-packages')

from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
import numpy as np
import ros_numpy
import rospy

from chainercv.visualizations import vis_bbox
from chainercv.visualizations.colormap import voc_colormap
import message_filters

from geometry_msgs.msg import Point
from realsense2_camera.msg import Extrinsics
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from object_detector_3d.msg import Detection3D, Detection3DResult

from object_detectors import COCOSSD300Detector


def extract_3d_points_in_bbox_frustum(bboxes, points_3d, camera_info):
    """
    Args:
        bboxes (array-like of (B, 4,)-shape):
            B rectangles, in particular bounding boxes which are obtained
            from a 2d object detector. The format of a box is
            [ymin, xmin, ymax, xmax] which represents top-left and
            bottom-right corners of the box. Thus `bboxes` consists of as
            [[ymin_1, xmin_1, ymax_1, xmax_1],
             ...,
             [ymin_B, xmin_B, ymax_B, xmax_B]].
        points_3d (ndarray of (N, 3)-shape):
            A point cloud of N points in 3D coordinates.
        camera_info (sensor_msgs.msg.CameraInfo):
            Intrinsic parameters of the color camera.
    """
    K = np.array(camera_info.K, np.float32).reshape(3, 3)
    K_inv = np.linalg.inv(K)

    bboxes = np.asarray(bboxes)
    bboxes_points_3d = []
    for bbox in bboxes:
        # TL, BL, BR, TR vertices in image coordinates
        vertices_2d = np.array([bbox[[1, 0]],
                                bbox[[1, 2]],
                                bbox[[3, 2]],
                                bbox[[3, 0]]])

        vertices_2d = np.hstack((vertices_2d, np.ones((4, 1), np.float32)))
        vertices_3d = vertices_2d.dot(K_inv.T)

        # Normal vectors of 4 side planes of the frustum, directing outside.
        normal_vectors = np.cross(vertices_3d, vertices_3d[[1, 2, 3, 0]])

        # Compute for each point the inner product to 4 normal vectors.
        # If 4 inner products are all negative or zero,
        # the point is in the frustum.
        is_in_frustum = np.all(np.inner(points_3d, normal_vectors) <= 0, 1)

        bbox_points_3d = points_3d[is_in_frustum]
        bboxes_points_3d.append(bbox_points_3d)
    return bboxes_points_3d


def mean_center_extractor(points):
    return points.mean(0)


def callback(image, pointcloud2, pub_detection_3d, pub_result_image,
             detector_2d, camera_info, extrinsics):
    rgb_image = ros_numpy.image.image_to_numpy(image)

    bboxes, labels, scores = detector_2d.predict(rgb_image)
    bboxes, labels, scores = bboxes[0], labels[0], scores[0]

    # If no objects are detected then an empty Detection3DResult is sent.
    if len(bboxes) == 0:
        pub_detection_3d.publish(Detection3DResult())
        return

    # Coordinate transform of points from depth coord to color coord.
    R = np.array(extrinsics.rotation).reshape(3, 3)
    t = np.array(extrinsics.translation)
    points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pointcloud2)
    points = points.dot(R) + t

    # Compute 3D centroids of the detected objects
    bboxes_points = extract_3d_points_in_bbox_frustum(
        bboxes, points, camera_info)
    centroids = []
    for bbox_points in bboxes_points:
        centroid = mean_center_extractor(bbox_points)
        centroids.append(centroid)
    centroids = np.array(centroids)

    # Store and publish the result
    detection_result = Detection3DResult()
    for label, score, bbox, centroid in zip(labels, scores, bboxes, centroids):
        class_name = detector_2d.class_name_from_number[label]
        y_min, x_min, y_max, x_max = bbox
        position = Point(x=centroid[0], y=centroid[1], z=centroid[2])
        detection_3d = Detection3D(
            class_id=label, class_name=class_name, score=score,
            y_min=y_min, x_min=x_min, y_max=y_max, x_max=x_max,
            position=position)
        detection_result.detections.append(detection_3d)
    detection_result.num_detections = len(detection_result.detections)
    pub_detection_3d.publish(detection_result)

    # Publish the result as an image
    if pub_result_image is not None:
        # Draw a result image as numpy.ndarray
        fig = Figure()
        canvas = FigureCanvasAgg(fig)
        ax = fig.gca()
        vis_bbox(
            rgb_image.transpose(2, 0, 1), bboxes, labels, score=scores,
            label_names=detector_2d.class_name_from_number,
            instance_colors=voc_colormap(labels + 1), linewidth=2.0, ax=ax)
        np.set_printoptions(precision=3)
        for text, centroid in zip(ax.texts, centroids):
            text.set_text('{}\nxyz: {}'.format(text.get_text(), centroid))
        ax.axis('off')
        canvas.draw()
        width, height = (fig.get_size_inches() * fig.get_dpi()).astype(int)
        result_image = np.frombuffer(canvas.tostring_rgb(), dtype=np.uint8)
        result_image = result_image.reshape(height, width, 3)

        result_ros_image = ros_numpy.image.numpy_to_image(result_image, 'rgb8')
        pub_result_image.publish(result_ros_image)


if __name__ == '__main__':
    rospy.init_node('object_detector_3d', anonymous=True)

    topic_camera_info = rospy.get_param('~topic_camera_info')
    topic_extrinsics = rospy.get_param('~topic_extrinsics')
    topic_image_raw = rospy.get_param('~topic_image_raw')
    topic_pointcloud2 = rospy.get_param('~topic_pointcloud2')
    publish_result_image = rospy.get_param('~publish_result_image')
    data_directory = rospy.get_param('~data_directory')

    # Intrinsic parameters of a camera for 2D object detection
    camera_info = rospy.wait_for_message(topic_camera_info, CameraInfo)

    # Extrinsic parameter for coordinate transform from point cloud to camera
    extrinsics = rospy.wait_for_message(topic_extrinsics, Extrinsics)

    sub_image = message_filters.Subscriber(topic_image_raw, Image)
    sub_pointcloud2 = message_filters.Subscriber(
        topic_pointcloud2, PointCloud2)

    pub_detection_3d = rospy.Publisher(
        'object_detection_3d', Detection3DResult, queue_size=10)

    if publish_result_image:
        pub_result_image = rospy.Publisher(
            'object_detection_3d/result_image', Image, queue_size=10)
    else:
        pub_result_image = None

    # Create a 2D object detector
    detector_2d = COCOSSD300Detector()

    ts = message_filters.TimeSynchronizer([sub_image, sub_pointcloud2], 2)
    ts.registerCallback(
        callback, pub_detection_3d, pub_result_image,
        detector_2d, camera_info, extrinsics)

    rospy.spin()
