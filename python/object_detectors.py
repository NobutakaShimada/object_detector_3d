#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import os

import rospy

from chainercv.datasets import coco_bbox_label_names, voc_bbox_label_names
from chainercv.links import SSD300


#_ssd300_coco_file_path = r'./12_ssd300/model_iter_240000'
_ssd300_coco_file_path = r'12_ssd300/iter_240000.model'


class BaseObjectDetector(object):
    """Interface class of 2D object detector.

    If one want to use one's own detector, this class can be used as
    an adapter by implementing `predict` method and `class_name_from_number`
    property.
    """
    def predict(self, image):
        """
        Arg:
            image (numpy.ndarray):
                An image to be applied to object detection. The image must be
                color image of RGB, uint8 and (H, W, 3)-shape, where H and W
                denote height and width.

        Return:
            detection result (tuple of 3 `ndarray`s):
                Single detection is represented by a bounding box (bbox),
                a class number of the object and a detection score.
                For an image, in general, zero or multiple objects can be
                detected. Therefore, a detection result consists of
                a tuple of 3 ndarrays, in particular
                - bboxes: (N, 4)-shaped float ndarray, where `N` denotes the
                number of detected objects and a bbox is represented as
                2 coordinates of diagonal corners by
                `[ymin, xmin, ymax, xmax]`.
                - classes: (N,)-shaped integer ndarray that represents the
                number of class of the object.
                - scores: (N,)-shaped float ndarray that represents the
                certainty of its detection.
                `
        """
        raise NotImplementedError()

    @property
    def class_name_from_number(self):
        """A mapping from class number to class name.

        Return:
            mapping (list of `str`s):
                An non-negative integer as a class number is used as
                an index of the list to refer the corresponding string of
                the name of the class.
        """
        raise NotImplementedError()


class COCOSSD300Detector(BaseObjectDetector):
    def __init__(self, model_file_path=None, score_thresh=0.4):
        if model_file_path is None:
            data_directory = rospy.get_param('~data_directory')
            model_file_path = os.path.join(data_directory,
                                           _ssd300_coco_file_path)

        n_fg_class = len(coco_bbox_label_names)
        self.model = SSD300(n_fg_class=n_fg_class,
                            pretrained_model=model_file_path)

        self.model.use_preset('evaluate')
        self.model.score_thresh = score_thresh

    def predict(self, image):
        image = image.transpose(2, 0, 1)
        return self.model.predict([image])

    @property
    def class_name_from_number(self):
        return coco_bbox_label_names


class VOCSSD300Detector(BaseObjectDetector):
    def __init__(self):
        self.model = SSD300(pretrained_model='voc0712')
        self.model.use_preset('evaluate')

    def predict(self, image):
        image = image.transpose(2, 0, 1)
        return self.model.predict([image])

    @property
    def class_name_from_number(self):
        return voc_bbox_label_names
