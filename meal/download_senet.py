from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from food_detector_interface import FoodDetector

if __name__ == '__main__':
    detector = FoodDetector('.',use_cuda=False)