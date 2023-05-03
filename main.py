from generator import *
from valuefunction import *
from evaluator import *
import skimage.measure
import cv2

path = 'occupancy_grids/images/rolling_hills_map_10.png'
occupancy_image = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2GRAY)
# cv2.imshow("original occupancy", occupancy_image)
reduced_occupancy = skimage.measure.block_reduce(occupancy_image, (5, 5), np.max)
# cv2.imshow('reduced occupancy', reduced_occupancy)
dilated_occupancy = cv2.dilate(reduced_occupancy, np.ones((7, 7), np.uint8))
vf = ValueFunction(2, len(occupancy_image), len(occupancy_image[0]), zipper_gen([0.4, 0.6]))

#info0 = get_gen([[1, 2, 3, 4, 5], [4, 5, 6, 7, 8], [7, 8, 9, 10, 11], [1, 2, 3, 4, 5], [4, 5, 6, 7, 8]])
#info1 = get_gen([[5, 4, 3, 2, 1], [8, 7, 6, 5, 4], [11, 10, 9, 8, 7], [1, 2, 3, 4, 5], [4, 5, 6, 7, 8]])
#vf.apply_func(info0, 0)
#vf.apply_func(info1, 1)
eval = Evaluator(vf, 5, dilated_occupancy)
eval.print_map()

for i in range(5):
    eval.place_one()
    eval.print_map()
