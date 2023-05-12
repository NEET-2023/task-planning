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
dilated_occupancy = cv2.dilate(reduced_occupancy, np.ones((11, 11), np.uint8))
vf = ValueFunction(2, len(dilated_occupancy), len(dilated_occupancy[0]), zipper_gen([0.4, 0.6]))

info0 = sin_gen(100, 1, 2, 0.1, 0.1)
info1 = planar_gen(0, 0, 100, 100)
poi1 = grad_gen(250, 200, 300)
poi2 = grad_gen(260, 175, 300)
vf.apply_func(info0, 0)
vf.apply_func(info1, 1)
vf.apply_func(poi1, 0)
vf.apply_func(poi2, 1)
eval = Evaluator(vf, 5, dilated_occupancy, 1000)
eval.paint_map()

for i in range(5):
    eval.place_one()
    eval.paint_map()
