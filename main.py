from generator import *
from valuefunction import *
from evaluator import *

vf = ValueFunction(2, 5, 5, zipper_gen([0.4, 0.6]))
eval = Evaluator(vf, 5)

info0 = get_gen([[1, 2, 3, 4, 5], [4, 5, 6, 7, 8], [7, 8, 9, 10, 11], [1, 2, 3, 4, 5], [4, 5, 6, 7, 8]])
info1 = get_gen([[5, 4, 3, 2, 1], [8, 7, 6, 5, 4], [11, 10, 9, 8, 7], [1, 2, 3, 4, 5], [4, 5, 6, 7, 8]])
vf.apply_func(info0, 0)
vf.apply_func(info1, 1)
eval = Evaluator(vf, 5)
eval.print_map()

for i in range(5):
    eval.place_one()
    eval.print_map()
