from generator import *
from valuefunction import *
from evaluator import *

vf = ValueFunction(1, 5, 5, zipper_gen([1]))

while True:
    command = input("Input command ")
    args = command.split(' ')
    if args[0] == "sensor":
        func = grad_gen(int(args[1]), int(args[2]), -1)
    else:
        func = get_gen([[1, 2, 3, 4, 5], [4, 5, 6, 7, 8], [7, 8, 9, 10, 11], [1, 2, 3, 4, 5], [4, 5, 6, 7, 8]])
    vf.apply_func(func, 0)
    vf.print_map()
