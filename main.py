from generator import *
from valuefunction import *

vf = ValueFunction(3, 3)

while True:
    command = input("Input command ")
    args = command.split(' ')
    if args[0] == "sensor":
        func = grad_gen(int(args[1]), int(args[2]))
    else:
        func = get_gen([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
    vf.apply_func(func)
    vf.print_map()