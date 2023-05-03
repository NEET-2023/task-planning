import math

def zipper_gen(weights):
    def zipper(values):
        ret = 0
        for i in range(len(values)):
            ret += values[i]*weights[i]
        return ret
    return zipper

def grad_gen(sen_x,sen_y, c):
    def gradient(x,y):
        return c/(math.dist([sen_x,sen_y],[x,y]) + 0.01)
    return gradient

def set_func(bad_x,bad_y, value):
    def set(x,y):
        if x == bad_x and y == bad_y:
            return value
        else:
            return 0
    return set

def sin_gen(scalar, shiftx, shifty):
    def sin(x,y):
        return scalar * (sin(x + shiftx) + sin(y + shifty))
    return sin

def get_gen(map):
    def get_value(x,y):
        return map[x][y]
    return get_value

