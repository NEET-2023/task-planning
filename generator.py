import math

def grad_gen(sen_x,sen_y):
    def gradient(x,y):
        return 1/math.dist([sen_x,sen_y],[x,y])
    return gradient

def get_gen(map):
    def get_value(x,y):
        return map[x][y]
    return get_value

