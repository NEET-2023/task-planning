import numpy as np
import rospy

class ValueFunction:
    def __init__(self, variables: int, height: int, width: int, zipper):
        self.best = 0
        self.bestx = 0
        self.besty = 0
        self.variables = variables
        self.height = height
        self.width = width
        self.map = np.zeros((variables, height, width))
        self.zipper = zipper

    def get_dims(self):
        return (self.variables, self.height, self.width)

    def apply_func(self, func, v):
        for i in range(self.height):
            for j in range(self.width):
                self.map[v, i, j] += func(i, j)
                subvalues = self.map[:,i,j]
                info = self.zipper(subvalues)
                if (i == 0 and j == 0) or info > self.best:
                    self.best = info
                    self.bestx = i
                    self.besty = j

    def apply_func_all(self, func):
        for v in range(self.variables):
            self.apply_func(func, v)

    
    def best_coords(self):
        return (self.bestx, self.besty, self.best)

    def value_at(self, x, y):
        return self.zipper(self.map[:, x, y])
    
    def print_map(self):
        for i in range(self.height):
            for j in range(self.width):
                print(self.zipper(self.map[:, i, j]), end =" ")
            print("")
        print("")
