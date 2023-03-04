from generator import *
from valuefunction import *

class Evaluator:
    def __init__(self, variables, height, width, zipper, sensors):
        self.vf = ValueFunction(variables, height, width, zipper)
        self.sensors = sensors
        self.map = np.zeros((height, width))
        self.placements = np.zeros((height, width))
        for i in range(sensors):
            self.place_one()

    def place_one(self):
        x, y, val = self.vf.best_coords()
        place = grad_gen(x, y, -1)
        self.vf.apply_func_all(place)
        self.placements[x, y] = 1

    def mark_impassable(self, x, y):
        setter = set_func(x, y, -100)
        self.vf.apply_func_all(setter)
        self.map[x, y] = 1
        if self.placements == 1:
            self.placements = 0
            remove = grad_gen(x, y, 1)
            self.vf.apply_func_all(remove)
            self.place_one()