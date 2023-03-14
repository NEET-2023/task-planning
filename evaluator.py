from generator import *
from valuefunction import *

class Evaluator:
    def __init__(self, eval: ValueFunction, sensors: int):
        self.vf = vf
        self.sensors = sensors
        variables, height, width = vf.get_dims()
        self.map = np.zeros((height, width))
        self.placements = np.zeros((height, width))

    def place_one(self):
        x, y, val = self.vf.best_coords()
        place = grad_gen(x, y, -1)
        self.vf.apply_func_all(place)
        self.placements[x, y] = 1

    def mark_impassable(self, x, y):
        setter = set_func(x, y, -100)
        self.vf.apply_func_all(setter)
        self.map[x, y] = 1
        if self.placements[x, y] == 1:
            self.placements[x, y] = 0
            remove = grad_gen(x, y, 1)
            self.vf.apply_func_all(remove)
            self.place_one()
    
    def print_map(self):
        self.vf.print_map()
    
    def get_dims(self):
        return self.vf.get_dims()

    def get_map(self):
        return self.map
    
    def get_placements(self):
        for i in range(self.sensors):
            self.place_one()
        ret = []
        for x in range(self.height):
            for y in range(self.width):
                if self.placements[x, y] == 1:
                    ret.append((x, y))
        return ret