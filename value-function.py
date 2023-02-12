class ValueFunction:
    def __init__(self, height, width):
        self.best = 0
        self.bestx = 0
        self.besty = 0
        self.height = height
        self.width = width
        self.map = []
        for i in range(height):
            self.map.append([])
            for j in range(width):
                self.map[i].append(0)
    
    def apply_func(self, func):
        for i in range(self.height):
            for j in range(self.width):
                self.map[i][j] += func(i, j)
                if (i == 0 and j == 0) or self.map[i][j] > self.best:
                    self.best = self.map[i][j]
                    self.bestx = i
                    self.besty = j

    
    def best_coords(self):
        return (self.bestx, self.besty, self.best)

    def value_at(self, x, y):
        return self.map[x][y]
        