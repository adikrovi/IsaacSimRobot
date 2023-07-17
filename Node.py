class Node:
    def __init__(self, id, x, y, g, h, p):
        self.id = id
        self.g = g
        self.h = h
        self.f = g + h
        self.x = x
        self.y = y
        self.parent = p

    def getG(self):
        return self.g
    
    def getH(self):
        return self.h
    
    def getID(self):
        return self.id
    
    def getF(self):
        return self.f
    
    def getX(self):
        return self.x
    
    def getY(self):
        return self.y
    
    def getParent(self):
        return self.parent
    
    def __eq__(self, other):
        return (self.x == other.x) and (self.y == other.y) and (self.g == other.g) and (self.h == other.h) and (self.id == other.id)

    def __ne__(self, other):
        return not ((self.x == other.x) and (self.y == other.y) and (self.g == other.g) and (self.h == other.h) and (self.id == other.id))

    def __lt__(self, other):
        return self.f < other.f

    def __gt__(self, other):
        return self.f > other.f

    def __le__(self, other):
        return self.f <= other.f

    def __ge__(self, other):
        return self.f >= other.f
    
    

    