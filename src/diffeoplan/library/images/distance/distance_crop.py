from contracts import contract
from diffeoplan.configuration import get_current_config


class DistanceCrop:
    @contract(top='>=0,<0.5', right='>=0,<0.5',
              bottom='>=0,<0.5', left='>=0,<0.5',
              distance='str')
    def __init__(self, top, right, bottom, left, distance):
        self.top = top
        self.right = right
        self.bottom = bottom
        self.left = left
        self.other = get_current_config().distances.instance(distance)
        

    def distance(self, y0, y1):
        y0 = y0.crop(self.top, self.right, self.bottom, self.left)
        y1 = y1.crop(self.top, self.right, self.bottom, self.left)
        return self.other.distance(y0, y1)
