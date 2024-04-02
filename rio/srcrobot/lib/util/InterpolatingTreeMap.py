from pytreemap import TreeMap

class InterpolatingTreeMap:
    def __init__(self):
        self.m_map = TreeMap()
    
    def put(self,key, value):
        self.m_map.put(key, value)
    
    def get(self, key) -> float:
        value = self.m_map.get(key)
        if value == None:
            ceilingKey = self.m_map.ceiling_key(key)
            floorKey = self.m_map.floor_key(key)

            if ceilingKey == None and floorKey == None:
                return None
            if ceilingKey == None:
                return float(self.m_map.get(floorKey))
            if floorKey == None:
                return float(self.m_map.get(ceilingKey))
            floor = self.m_map.get(floorKey)
            ceiling = self.m_map.get(ceilingKey)
            return self.interpolate(floor, ceiling, self.inverseInterpolate(ceilingKey, key, floorKey))
        else:
            return float(value)
    
    def clear(self):
        self.m_map.clear()

    def interpolate(self, val1, val2, d: float) -> float:
        dydx = float(val2)-float(val1)
        return dydx*d+float(val1)
    
    def inverseInterpolate(self, up, q, down) -> float:
        uppertoLower = float(up)-float(down)
        if uppertoLower <= 0:
            return 0.0
        querytoLower = float(q)-float(down)
        if querytoLower <= 0:
            return 0.0
        return querytoLower/uppertoLower



