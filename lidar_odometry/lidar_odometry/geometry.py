import math


MAX_ERROR = 0.02
MAX_DIST = 0.1
MAX_SIN_ERROR = math.sin(math.radians(60))
BEACON_SIZE = 0.095



class Vec2():
    def __init__(self, x: float, y: float):
        self.x = float(x)
        self.y = float(y)

    def __str__(self) -> str:
        return '{' + f"{self.x:.3f}, {self.y:.3f}" + '}'
    
    def __repr__(self) -> str:
        return f"Vec2({self.x:.3f}, {self.y:.3f})"
    
    def __add__(self, other):
        return Vec2(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other):
        return Vec2(self.x - other.x, self.y - other.y)
    
    def __eq__(self, value) -> bool:
        if isinstance(value, Vec2):
            return self.x == value.x and self.y == value.y
        
        return self is value
    
    def __abs__(self) -> float:
        return math.sqrt( self.x*self.x + self.y*self.y )
    
    def __neg__(self):
        return Vec2(-self.x, -self.y)
    
    def __len__(self):
        return abs(self)
    
    def __mul__(self, other):
        if isinstance(other, float):
            return Vec2(self.x * other, self.y * other)
        
        if isinstance(other, int):
            return Vec2(self.x * float(other), self.y * float(other))
        
        elif isinstance(other, Vec2):
            return self.x*other.x + self.y*other.y
        
        return None
        
    def __truediv__(self, other):
        if isinstance(other, float):
            return Vec2(self.x / other, self.y / other)
        
        if isinstance(other, int):
            return Vec2(self.x / float(other), self.y / float(other))
        
        return None
        
    def cos(self, other=None) -> float:
        if other is None:
            other = Vec2(0,1)
        return self*other / ( abs(self) * abs(other) )
    
    def sin(self, other=None) -> float:
        if other is None:
            other = Vec2(0,1)
        return abs(self.x*other.y - self.y*other.x) / ( abs(self) * abs(other) )
    
    def ang(self, other=None) -> float:
        return math.acos(self.cos(other))
    
    def rotate(self, angle: float):
        sin = math.sin(angle)
        cos = math.cos(angle)

        return Vec2( cos * self.x + sin * self.y,
                     cos * self.y - sin * self.x )
    
    def rotate90(self):
        return Vec2(-self.y, self.x)


class Line():
    def __init__(self, start: Vec2, end: Vec2):
        self.start = start
        self.end = end

    def __str__(self) -> str:
        return f"[{str(self.start)}, {str(self.end)}]"
    
    def __repr__(self) -> str:
        return f"Line({repr(self.start)}, {repr(self.end)})"

    def __abs__(self) -> float:
        return abs(self.end - self.start)
    
    def __len__(self) -> float:
        return abs(self)
    
    def dir(self) -> Vec2:
        return self.end - self.start

    def cos(self, other) -> float:
        return self.dir().cos(other.dir())
    
    def sin(self, other) -> float:
        return self.dir().sin(other.dir())


class PointsLine():
    dir: Vec2
    start: Vec2
    end: Vec2

    def __init__(self, start: Vec2, end: Vec2):
        self.start = start
        self.end = end
        self.points_count = 2

        # Directional vector
        self.dir = end - start

    def __str__(self) -> str:
        return f"[{str(self.start)}, {str(self.end)}, dir: {self.dir}, points: {self.points_count}]"
    
    def __repr__(self) -> str:
        return f"PointsLine({repr(self.start)}, {repr(self.end)})"

    def __abs__(self) -> float:
        return abs(self.end - self.start)
    
    def __len__(self) -> float:
        return abs(self)
    
    def on_line(self, point: Vec2) -> bool:
        # Points
        # P - point
        # S - line start
        # E - line end

        SP = point - self.start
        EP = point - self.end
        ES = self.start - self.end

        abs_EP = abs(EP)
        EP_sin_ES = EP.sin(ES)

        if abs_EP > MAX_DIST:
            return False
        
        if abs_EP * EP_sin_ES> MAX_ERROR:
            return False
        
        if abs(SP) * SP.sin(self.dir) > MAX_ERROR:
            return False
        
        if EP_sin_ES > MAX_SIN_ERROR:
            return False

        return True
    
    def is_start(self, point: Vec2) -> bool:
        # Points
        # P - point
        # S - line start
        # E - line end

        EP = point - self.end
        SP = point - self.start
        SE = self.end - self.start

        abs_SP = abs(SP)
        SP_sin_SE = SP.sin(SE)

        if abs_SP > MAX_DIST:
            return False
        
        if abs_SP * SP_sin_SE > MAX_ERROR:
            return False
        
        if abs(EP) * EP.sin(self.dir) > MAX_ERROR:
            return False
        
        if SP_sin_SE > MAX_SIN_ERROR:
            return False

        return True
    
    def add_to_start(self, point: Vec2) -> bool:
        if not self.is_start(point):
            return False
        
        self.start = point
        self.dir += point - self.start
        self.points_count += 1

        return True
    
    def append(self, next: Vec2) -> bool:
        if not self.on_line(next):
            return False
        
        self.end = next
        self.dir += ( next - self.start ) / self.points_count
        self.points_count += 1

        return True
    
    def combine(self, other) -> bool:
        if not self.on_line(other.start):
            return False
        
        if not self.on_line(other.end):
            return False
        
        
        
        self.end = other.end
        self.dir += other.dir
        self.points_count += other.points_count
        return True
    
    def to_line(self) -> Line:
        return Line(self.start, self.end)

        

class PointsObject():
    def __init__(self, line: Line):
        self.line0 = line
        self.line1 = None
        self.cos = 1.0

    def __str__(self) -> str:
        return f"[ {str(self.line0)}; {str(self.line1)}; matches: {self.matches():.2f} ]"

    def append(self, line: Line):
        if abs(line.start - self.line0.end) > MAX_DIST:
            return False
        
        self.cos = self.line0.cos(line)
        if self.cos > 1 / math.sqrt(2.0):
            self.cos = 1
            return False
        
        self.line1 = line

        return True

    def matches(self) -> float:
        if self.line1 is not None:
            return ( ( 1 - abs(self.cos) ) / ( 25 * abs(BEACON_SIZE - abs(self.line0)) + 1 ) * ( 25 * abs(BEACON_SIZE - abs(self.line0)) + 1 ) ) ** ( 1.0 / 3.0 )
        else:
            return 1 / ( 25 * abs(BEACON_SIZE - abs(self.line0)) + 1 )

