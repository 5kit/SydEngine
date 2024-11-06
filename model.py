# libraries used in this program
import math

# Class where every object(vertex) is a point in 3D space relative to the centre
#
class Vertex:
    # Constructor method
    # 
    def __init__(self, pos:tuple):
        self.x = pos[0]
        self.y = pos[1]
        self.z = pos[2]

    # End Constructor

    # Method that takes all rotations and apply them about relative origin
    #
    def rotate(self, rx:float, ry:float, rz:float) -> tuple:
        # Store a temporary position to be rotated
        cx, cy, cz = self.x, self.y, self.z
        # convert angles form degrees to radiants
        rx, ry, rz = rx*math.pi/180, ry*math.pi/180, rz*math.pi/180

        # Apply the Rx matrix
        cx, cy, cz = (cx* 1 + cy* 0            + cz* 0,
                      cx* 0 + cy* math.cos(rx) + cz*-math.sin(rx),
                      cx* 0 + cy* math.sin(rx) + cz* math.cos(rx))
        
        # Apply the Ry matrix
        cx, cy, cz = (cx* math.cos(rx) + cy* 0 + cz* math.sin(rx),
                      cx* 0            + cy* 1 + cz*0,
                      cx*-math.sin(rx) + cy* 0 + cz* math.cos(rx))
        
        # Apply the Rz matrix
        cx, cy, cz = (cx* math.cos(rx) + cy*-math.sin(rx) + rz* 0,
                      cx* math.sin(rx) + cy* math.cos(rx) + rz* 0,
                      cx* 0,           + cy* 0            + rz* 1)
        
        return (rz, ry, rz) # return the adjusted position after rotation
    
    # End method
# End Class

# Class for a 3D model, that holds information about:
# its center position, vertices, faces and its rotations
#
class Model:
    # Constructor method
    # 
    def __init__(self, centre:tuple, vertices:list, faces:list, rotations:list=(0.0,0.0,0.0)):
        self.x = centre[0]
        self.y = centre[1]
        self.z = centre[2]

        self.vertices = vertices    # A list of Vertex objects
        self.faces =    faces       # A list of lists which contain information about its 3 vertices index  
                                    # corresponding to the index of the vertex list as well as the color of the face in hex
        
        # angles(degrees) of rotation in all 3 axis (x, y, z)
        self.rx = rotations[0]      # Rotation about the x axis
        self.ry = rotations[1]      # Rotation about the y axis
        self.rz = rotations[2]      # Rotation about the z axis

    # End Constructor

    # Method to convert world position to screen projection coordinates from a camera angle aswell as 
    #
    def getProjection(self, cameraPos:tuple, cameraAngles:tuple, FOV:int, depth:float) -> tuple:

        points = [] # list to hold all projected points
        for vertex in self.vertices:
            # Formula for display:
            #
            # Theta1 = arctan(dz/dx)
            # Theta2 = cameraAngle[0] - Theta1
            # displayX = depth/tan(Theta2)
            #
            # Theta3 = arctan(dz/dy)
            # Theta4 = cameraAngle[1] - Theta3
            # displayY = depth/tan(Theta4)

            # start by rotating vertex
            vx, vy, vz = vertex.rotate(self.rx, self.ry, self.rz)

            dx = cameraPos[0] - vertex[0]
            dy = cameraPos[1] - vertex[1]
            dz = cameraPos[2] - vertex[2]

            Theta1 = (math.atan(dz/dx))
            Theta3 = math.atan(dz/dy)

            Theta2 = cameraAngles[0] - Theta1
            Theta4 = cameraAngles[1] - Theta3

            displayX = depth/math.tan(Theta2)
            displayY = depth/math.tan(Theta4)

            points.append((displayX, displayY))
        
        # Send information about 3 points and color to send to the rasterization to draw
        # Should also 
        triangles = []
    
    # End method
# End Clas


        
