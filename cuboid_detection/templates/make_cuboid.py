import argparse
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('-L', '--length', default=0.2, type=float, help='cuboid length (m)')
parser.add_argument('-W', '--width', default=0.1, type=float, help='cuboid width (m)')
parser.add_argument('-H', '--height', default=0.075, type=float, help='cuboid height (m)')
parser.add_argument('-d', '--density', default=0.002, type=float, help='number of points per unit (m)')
parser.add_argument('-f', '--filename', default='', type=str, help='output filename')
args = parser.parse_args()

# Specify dimensions in mm
L = args.length
W = args.width
H = args.height

# Handle filename
if not args.filename:
    args.filename = 'template_cuboid_L%d_W%d_H%d_3faces.pcd' % (L * 1000, W * 1000, H * 1000)
elif not args.filename.endswith('.pcd'):
    args.filename += '.pcd'

# PCL header
header = '''# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH %d
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS %d
DATA ascii
'''

# Interpolate points
X = np.arange(-L/2.0, L/2.0, args.density)
Y = np.arange(-W/2.0, W/2.0, args.density)
Z = np.arange(-H/2.0, H/2.0, args.density)
N = [Y.shape[0] * Z.shape[0], X.shape[0] * Z.shape[0], X.shape[0] * Y.shape[0]]

# Function to make (N, 1) face points
def face(xin, yin):
    xout, yout = np.meshgrid(xin, yin)
    return np.matrix([xout.flatten(), yout.flatten()]).T

# Single column filled with same value
column = lambda x: np.full((x[1], 1), x[0])

# Make all 6 faces
faces = [None] * 3
faces[0] = np.hstack((face(X, Y), column([-H/2.0, N[2]])))
faces[1] = np.hstack((face(X, Z)[:, 0], column([-W/2.0, N[1]]), face(X, Z)[:, 1]))
faces[2] = np.hstack((column([-L/2.0, N[0]]), face(Y, Z)[:, 0], face(Y, Z)[:, 1]))
# faces[3] = np.hstack((face(X, Y), column([H, N[2]])))
# faces[4] = np.hstack((face(X, Z)[:, 0], column([W, N[1]]), face(X, Z)[:, 1]))
# faces[5] = np.hstack((column([L, N[0]]), face(Y, Z)[:, 0], face(Y, Z)[:, 1]))

faces = np.vstack(faces)

# Save PCD file
with open(args.filename, 'w') as pcd:
    pcd.write(header % (len(faces), len(faces)))
    for row in faces:
        pcd.write('%f %f %f\n' % (row[0, 0], row[0, 1], row[0, 2]))

print('Saved "%s" with %d points' % (args.filename, len(faces)))
