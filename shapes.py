import matplotlib.pyplot as plt
import numpy
from matplotlib.patches import PathPatch
from matplotlib.path import Path

vertices = []
codes = []

codes = [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
vertices = [(-0.1, -0.5), (-0.1, -0.05), (0.3, -0.05), (0.3, -0.5), (0, 0)]
codes += [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
vertices += [(-0.1, 0.5), (-0.1, 0.05), (0.3, 0.05), (0.3, 0.5), (0, 0)]
codes += [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
vertices += [(0.6, -0.1), (0.6, 0.1), (0.9, 0.1), (0.9, -0.1), (0, 0)]

# codes += [Path.MOVETO] + [Path.LINETO]*2 + [Path.CLOSEPOLY]
# vertices += [(4, 4), (5, 5), (5, 4), (0, 0)]

path = Path(vertices, codes)

pathpatch = PathPatch(path, facecolor='none', edgecolor='green')

fig, ax = plt.subplots()
ax.add_patch(pathpatch)
# data = numpy.loadtxt('path_RRT.txt')
data = numpy.loadtxt('path_QRRT.txt')
# fig = plt.figure()
# ax = plt.axes()
ax.plot(data[:,0],data[:,1],'.-')
ax.quiver(data[:,0],data[:,1],numpy.cos(data[:,2]),numpy.sin(data[:,2]))
# plt.show()
ax.set_title('A compound path')

ax.autoscale_view()

plt.show()