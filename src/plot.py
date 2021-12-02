import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

vertices = [
    (0, 0),
    (0, 1),
    (1, 1),
    (1, 0),
    (0, 0),
]

codes = [Path.MOVETO]
for i in range(0, len(vertices)-1):  # for the amount of vertices there are draw lines to them in order
    codes.append(Path.LINETO)
path = Path(vertices, codes)

fig, ax = plt.subplots()
patch = patches.PathPatch(path, facecolor='orange', lw=2)
ax.add_patch(patch)
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
plt.show()
