import matplotlib.pyplot as plt
import numpy as np
from matplotlib.path import Path
import matplotlib.patches as patches

vertices = [
    (0, 0),
    (-2.331747612038459, 0.0037701035888241586),
    (-2.2849977440987774, -0.998334561043527),
    (0.003856912691742477, -0.9892889217967787),
    (-0.00336721008113167, -1.994761941915471),
    (-2.3019629927909735, -1.9972753297176014),
    (-2.2875115114263043, -2.999748213606384),
]

codes = [Path.MOVETO]
for i in range(0, len(vertices) - 1):  # for the amount of vertices there are draw lines to them in order
    codes.append(Path.LINETO)
path = Path(vertices, codes)

fig, ax = plt.subplots()
patch = patches.PathPatch(path, facecolor='none', lw=2)
ax.add_patch(patch)
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.spines['left'].set_position('center')
ax.spines['bottom'].set_position('center')
ax.spines['right'].set_color('none')
ax.spines['top'].set_color('none')
plt.xticks(np.arange(-10, 10, 1.0))
plt.yticks(np.arange(-10, 10, 1.0))
ax.xaxis.set_ticks_position('bottom')
ax.yaxis.set_ticks_position('left')
plt.grid()
plt.show()
