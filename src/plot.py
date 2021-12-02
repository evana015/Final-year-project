import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

vertices = [
    (0, 0),
    (-0.001367386770636845, 4.025678719520677),
    (1.0168936340784462, 3.984917594906297),
    (0.9935691928041689, -0.03184290729694881),
    (1.9790004081180403, -0.020690796224752825),
    (2.0247217378103852, 3.951355565817522),
    (3.010700196086432, 3.9843550921238693),
    (2.9994875653518362, -0.0071499794822199566),
    (4.050000319470382, -0.0017988878334018119)
]

codes = [Path.MOVETO]
for i in range(0, len(vertices)-1):  # for the amount of vertices there are draw lines to them in order
    codes.append(Path.LINETO)
path = Path(vertices, codes)

fig, ax = plt.subplots()
patch = patches.PathPatch(path, facecolor='none', lw=2)
ax.add_patch(patch)
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
plt.grid()
plt.show()
