# Only tested with Matplotlib 3.4.3 as 3.5.1 ran into issues

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
from mpl_toolkits.mplot3d.proj3d import proj_transform
from matplotlib.text import Annotation
from matplotlib.patches import FancyArrowPatch
from inf.disk_ops import load_brain_in_memory
mpl.use('macosx')


class Annotation3D(Annotation):

    def __init__(self, text, xyz, *args, **kwargs):
        super().__init__(text, xy=(0, 0), *args, **kwargs)
        self._xyz = xyz

    def draw(self, renderer):
        x2, y2, z2 = proj_transform(*self._xyz, self.axes.M)
        self.xy = (x2, y2)
        super().draw(renderer)

def _annotate3D(ax, text, xyz, *args, **kwargs):
    '''Add anotation `text` to an `Axes3d` instance.'''

    annotation = Annotation3D(text, xyz, *args, **kwargs)
    ax.add_artist(annotation)


class Arrow3D(FancyArrowPatch):

    def __init__(self, x, y, z, dx, dy, dz, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._xyz = (x, y, z)
        self._dxdydz = (dx, dy, dz)

    def draw(self, renderer):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        super().draw(renderer)


def _arrow3D(ax, x, y, z, dx, dy, dz, *args, **kwargs):
    """Add an 3d arrow to an `Axes3D` instance."""
    arrow = Arrow3D(x, y, z, dx, dy, dz, *args, **kwargs)
    ax.add_artist(arrow)
#
# setattr(Axes3D, 'annotate3D', _annotate3D)
setattr(Axes3D, 'arrow3D', _arrow3D)

# prepare some coordinates
x, y, z = np.indices((20, 20, 20))

# draw cuboids in the top left and bottom right corners, and a link between them
cube1 = (x < 8) & (y < 10) & (z < 1)
cube2 = (x >= 12) & (y < 10) & (z < 1)
# link = abs(x - y) + abs(y - z) + abs(z - x) <= 2

# combine the objects into a single boolean array
voxels = cube1 | cube2

# set the colors of each object
colors = np.empty(voxels.shape, dtype=object)
# colors[link] = 'red'
colors[cube1] = 'blue'
colors[cube2] = 'green'


fig = plt.figure()

ax = fig.add_subplot(111, projection='3d')
ax.voxels(voxels, facecolors=colors, edgecolor='k', alpha=0.2)

# Extract the actual synapse data from connectome
connectome = load_brain_in_memory('/var/folders/gy/4ydq5w4n76jg9zlpjs6mfrlm0000gn/T/2022-02-26_22-01-31_298037_K6UIT1_R/connectome/', ["i__inf", "i__inf"])
cortical_data = {}

__src_cortical_area = "i__inf"
__dst_cortical_area = "i__inf"

cortical_data[__src_cortical_area] = {}
cortical_data[__dst_cortical_area] = {}

vectors = []

cortical_area = __src_cortical_area
for neuron in connectome[cortical_area]:
    if connectome[cortical_area][neuron]["neighbors"] is not None:
        src_neuron_location = connectome[cortical_area][neuron]["soma_location"]
        for dst_neuron in connectome[cortical_area][neuron]["neighbors"]:
            if connectome[cortical_area][neuron]["neighbors"][dst_neuron]["cortical_area"] == __dst_cortical_area:
                dst_neuron_location = connectome[__dst_cortical_area][dst_neuron]["soma_location"]
                vectors.append(src_neuron_location + dst_neuron_location)


print(vectors)


for x in range(3):
    for y in range(3):
        for z in range(3):
            ax.arrow3D(x + 0.5, y + 0.5, z + 0.5,
                       5, 0, 0,
                       mutation_scale=15, ec="red", fc="red")

plt.show()
