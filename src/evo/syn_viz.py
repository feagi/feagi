# Only tested with Matplotlib 3.4.3 as 3.5.1 ran into issues

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import json
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
from mpl_toolkits.mplot3d.proj3d import proj_transform
from matplotlib.text import Annotation
from matplotlib.patches import FancyArrowPatch
from inf.disk_ops import load_brain_in_memory, load_genome_in_memory
mpl.use('macosx')

plt.style.use('dark_background')

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

# Extract the actual synapse data from connectome
__src_cortical_area = "i__pro"
__dst_cortical_area = "t__pro"

cortical_areas = [__src_cortical_area, __dst_cortical_area]

connectome_path = '/var/folders/16/7xn3q7792q30rf7yqfvm24sh0000gn/T/2022-02-28_09-29-12_614138_DLW9JN_R/connectome/'
genome_path = connectome_path[:-11] + 'genome_tmp.json'

with open(genome_path, "r") as genome_file:
    genome = json.load(genome_file)

print("Genome:", genome)
connectome = load_brain_in_memory(connectome_path, [__src_cortical_area, __dst_cortical_area])

cortical_data = {}
cortical_dims = list()

# Extract Cortical dimensions from Genome
for cortical_area in cortical_areas:
    cortical_dims.append(genome["blueprint"][cortical_area]["neuron_params"]["block_boundaries"])

print("original cortical dimensions=", cortical_dims)

# Swapping y and z to match Godot axis
tmp = cortical_dims[0][2]
cortical_dims[0][2] = cortical_dims[0][1]
cortical_dims[0][1] = tmp

tmp = cortical_dims[1][2]
cortical_dims[1][2] = cortical_dims[1][1]
cortical_dims[1][1] = tmp


max_x = max(cortical_dims[0][0], cortical_dims[1][0])
max_y = max(cortical_dims[0][1], cortical_dims[1][1])
max_z = max(cortical_dims[0][2], cortical_dims[1][2])

padding = max_x * 2

max_max = max(2 * max_x + padding, max_y, max_z)

cortical_data[__src_cortical_area] = {}
cortical_data[__dst_cortical_area] = {}

vectors = []

# prepare some coordinates
x, y, z = np.indices((max_max, max_max, max_max))

cortical_area_1 = (x < cortical_dims[0][0]) & (y < cortical_dims[0][1]) & (z < cortical_dims[0][2])
cortical_area_2 = (padding + cortical_dims[0][0] <= x) & (x < padding + cortical_dims[0][0] + cortical_dims[1][0]) & (y < cortical_dims[1][1]) & (z < cortical_dims[1][2])

# combine the objects into a single boolean array
voxels = cortical_area_1 | cortical_area_2

# set the colors of each object
colors = np.empty(voxels.shape, dtype=object)

colors[cortical_area_1] = 'blue'
colors[cortical_area_2] = 'green'

fig = plt.figure()

ax = plt.axes(projection='3d')
ax.voxels(voxels, facecolors=colors, edgecolor='gray', alpha=0.2)

# Dark background
fig.set_facecolor('black')
ax.set_facecolor('black')
ax.grid(False)
ax.w_xaxis.set_pane_color((0.0, 0.0, 0.0, 0.0))
ax.w_yaxis.set_pane_color((0.0, 0.0, 0.0, 0.0))
ax.w_zaxis.set_pane_color((0.0, 0.0, 0.0, 0.0))


cortical_area = __src_cortical_area
for neuron in connectome[cortical_area]:
    if connectome[cortical_area][neuron]["neighbors"] is not None:
        src_neuron_location = connectome[cortical_area][neuron]["soma_location"]
        for dst_neuron in connectome[cortical_area][neuron]["neighbors"]:
            if connectome[cortical_area][neuron]["neighbors"][dst_neuron]["cortical_area"] == __dst_cortical_area:
                dst_neuron_location = connectome[__dst_cortical_area][dst_neuron]["soma_location"]
                vectors.append(src_neuron_location + dst_neuron_location)

print("vectors", vectors)

for vector in vectors:
    sx, sy, sz, dx, dy, dz = vector
    ax.arrow3D(sx + 0.5, sy + 0.5, sz + 0.5, padding + dx, dy, dz, mutation_scale=15, ec="red", fc="red")

plt.show()
