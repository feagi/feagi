#from .SVONode import SVONode
import numpy as np
import struct

MAX_ALLOWED_SVO_DEPTH: int = 32 # You should never be approaching this ANYWAYS, 4b in any dimension!
DEFAULT_PERCENTAGE_AREA_EXPECTED_TO_BE_ACTIVATED: float = 0.30

class SVOTree:

    def __init__(self, depth: int, representing_dimensions: np.ndarray):
        if representing_dimensions.shape != (3,):
            raise ValueError("representing_dimensions must have exactly 3 elements.")

        self._max_depth: int = depth
        self._user_dimension_limit: np.ndarray = representing_dimensions
        self._total_number_nonleaf_nodes: int = 1
        self._number_nodes_per_nonleaf_layer: np.ndarray = np.zeros(self._max_depth, dtype=np.int32)
        self._tree_dimension_limit: int = 2 ** depth
        self._root_node: dict = {}
        self._image_size: np.ndarray = np.array([1,1], dtype=np.int16)
        self._data: bytearray
        self.reset_tree()

    @staticmethod
    def create_SVOTree(target_minimum_dimensions: np.ndarray) -> 'SVOTree':
        if target_minimum_dimensions.shape != (3,):
            raise ValueError("target_minimum_dimensions must have exactly 3 elements.")
        max_dim_size: int = target_minimum_dimensions.max()
        if max_dim_size < 1:
            raise ValueError("Dimensions must be or exceed <1,1,1> for SVO!")
        calculated_depth: int = np.ceil(np.log2(max_dim_size)).astype(int)
        calculated_depth = max(calculated_depth, 1) # # There must be at least 1 later
        if calculated_depth > MAX_ALLOWED_SVO_DEPTH:
            raise ValueError("Dimensions size Exceeded for SVO!")
        return SVOTree(calculated_depth, target_minimum_dimensions)


    def reset_tree(self) -> None:
        """
        Clears all child nodes, to be used again
        """
        self._root_node = {}
        self._number_nodes_per_nonleaf_layer = np.zeros(self._max_depth, dtype=np.int32)
        self._number_nodes_per_nonleaf_layer[0] = 1 # root
        self._recompute_texture_memory(np.ceil(float(self._user_dimension_limit[0] * self._user_dimension_limit[1] * self._user_dimension_limit[2]) * DEFAULT_PERCENTAGE_AREA_EXPECTED_TO_BE_ACTIVATED / 8.0).astype(int))

    def set_nodes(self, node_coordinates: np.ndarray) -> None:

        if node_coordinates.shape[1] != 3:
            raise ValueError("Input coordinates must be a (N,3) array.")

        # start with root node
        # we will calculate the bitmask in a separate step
        node: dict = {}
        node_coordinates = node_coordinates.astype(int)  # enforce type
        node_coordinates[:,2] = self._user_dimension_limit[2] - node_coordinates[:,2] - 1
        num_points: int = node_coordinates.shape[0]
        child_node_references: np.ndarray = np.array([node] * num_points, dtype=object)
        bound: int = 2 ** (self._max_depth - 1)
        direction: np.ndarray = np.zeros(node_coordinates.shape, dtype=int)  # working memory space to calculate vectorized direction of child leaf, essentially all 0 or 1 in x y z directions
        bitpack_fields: np.ndarray = np.zeros(num_points, dtype=int)  # temporarily holds the bitpacked "direction" of the given leaf

        _cache_bitpackval: int = 0

        layer_index: int = 0
        for depth in range(self._max_depth, 1, -1):
            direction[:] = node_coordinates >= bound
            bitpack_fields = direction[:, 0] | (direction[:, 1] << 1) | (direction[:, 2] << 2)

            for point_index in range(num_points):
                _cache_bitpackval = bitpack_fields[point_index]
                if _cache_bitpackval not in child_node_references[point_index]:
                    self._number_nodes_per_nonleaf_layer[layer_index + 1] += 1
                    self._total_number_nonleaf_nodes += 1
                    child_node_references[point_index][_cache_bitpackval] = {}
                child_node_references[point_index] = child_node_references[point_index][_cache_bitpackval]

            node_coordinates[:] -= direction[:] * bound
            bound = bound >> 1
            layer_index += 1
        if self._max_depth == 1: # If the depth is 1, we have one voxel, adn if we are getting it, we know it exists
            bitpack_fields[0] = 1
            self._total_number_nonleaf_nodes = 2


        # the same but for leafs (dont have to worry about next iteration)
        direction[:] = node_coordinates >= bound
        bitpack_fields = direction[:, 0] | (direction[:, 1] << 1) | (direction[:, 2] << 2)

        for point_index in range(num_points):
            _cache_bitpackval = bitpack_fields[point_index]
            if _cache_bitpackval not in child_node_references[point_index]:
                child_node_references[point_index][_cache_bitpackval] = {}



        self._root_node = node

    def export_as_byte_array(self) -> bytes:
        """
        Exports the tree as is as a bytes array ready for ray marching visualization by Brain Visualizer

        2 parts. the first part is the x y size of what the texture should be (each as 2 byte unsigned shorts), the second part is the output array:
            We have to calculate the byte structure: structure is the following (where each node is 4 bytes)
            Node (inclusive byte ranges):
            Byte 0: Bitmask of active children (0-7),
            Byte 1-3: uint24 node count offset to first child node (if applicable), reverse byte order. If this node was a parent of a leaf (which is not exported, only represented by bitmask), then this value will be 0xFFFFFF
            NOTE: leaf nodes are skipped in the output array, as we only need the bitmask of their parent node to ensure their existance (leaf nodes cannot have child nodes)
            example structure, where the number represents the child index from the parent and the letter is a reference to the node
            ( Root ) a
               ├── [0]b
               │   └── [4]d ─ [0]g
               └── [7]c
                   ├── [3]e ─ [3]h
                   └── [7]f
                        ├─── [1]i
                        └─── [7]j
            The above represents 8x8x8 cube with voxels (top down) <0,0,2>, <7,7,4>, <7,6,6>, <7,7,7>
            Nodes are ordered by layer when outputted, skipping leaf nodes. So the output looking by node structure would be [a,b,c,d,e,f]
            As bytes, this would look like the following (bytes, but each node seperated by parentheses, and sections by | for easier reading):
            (0x81|0x010000)(0x10|0x020000)(0x88|0x020000)(0x01|0xFFFFFF)(0x08|0xFFFFFF)(0x82|0xFFFFFF)
        """
        svo_bytes: bytes = self._export_as_bytes_for_shader_image()
        return struct.pack('HH', self._image_size[0], self._image_size[1]) + svo_bytes

    def shrink_memory_usage(self) -> None:
        """
        While the memory allocation can scale up automatically, this function must be called if you intend to scale down memory usage. Intensive so it shouldnt be called often
        """
        self._recompute_texture_memory(self._total_number_nonleaf_nodes)

    def get_tree_max_depth(self) -> int:
        return self._max_depth

    def get_image_dimensions(self) -> np.ndarray:
        return self._image_size


    def _recompute_texture_memory(self, number_of_nodes_to_hold: int) -> None:
        """
        Resizes the bytes array to the number needed, also assuming limitations given texture
        """

        # This is so when using the RF image format, where each pixel is 4 bytes (for 1 float normally), each node only takes 1 pixel
        number_pixels_needed: int = number_of_nodes_to_hold

        # We need to (quickly) find the best rectangle that can hold this data.
        # This following algorithm likely isnt the most memory efficient but is fast
        smallest_square_side: int = np.ceil(np.sqrt(float(number_pixels_needed))).astype(int)
        smallest_rect_side: int = np.ceil(float(number_pixels_needed) / float(smallest_square_side)).astype(int)
        self._max_number_of_nonleaf_nodes_image_can_hold = smallest_square_side * smallest_rect_side

        # we calculated what we need to, now update this object
        self._data = bytearray(self._max_number_of_nonleaf_nodes_image_can_hold * 4) # 4 bytes per pixel
        self._image_size[0] = smallest_square_side
        self._image_size[1] = smallest_rect_side
        return



    def _export_as_bytes_for_shader_image(self) -> bytearray:
        if self._total_number_nonleaf_nodes > self._max_number_of_nonleaf_nodes_image_can_hold:
            self._recompute_texture_memory(self._total_number_nonleaf_nodes)  # expand memory allocation if needed

        if self._max_depth == 0:
            # unique case for single voxel area
            # TODO we should decide how to handle this. Perhaps have the child reference already be 0xFF?
            return bytearray()

        if self._total_number_nonleaf_nodes == 1:
            # Unique case where no nodes were added
            self._data[0] = 0
            self._data[1] = 255
            self._data[2] = 255
            self._data[3] = 255

        else:
            # do all internal nodes
            current_parent_nodes: list[dict] = [self._root_node]
            current_node_array_index: int = 0

            for current_depth in range(self._max_depth - 1):  # loop for all nodes but leaf nodes
                next_parent_nodes: list[dict] = []
                child_count_offset: int = 0
                parent_node_index_of_current_depth = 0
                for current_parent_node in current_parent_nodes:
                    parent_nodes_remaining_at_this_depth: int = self._number_nodes_per_nonleaf_layer[current_depth] - parent_node_index_of_current_depth

                    byte_index: int = current_node_array_index * 4

                    # Calculate bit mask
                    bit_mask: int = 0
                    for val in current_parent_node:
                        bit_mask = bit_mask | (1 << val)
                    self._data[byte_index] = bit_mask

                    child_node_offset_index: int = child_count_offset + parent_nodes_remaining_at_this_depth
                    child_offset_uint24: bytes = struct.pack('i', child_node_offset_index)
                    self._data[byte_index + 1] = child_offset_uint24[0]
                    self._data[byte_index + 2] = child_offset_uint24[1]
                    self._data[byte_index + 3] = child_offset_uint24[2]

                    for child_index in range(8):
                        if bit_mask & (1 << child_index):  # loop over only children that are existing
                            next_parent_nodes.append(current_parent_node[child_index])
                            child_count_offset += 1

                    parent_node_index_of_current_depth += 1

                    current_node_array_index += 1
                current_parent_nodes = next_parent_nodes

            # do not export leaf nodes, instead just add the current parent nodes with the bitmasks, but set their address to #FFFFFF
            for current_parent_node in current_parent_nodes:
                byte_index: int = current_node_array_index * 4
                bit_mask: int = 0
                for val in current_parent_node:
                    bit_mask = bit_mask | (1 << val)
                self._data[byte_index] = bit_mask
                self._data[byte_index + 1] = 255
                self._data[byte_index + 2] = 255
                self._data[byte_index + 3] = 255
                current_node_array_index += 1

        return self._data
