# Tactile feedback for GHand.
#
# Layout conventions:
# - fingers are ordered: thumb, index, middle, ring, little
# - resultant_force is flattened as [thumb_x, thumb_y, thumb_z,
#                                index_x, index_y, index_z,
#                                middle_x, middle_y, middle_z,
#                                ring_x, ring_y, ring_z,
#                                little_x, little_y, little_z]
# - distributed_force is flattened per finger as
#   [point0_x, point0_y, point0_z, point1_x, point1_y, point1_z, ...]
#   with the same finger order as resultant_force.

float64[] resultant_force
float64[] distributed_force
