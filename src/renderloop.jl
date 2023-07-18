
# Use a triple buffer for the render and compute threads
# Compute thread flips between two buffers
# Render thread swaps between the unused compute buffer and the just rendered buffer

# Re-sort the unfilled locations