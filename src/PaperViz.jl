# Priorities:
# - Get space parameterization working for 3D Euclidean problem (3D Rosenbrock)
# - Switch to 4D cost (4D Rosenbrock)
# - Get parameterization working for non-Euclidean cost (Bundle Adjustment)
# - Add pulldowns to select colour compression and colourmap
# - Add buttons to save the raw data/figures
# - Get progressive update working
# - Add ability to set priority areas
module PaperViz
using GLMakie
export paperviz, makewhitener, maketrajectory

include("trajectory.jl")
include("whiten.jl")

function paperviz(trajectory::Trajectory, costfunc, whitener=NoWhitening())
    flattened = flattentrajectory(trajectory.deltas)

    # Draw the figure
    GLMakie.activate!(inline=false)
    fig = Figure()
    ax = Axis(fig[1, 1]) 
    scatterlines!(ax, flattened)
    return fig
end



end