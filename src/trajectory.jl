struct Trajectory
    states::Vector
    deltas::Vector
    updatefunc
    deltafunc
end

euclideanupdatefunc(start, delta) = start + delta
euclideandeltafunc(start, finish) = finish - start

maketrajectory(states) = maketrajectory(states, euclideanupdatefunc, euclideandeltafunc)
maketrajectory(state, deltas) = maketrajectory(state, deltas, euclideanupdatefunc, euclideandeltafunc)

function maketrajectory(states::Vector, updatefunc, deltafunc)
    # Create the deltas from the steps
    @assert length(states) > 1
    deltas = [deltafunc(states[1], states[2])]
    sizehint!(deltas, length(states)-1)
    for step = 2:length(states)-1
        push!(deltas, deltafunc(states[step], states[step+1]))
    end
    return Trajectory(states, deltas, updatefunc, deltafunc)
end

function maketrajectory(state, deltas::Vector, updatefunc, deltafunc)
    # Create the states from the deltas
    states = [state]
    sizehint!(states, length(deltas)+1)
    for step = 1:length(deltas)
        push!(states, updatefunc(states[step], deltas[step]))
    end
    return Trajectory(states, deltas, updatefunc, deltafunc)
end

subtractcomponent(x, y) = x - y .* dot(x, y)

using LinearAlgebra, StaticArrays

function flattentrajectory(deltas::Vector)
    # Turn an N-d trajectory into a 2-d trajectory, with the following constraints:
    # - Step lengths are preserved
    # - Angles between neighbouring steps are preserved
    @assert !isempty(deltas)

    # Compute the magnitudes and angles
    N = length(deltas)
    magnitudes = map(norm, deltas)
    normdeltas = map(normalize, deltas)
    angles = [acos(dot(normdeltas[i], normdeltas[i+1])) for i in 1:N-1]

    # Construct the 2D trajectory
    currpos = zeros(SVector{2, Float64})
    flattened = [currpos]
    sizehint!(flattened, N+1)
    angle = 0.0
    step = 1
    while true
        # Compute and store the new position, based on direction (angle) and length
        currpos += SVector{2, Float64}(sincos(angle)) .* magnitudes[step]
        push!(flattened, currpos)

        # Check for termination
        if step >= N
            break
        end
        updateangle = angles[step]
        step += 1

        # Compute the new direction
        if step > 2
            updateangle = dot(normdeltas[step], subtractcomponent(normdeltas[step-1], normdeltas[step-2])) > 0 ? -updateangle : updateangle
        end
        angle += updateangle
    end
    return flattened
end
