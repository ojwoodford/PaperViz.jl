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

subtractcomponent(x, y) = x - y .* (dot(x, y) / dot(y, y))

using LinearAlgebra, StaticArrays

function flattentrajectory(deltas::Vector)
    # Turn an N-d trajectory into a 2-d trajectory, with the following constraints:
    # - Step lengths are preserved
    # - Angles between neighbouring steps are preserved
    @assert !isempty(deltas)
    currpos = zeros(SVector{2, Float64})
    flattened = [currpos]
    sizehint!(flattened, length(deltas)+1)
    angle = 0.0
    normddelta = normalize(deltas[1])
    step = 1
    while true
        # Compute and store the new position, based on direction (angle) and length
        currpos += SVector{2, Float64}(sincos(angle)) .* norm(deltas[step])
        push!(flattened, currpos)

        # Check for termination
        step += 1
        if step > length(deltas)
            break
        end

        # Compute the new direction
        normddelta_ = normalize(deltas[step])
        updateangle = acos(dot(normddelta, normddelta_))
        if step > 2
            updateangle = dot(deltas[step], subtractcomponent(deltas[step-1], deltas[step-2])) > 0 ? -updateangle : updateangle
        end
        normddelta = normddelta_
        angle += updateangle
    end
    return flattened
end
