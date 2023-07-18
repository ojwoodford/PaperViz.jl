import NLLSsolver
using StaticArrays, Static, PaperViz

# Define the problem
struct SineResidual <: NLLSsolver.AbstractResidual
    freq::Float64
    varind::SVector{2, Int}
end
SineResidual(f, v1, v2) = SineResidual(f, SVector{2, Int}(v1, v2))
NLLSsolver.ndeps(::SineResidual) = static(2) # Residual depends on 2 variables
NLLSsolver.nres(::SineResidual) = static(2) # Residual vector has length 1
NLLSsolver.varindices(res::SineResidual) = res.varind
NLLSsolver.getvars(res::SineResidual, vars::Vector) = vars[res.varind[1]]::Float64, vars[res.varind[2]]::Float64
NLLSsolver.computeresidual(res::SineResidual, x, y) = SVector(x * 0.01 , sin(x * res.freq) - y)
Base.eltype(::SineResidual) = Float64

function constructproblem(ndims)
    problem = NLLSsolver.NLLSProblem(Float64)
    NLLSsolver.addvariable!(problem, 10.0)
    freq = 0.7
    mu = 1.9 ^ (1.0 / ndims)
    for dim = 2:ndims
        NLLSsolver.addvariable!(problem, 10.0)
        NLLSsolver.addresidual!(problem, SineResidual(freq, dim-1, dim))
        freq *= mu
    end
    NLLSsolver.addresidual!(problem, SineResidual(freq, ndims, 1))
    return problem
end
ndims = 6
problem = constructproblem(ndims)

# Optimize and compute the tracjectory
result = NLLSsolver.optimize!(problem, NLLSsolver.NLLSOptions(iterator=NLLSsolver.dogleg, storetrajectory=true, absdcost=0.0, reldcost=0.0))
show(result)

# Compute the trajectory
deltafunc(start, finish) = finish - start
updatefunc(start, delta) = start + SVector{ndims, Float64}(delta)
trajectory = maketrajectory(SVector{ndims, Float64}(ones(ndims)*10.0), result.trajectory, updatefunc, deltafunc)

# Call the visualization
paperviz(trajectory, (state)->cost(state, problem.residuals))
