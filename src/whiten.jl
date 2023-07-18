abstract type AbstractWhitening end

struct NoWhitening <: AbstractWhitening
end

struct DenseWhitening <: AbstractWhitening
end

struct StaticWhitening <: AbstractWhitening
end

struct SparseWhitening <: AbstractWhitening
end

struct DiagWhitening <: AbstractWhitening
end

function makewhitener(hessian::AbstractMatrix)
    return DenseWhitening()
end

whiten(transform::AbstractWhitening, delta) = delta