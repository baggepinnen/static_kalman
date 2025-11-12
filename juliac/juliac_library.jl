module StateEstimator

using LowLevelParticleFilters
using Random, LinearAlgebra, StaticArrays
println(Core.stdout, 0)

const Ts  = 0.005 # sample time
const x0  = SA[0.8, 0.5, 134.14, 130] # Initial state
const u0 = SA[12.0, -4000] # Initial input
const p = nothing

measurement(x,u,p,t) = x # We can measure the full state

include("dynamics.jl") # This defines variable "dynamics"
const discrete_dynamics = LowLevelParticleFilters.rk4(dynamics, Ts)
function discrete_dynamics_wrapper(x,u,p,t) # The silly identity function avoids a segfault in has_ip
    discrete_dynamics(x,u,p,t)
end
discrete_dynamics_wrapper(x0, u0, p, 0.0)



typical_magnitudes = x0

const nx = 4 # Dimension of state
const nu = 2 # Dimension of input
const ny = 4 # Dimension of measurements

const R1 = 0.1*SA[0.0005790557598954602 -6.106518199161526e-5 -0.04490477069514286 -0.012974208985770341; -6.106518199161526e-5 0.000118640098523924 0.01151089963430807 0.003243003007406098; -0.04490477069514286 0.01151089963430807 7.705441811603613 1.8508801117780993; -0.012974208985770341 0.003243003007406098 1.8508801117780993 6.122836707225409]
const R2 = SMatrix{nx,nx}(Diagonal(typical_magnitudes .^ 2 ./ 10))
const d0 = LowLevelParticleFilters.SimpleMvNormal(x0,R1)   # Initial state Distribution

const kf = UnscentedKalmanFilter(discrete_dynamics_wrapper, measurement, R1, R2, d0; ny, nu, p=nothing) 

Base.@ccallable function step(x::Ptr{SVector{nx, Float64}}, ui::Ptr{SVector{nu, Float64}}, yi::Ptr{SVector{ny, Float64}})::Cint
    u = unsafe_load(ui)
    y = unsafe_load(yi)

    kf(u, y) # Update filter state
    unsafe_store!(x, kf.x)
    return Cint(0)
end

end


# cd(@__DIR__)
# Compile using JuliaC:
# run(`juliac --output-lib juliac_library --bundle build --compile-ccallable --trim=unsafe-warn --experimental --project=. juliac_library.jl`)
# The compiled library will be at build/lib/juliac_library.so (or .dylib/.dll)
# gcc -o state_estimation_program test_juliac_library.c -I <julia-path>/include/julia/ -L<julia-path>/lib -ljulia -ldl
# where <julia-path> is your Julia installation directory (e.g., ~/.julia/juliaup/julia-1.12+0)
