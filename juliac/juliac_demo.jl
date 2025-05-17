cd(@__DIR__)
using Pkg
Pkg.activate(".")
module StateEstimator

using LowLevelParticleFilters
using Random, LinearAlgebra, StaticArrays
println(Core.stdout, 0)

const Ts  = 0.005 # sample time
const x0  = SA[0.8, 0.5, 134.14, 130] # Initial state
const u0 = SA[12.0, -4000] # Initial input
const p = SA[0.0, 0.0] # not sure why this is needed...

const lb = SA[0.1, 0.1, 50, 50, 5, -8500]
const ub = SA[2, 2, 142, 140, 100, 0.0]
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

println(Core.stdout, 1)
# const kf   = KalmanFilter(_A, _B, _C, 0, R1, R2, d0, check=false)
const kf = UnscentedKalmanFilter(discrete_dynamics_wrapper, measurement, R1, R2, d0; ny, nu, p=nothing) 
println(Core.stdout, 2)

Base.@ccallable function main()::Cint
    println(Core.stdout, "I'm alive and well")

    y = reinterpret(SVector{4, Float64}, read("data_y.bin")) # Data was written using write("data_y.bin", y)
    u = reinterpret(SVector{2, Float64}, read("data_u.bin")) # Data was written using write("data_u.bin", u)
    @assert length(y) == length(u)
    println(Core.stdout, "I read the data, it has length ", length(y))

    sol = forward_trajectory(kf, u, y)
    println(Core.stdout, "I got loglik = ", sol.ll)

    # xT,_ = smooth(sol, kf, u, y)
    # println(Core.stdout, "I also spent the effort smoothing the data, here's the result: ", xT)
    return zero(Cint)
end

end

# cd(@__DIR__)
# compile using something like
# run(`julia +nightly --project=/home/fredrikb/repos/static_kalman/juliac /home/fredrikb/repos/julia/contrib/juliac.jl --output-exe juliac_demo --trim=unsafe-warn /home/fredrikb/repos/static_kalman/juliac/juliac_demo.jl`)
# run(`ls -ltrh`) # marvel at the smallness of the binary
# run(`./juliac_demo`)


# using Plots
# plot(sol, size=(1900,1200), ploty=false)
# plot!(reduce(hcat, y)', sp=(1:4)', label="y")
# plot!(reduce(hcat, xT)', sp=(1:4)', label="smooth")