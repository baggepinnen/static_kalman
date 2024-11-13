module StateEstimator

using LowLevelParticleFilters
using Random, LinearAlgebra, StaticArrays
println(Core.stdout, 0)

const dynamics = function (ˍ₋arg1, ˍ₋arg2, ˍ₋arg3, t)
    SA[(+)((+)((*)((*)(-9.043e9, (exp)((/)(-8560.0, (+)(273.15, ˍ₋arg1[3])))), (abs2)(ˍ₋arg1[1])), (*)((+)(5.1, (*)(-1, ˍ₋arg1[1])), ˍ₋arg2[1])), (*)((*)(-1.287e12, ˍ₋arg1[1]), (exp)((/)(-9758.3, (+)(273.15, ˍ₋arg1[3]))))), (+)((+)((*)((*)(1.287e12, ˍ₋arg1[1]), (exp)((/)(-9758.3, (+)(273.15, ˍ₋arg1[3])))), (*)((*)(-1.0, ˍ₋arg1[2]), ˍ₋arg2[1])), (*)((*)(-1.287e12, ˍ₋arg1[2]), (exp)((/)(-9758.3, (+)(273.15, ˍ₋arg1[3]))))), (+)((+)((*)(30.828516377649326, (+)(ˍ₋arg1[4], (*)(-1, ˍ₋arg1[3]))), (*)(-0.35562611177613196, (+)((+)((*)((*)(-3.7844955e11, (exp)((/)(-8560.0, (+)(273.15, ˍ₋arg1[3])))), (abs2)(ˍ₋arg1[1])), (*)((*)(5.4054e12, ˍ₋arg1[1]), (exp)((/)(-9758.3, (+)(273.15, ˍ₋arg1[3]))))), (*)((*)(-1.4157e13, ˍ₋arg1[2]), (exp)((/)(-9758.3, (+)(273.15, ˍ₋arg1[3]))))))), (*)((+)(130.0, (*)(-1, ˍ₋arg1[3])), ˍ₋arg2[1])), (*)(0.1, (+)((*)(866.88, (+)((*)(-1, ˍ₋arg1[4]), ˍ₋arg1[3])), ˍ₋arg2[2]))]
end

const Ts  = 0.005 # sample time
const x0  = SA[0.8, 0.5, 134.14, 130] # Initial state
const u0 = SA[12.0, -4000] # Initial input
const p = nothing

const lb = SA[0.1, 0.1, 50, 50, 5, -8500]
const ub = SA[2, 2, 142, 140, 100, 0.0]
measurement(x,u,p,t) = x # We can measure the full state
const discrete_dynamics = LowLevelParticleFilters.rk4(dynamics, Ts)
discrete_dynamics(x0, u0, p, 0.0)



typical_magnitudes = x0

const nx = 4 # Dimension of state
const nu = 2 # Dimension of input
const ny = 4 # Dimension of measurements

const R1 = SMatrix{nx,nx}(Diagonal(typical_magnitudes .^ 2 ./ 1000))
const R2 = SMatrix{nx,nx}(Diagonal(typical_magnitudes .^ 2 ./ 10))

const d0 = LowLevelParticleFilters.SimpleMvNormal(x0,R1)   # Initial state Distribution

println(Core.stdout, 1)
# const kf   = KalmanFilter(_A, _B, _C, 0, R1, R2, d0, check=false)
const kf = UnscentedKalmanFilter((x,u,p,t)->discrete_dynamics(x,u,p,t), measurement, R1, R2, d0; ny, nu, p=nothing) # The silly identity function avoids a segfault in has_ip
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
# run(`julia +nightly --project=/home/fredrikb/Desktop/semi_tmp/static_kalman/juliac /home/fredrikb/repos/julia/contrib/juliac.jl --output-exe juliac_demo --trim=unsafe-warn /home/fredrikb/Desktop/semi_tmp/static_kalman/juliac/juliac_demo.jl`)
# run(`ls -ltrh`) # marvel at the smallness of the binary
# run(`./juliac_demo`)


