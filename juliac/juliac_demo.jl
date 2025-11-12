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
const p = nothing

measurement(x,u,p,t) = x # We can measure the full state

include("dynamics.jl") # This defines variable "dynamics"
const discrete_dynamics = LowLevelParticleFilters.rk4(dynamics, Ts)
discrete_dynamics(x0, u0, p, 0.0)

typical_magnitudes = x0

const nx = 4 # Dimension of state
const nu = 2 # Dimension of input
const ny = 4 # Dimension of measurements

const R1 = 0.1*SA[0.0005790557598954602 -6.106518199161526e-5 -0.04490477069514286 -0.012974208985770341; -6.106518199161526e-5 0.000118640098523924 0.01151089963430807 0.003243003007406098; -0.04490477069514286 0.01151089963430807 7.705441811603613 1.8508801117780993; -0.012974208985770341 0.003243003007406098 1.8508801117780993 6.122836707225409]
const R2 = SMatrix{nx,nx}(Diagonal(typical_magnitudes .^ 2 ./ 10))
const d0 = LowLevelParticleFilters.SimpleMvNormal(x0,R1)   # Initial state Distribution

println(Core.stdout, 1)
# const kf   = KalmanFilter(_A, _B, _C, 0, R1, R2, d0, check=false)
const kf = UnscentedKalmanFilter{false,false,false,false}(discrete_dynamics, measurement, R1, R2, d0; ny, nu, p=nothing) 
println(Core.stdout, 2)

Base.@ccallable function main()::Cint
    println(Core.stdout, "I'm alive and well")

    y = reinterpret(SVector{4, Float64}, read("data_y.bin")) # Data was written using write("data_y.bin", y)
    u = reinterpret(SVector{2, Float64}, read("data_u.bin")) # Data was written using write("data_u.bin", u)
    @assert length(y) == length(u)
    println(Core.stdout, "I read the data, it has length ", length(y))

    sol = forward_trajectory(kf, u, y)
    println(Core.stdout, "I got loglik = ", sol.ll)

    return zero(Cint)
end


# function take_heap_snapshot(filename::String)
#     flags = Base.open_flags(
#         read = true,
#         write = true,
#         create = true,
#         truncate = true,
#         append = false,
#     )
#     nodes = IOStream("<file $filename.nodes>")
#     ccall(:ios_file, Ptr{Cvoid}, (Ptr{UInt8}, Cstring, Cint, Cint, Cint, Cint),
#         nodes.ios, "$filename.nodes", flags.read, flags.write, flags.create, flags.truncate)
#     edges = IOStream("<file $filename.edges>")
#     ccall(:ios_file, Ptr{Cvoid}, (Ptr{UInt8}, Cstring, Cint, Cint, Cint, Cint),
#         edges.ios, "$filename.edges", flags.read, flags.write, flags.create, flags.truncate)
#     strings = IOStream("<file $filename.strings>")
#     ccall(:ios_file, Ptr{Cvoid},(Ptr{UInt8}, Cstring, Cint, Cint, Cint, Cint),
#         strings.ios, "$filename.strings", flags.read, flags.write, flags.create, flags.truncate)
#     json = IOStream("<file $filename.metadata.json>")
#     ccall(:ios_file, Ptr{Cvoid}, (Ptr{UInt8}, Cstring, Cint, Cint, Cint, Cint),
#         json.ios, "$filename.metadata.json", flags.read, flags.write, flags.create, flags.truncate)
#     ccall(:jl_gc_take_heap_snapshot,
#         Cvoid,
#         (Ptr{Cvoid},Ptr{Cvoid},Ptr{Cvoid},Ptr{Cvoid}, Cchar),
#         nodes.handle, edges.handle, strings.handle, json.handle,
#         Cchar(false))
#     ccall(:ios_close, Cint, (Ptr{Cvoid},), nodes.ios)
#     ccall(:ios_close, Cint, (Ptr{Cvoid},), edges.ios)
#     ccall(:ios_close, Cint, (Ptr{Cvoid},), strings.ios)
#     ccall(:ios_close, Cint, (Ptr{Cvoid},), json.ios)
#     return nothing
# end

# take_heap_snapshot("heap_snapshot")

end

@main(args) = StateEstimator.main()

# StateEstimator.main()

# cd(@__DIR__)
# Compile using JuliaC:
# run(`juliac --output-exe juliac_demo --bundle build --trim=unsafe-warn --experimental --project=. juliac_demo.jl`)
# The compiled binary will be at build/bin/juliac_demo
# run(`ls -ltrh build/bin`) # marvel at the smallness of the binary
# run(`./build/bin/juliac_demo`)


# using Plots
# plot(sol, size=(1900,1200), ploty=false)
# plot!(reduce(hcat, y)', sp=(1:4)', label="y")
# plot!(reduce(hcat, xT)', sp=(1:4)', label="smooth")