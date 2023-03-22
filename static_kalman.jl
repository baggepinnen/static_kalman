cd(@__DIR__)
using Pkg
Pkg.activate(".")
using StaticTools, StaticArrays, LinearAlgebra, StaticCompiler

struct State{T,nx,nx2}
    x::SVector{nx, T}
    R::SMatrix{nx, nx, T, nx2}
end

"""
    KF{T, nx, nu, ny, nx2, nxnu, nxny, nynu, ny2}
    KF(A, B, C, D, R1, R2)

A Kalman filter for the (discrete-time) dynamics
```math
x(t+1) = A x(t) + B u(t) + w(t)
y(t)   = C x(t) + D u(t) + v(t)
```
where ``w(t)`` and ``v(t)`` are independent zero-mean Gaussian noise processes with covariance matrices ``R1`` and ``R2``, respectively.
"""
struct KF{T,nx,nu,ny,nx2,nxnu,nxny,nynu,ny2}
    A::SMatrix{nx, nx, T, nx2}
    B::SMatrix{nx, nu, T, nxnu}
    C::SMatrix{ny, nx, T, nxny}
    D::SMatrix{ny, nu, T, nynu}
    R1::SMatrix{nx, nx, T, nx2}
    R2::SMatrix{ny, ny, T, ny2}
end

"""
    new_state = predict(state::S, kf::KF, u)

Perform a prediction step of the Kalman filter, i.e., evolve the dynamics one time step forward.
"""
function predict(state::S, kf::KF, u)::S where S <: State
    (; A, B, R1) = kf
    (; x, R) = state
    x = A*x .+ B*u 
    R = symmetrize(A*R*A') + R1
    S(x, R)
end

"""
    new_state, prediction_error = correct(state::S, kf::KF, u, y)

Perform a correction step of the Kalman filter, i.e., update the state estimate based on a measurement.
"""
function correct(state::S, kf::KF, u, y::Y)::Tuple{S,Y} where {S <: State, Y}
    (; C, D, R2) = kf
    (; x, R) = state
    e   = y .- C*x .- D*u # Prediction error

    P   = symmetrize(C*R*C') + R2
    Pᵪ  = cholesky(P)
    K   = (R*C')/Pᵪ
    x  += K*e
    R   = symmetrize((I - K*C)*R)
    S(x, R), e
end

@inline function symmetrize(x::SArray) # For numerical stability
    eltype(x)(0.5) .* (x .+ x')
end


## Create an example system, double integrator
nx = 2 # Number of state variables
nu = 1 # Number of inputs
ny = 1 # Number of outputs
T = Float32 # Numeric type used for the computations

A = @SMatrix T[1 1; 0 1]
B = @SMatrix T[0; 1]
C = @SMatrix T[1 0]
D = @SMatrix T[0]

R1 = @SMatrix T[1 0; 0 1] # Process noise covariance
R2 = @SMatrix T[1]        # Measurement noise covariance

x0 = @SVector T[0, 0]     # Initial state vector
R0 = @SMatrix T[1 0; 0 1] # Initial state covariance

state = State(x0, R0)       # Initial filter state
kf = KF(A, B, C, D, R1, R2) # Kalman filter

## Some sample inputs
u0 = @SVector T[7]  # Input vector
y0 = @SVector randn(T, ny)  # Measurement vector


## Test that the functions work
statep = predict(state, kf, u0)
statec, e = correct(statep, kf, u0, y0)

## Type stable? If not, some variable types are printed in red. Type stability required for compilation
@code_warntype predict(state, kf, u0)
@code_warntype correct(state, kf, u0, y0)

## Check for type instabilities using JET. This looks deeper than @code_warntype above.
using JET
@test_opt predict(state, kf, u0)
@test_opt correct(state, kf, u0, y0)

@test_call predict(state, kf, u0)
@test_call correct(state, kf, u0, y0)

## Compile predict step
argtypes_predict = Tuple{typeof(state), typeof(kf), typeof(u0)}
predict_compiled, path_predict = compile(predict, argtypes_predict, "predict") # Use compile_executable or compile_shlib instead to get binaries

## Test predict step
state2 = predict(state, kf, u0)
state2_compiled = predict_compiled(state, kf, u0)
@assert state2 === state2_compiled

## Compile correct step
argtypes_correct = Tuple{typeof(state), typeof(kf), typeof(u0), typeof(y0)}
correct_compiled, path_correct = compile(correct, argtypes_correct, "correct")

## Test correct step
state3, e = correct(state, kf, u0, y0)
state3_compiled, e_compiled = correct_compiled(state, kf, u0, y0)
@assert state3 === state3_compiled
@assert e === e_compiled



## Making predict callable from C
# We start by defining methods that take Refs instead of the actual values, and unpacks the reference before calling the original function.
# I have thus far not been able to return a Julia object directly, and I thus update the state reference with the result and return an Int
function predict(state::Ref, kf::Ref, u0::Ref)
    state2 = predict(state[], kf[], u0[])
    state[] = state2 # Store result in the incoming state
    0
end

function correct(state::Ref, kf::Ref, u0::Ref, y0::Ref)
    state2, e = correct(state[], kf[], u0[], y0[])
    state[] = state2 # Store result in the incoming state
    e
end

# We then change the signature of the compiled function to instead take these references
using Base: RefValue
argtypes_predict = Tuple{RefValue{typeof(state)}, RefValue{typeof(kf)}, RefValue{typeof(u0)}}
path_predict = compile_shlib(predict, argtypes_predict, "predict"; filename="libpredict") # Use compile_executable or compile_shlib instead to get binaries

# To test that we can call the compiled funciton from C, we make a C-call.
# This step would be performed in C otherwise.
# To do this, we say that we are passing references to our Julia objects, the ccall will create those references automatically
function c_predict(state, kf, u0)
    Libc.Libdl.dlopen(path_predict) do lib
        fn = Libc.Libdl.dlsym(lib, :julia_predict)
        GC.@preserve state kf u0 begin
            res = ccall(fn, Cint, (Ref{State{Float32, 2, 4}}, Ref{KF{Float32, 2, 1, 1, 4, 2, 2, 1, 1}}, Ref{SVector{1, Float32}}), state, kf, u0)
        end
    end
end


##
#=
To actually call this code from C, we need to write some C code.
I'm not interested in coming up with the C structs that represent my Julia types,
so I'll just create one C array for the KF type and one for the State type.

These two lines are needed to make the C code find the shared library. In my case, I had to call
LD_LIBRARY_PATH=/home/fredrikb/Desktop/semi_tmp/static_kalman/predict/
export LD_LIBRARY_PATH

And then compile with the following command:
=#

run(`gcc test.c  -Lpredict -lpredict -o test`)
