cd(@__DIR__)
using Pkg
Pkg.activate(".")
using StaticTools, StaticArrays, LinearAlgebra, StaticCompiler

struct State{T,nx,nu,nx2}
    x::SVector{nx, T}
    R::SMatrix{nx,nx, T, nx2}
end

"""
    KF{T, nx, nu, ny, nx2, nxnu, nxny, nynu, ny2}
    KF(A, B, C, D, R1, R2)

A Kalman filter for the dynamics
```math
x(t+1) = A x(t) + B u(t) + w(t)
y(t) = C x(t) + D u(t) + v(t)
```
where ``w(t)`` and ``v(t)`` are independent zero-mean Gaussian noise processes with covariance matrices ``R1`` and ``R2``, respectively.
"""
struct KF{T,nx,nu,ny,nx2,nxnu,nxny,nynu,ny2}
    A::SMatrix{nx,nx, T, nx2}
    B::SMatrix{nx,nu, T, nxnu}
    C::SMatrix{ny,nx, T, nxny}
    D::SMatrix{ny,nu, T, nynu}
    R1::SMatrix{nx,nx, T, nx2}
    R2::SMatrix{ny,ny, T, ny2}
end

"""
    new_state = predict(state::S, kf::KF, u)

Perform a prediction step of the Kalman filter, i.e., evolve the dynamics one time step forward.
"""
function predict(state::S, kf::KF, u)::S where S <: State
    (; A,B,R1) = kf
    (; x,R) = state
    x = A*x .+ B*u 
    R = symmetrize(A*R*A') + R1
    S(x, R)
end

"""
    new_state, prediction_error = correct(state::S, kf::KF, u, y)

Perform a correction step of the Kalman filter, i.e., update the state estimate based on a measurement.
"""
function correct(state::S, kf::KF, u, y::Y)::Tuple{S,Y} where {S <: State, Y}
    (; C,D,R2) = kf
    (; x,R) = state
    e   = y .- C*x .- D*u

    P   = symmetrize(C*R*C') + R2
    Pᵪ  = cholesky(P)
    K   = (R*C')/Pᵪ
    x  += K*e
    R   = symmetrize((I - K*C)*R)
    S(x, R), e
end

@inline function symmetrize(x::SArray)
    eltype(x)(0.5) .* (x .+ x')
end


## Create an example system, double integrator
nx = 2
nu = 1
ny = 1
T = Float32

A = @SMatrix T[1 1; 0 1]
B = @SMatrix T[0; 1]
C = @SMatrix T[1 0]
D = @SMatrix T[0]

R1 = @SMatrix T[1 0; 0 1]
R2 = @SMatrix T[1]

x0 = @SVector T[0, 0]
R0 = @SMatrix T[1 0; 0 1]

state = State{T,nx,nx,nx^2}(x0, R0)
kf = KF(A, B, C, D, R1, R2)

## Some sample inputs
u0 = @SVector randn(T, nu)
y0 = @SVector randn(T, ny)


## Test that the functions work
state = predict(state, kf, u0)
state, e = correct(state, kf, u0, y0)

## Type stable?
@code_warntype predict(state, kf, u0)
@code_warntype correct(state, kf, u0, y0)

## Compile predict step
argtypes_predict = Tuple{typeof(state), typeof(kf), typeof(u0)}
predict_compiled, path_predict = compile(predict, argtypes_predict, "predict")

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