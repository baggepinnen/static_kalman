This demonstration shows how to use juliac to compile a binary that performs Kalman filtering with a model implemented using ModelingToolkit.jl.

We

1. Build a simple model of a continuously stirred tank reactor (CSTR) using MTK
2. Generate a function that computes the dynamics on the form $\dot x = f(x, u, p, t)$ and  extract the julia expression of this function into a separate file.
3. Define a state estimator (UKF) using [LowLevelParticleFilters.jl](https://baggepinnen.github.io/LowLevelParticleFilters.jl/dev/) and discretize the continuous-time dynamics using fixed-step RK4.
4. Use juliac to compile a binary with a `main` function that reads some data from files and performs state estimation along the data trajectory, outputting the loglikelihood of the data.

juliac is not yet available in any released version of Julia and must thus be obtained by downloading the julia source.