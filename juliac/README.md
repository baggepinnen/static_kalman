This demonstration shows how to use juliac to compile a binary that performs Kalman filtering with a model implemented using ModelingToolkit.jl.

We

1. Build a simple model of a continuously stirred tank reactor (CSTR) using MTK
2. Generate a function that computes the dynamics on the form $\dot x = f(x, u, p, t)$ and  extract the julia expression of this function into a separate file.
3. Define a state estimator (UKF) using [LowLevelParticleFilters.jl](https://baggepinnen.github.io/LowLevelParticleFilters.jl/dev/) and discretize the continuous-time dynamics using fixed-step RK4.
4. Use juliac to compile a binary with a `main` function that reads some data from files and performs state estimation along the data trajectory, outputting the log-likelihood of the data.

juliac is not yet available in any released version of Julia and must thus be obtained by downloading the julia source.

## Instructions
1. Obtain the juliac driver script by either cloning the [julialang/julia repo](https://github.com/JuliaLang/julia) or by downloading the scripts independently using the instructions provided [in this blog post](https://jbytecode.github.io/juliac/).
2. Clone this repository and instantiate the manifest in this folder (a [special branch of LowLevelParticleFilters is required](https://github.com/baggepinnen/LowLevelParticleFilters.jl/tree/juliac)).
3. Run the script `julia --project cstr_model.jl` to generate the julia file containing the function that computes the dynamics.
4. Invoke the juliac compiler using something like `julia +nightly --project=<...>/static_kalman/juliac <...>/julia/contrib/juliac.jl --output-exe juliac_demo --trim=unsafe-warn <...>/static_kalman/juliac/juliac_demo.jl`. `julia +nightly` assumes that you are using the nightly version of julia installed using _juliaup_, if you have downloaded and compiled julia from source, point to your compiled binary instead. Replace `<...>` with the appropriate paths, i.e., the path to the julia source folder and the folder where you cloned this repository.
5. Run the compiled binary `./juliac_demo` to perform the state estimation and output the log-likelihood of the data.


## Running on Raspberry Pi
Follow exactly the same procedure as above, but directly on the Raspberry Pi instead. I've successfully tested this on a Raspberry Pi model 4 with 4GB of RAM, running Raspbian OS.