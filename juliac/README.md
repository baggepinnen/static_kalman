This demonstration shows how to use JuliaC to compile a binary that performs Kalman filtering with a model implemented using ModelingToolkit.jl.

We include two demos, one where JuliaC compiles an executable, and one where JuliaC creates a shared library that is called from a C program.

For the executable demo, we perform the following steps:

# Executable demo

1. Build a simple model of a continuously stirred tank reactor (CSTR) using MTK
2. Generate a function that computes the dynamics on the form $\dot x = f(x, u, p, t)$ and  extract the julia expression of this function into a separate file.
3. Define a state estimator (UKF) using [LowLevelParticleFilters.jl](https://baggepinnen.github.io/LowLevelParticleFilters.jl/dev/) and discretize the continuous-time dynamics using fixed-step RK4.
4. Use JuliaC to compile a binary with a `main` function that reads some data from files and performs state estimation along the data trajectory, outputting the log-likelihood of the data.

JuliaC is available in Julia 1.12+ and can be installed as a Julia app using the package manager.

## Instructions
1. Install JuliaC by running `pkg> app add JuliaC` in the Julia REPL. Optionally, add `~/.julia/bin` to your PATH to use `juliac` directly from the command line.
2. Clone this repository and instantiate the manifest in this folder (a [special branch of LowLevelParticleFilters is required](https://github.com/baggepinnen/LowLevelParticleFilters.jl/tree/juliac)).
3. Run the script `julia +release --project cstr_model.jl` to generate the julia file containing the function that computes the dynamics.
4. Invoke the JuliaC compiler using:
```bash
juliac --output-exe juliac_demo --bundle build --trim=unsafe-warn --experimental --project=. juliac_demo.jl
```
This creates a portable distribution in the `build/` directory with the executable in `build/bin/juliac_demo`.

5. Run the compiled binary `./build/bin/juliac_demo` to perform the state estimation and output the log-likelihood of the data. The terminal output should look like this:
```
I'm alive and well
I read the data, it has length 30
I got loglik = -238.8285148663645
```


## Running on Raspberry Pi
Follow exactly the same procedure as above, but directly on the Raspberry Pi instead. I've successfully tested this on a Raspberry Pi model 4 with 4GB of RAM, running Raspbian OS.


# Shared library demo
Follow steps 1-3 from the instructions above.

4. Invoke the JuliaC compiler using:
```bash
juliac --output-lib juliac_library --bundle build --compile-ccallable --trim=unsafe-warn --experimental --project=. juliac_library.jl
```
This creates the shared library at `build/lib/juliac_library.so` (or `.dylib` on macOS, `.dll` on Windows). Dependencies like OpenBLAS are placed in `build/lib/julia/`.

5. Compile the C program using:
```bash
gcc -o state_estimation_program test_juliac_library.c -I <julia-path>/include/julia/ -L<julia-path>/lib -ljulia -ldl
```
Where `<julia-path>` is your Julia installation directory (e.g., `~/.julia/juliaup/julia-1.12+0`).

6. Run the compiled C program `./state_estimation_program` to perform the state estimation using the sample inputs defined in the C file.