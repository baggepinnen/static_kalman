# static_kalman
See [static_kalman.jl](https://github.com/baggepinnen/static_kalman/blob/main/static_kalman.jl)

This repo demonstrates how to use [StaticCompiler.jl](https://github.com/tshort/StaticCompiler.jl) to statically compile a Julia function in a way that makes it callable from a C program.

The example makes use of StaticArrays.jl for stack-allocated arrays with compile-time known dimensions.

The compilation is in the example performed by the function `compile`. This returns a Julia function that calls the compiled code. If you instead would like to obtain a binary, such as an executable or a shared library for integration with C or python code, switch `compile` to `compile_executable` or `compile_shlib` accordingly.

Before calling `compile`, a check using `@code_warntype` is performed. This macro will print the result of type inference in the terminal, and if any type is printed in red, type inference has failed to infer a concrete type somewhere in your program. Static compilation requires that all types are inferred correctly, and any issue reported by `@code_warntype` thus must be addressed before calling `compile`. The packages [JET.jl](https://github.com/aviatesk/JET.jl) and [Cthulhu.jl](https://github.com/JuliaDebug/Cthulhu.jl) contain additional utilities to help you check for inference problems.

## Calling from C
To call the compiled functions from C, we need to
1. Figure out C types that correspond to the Julia types used in the function. In the example code, I use simple flat, homogeneous arrays on the C side. The Julia struct `State` with fields `x` and `R` is thus represented as `[x; vec(R)]` on the C side. Creating corresponding C structs would be a more elegant solution.
2. A way to return the result. In the example, I added a wrapper to the Julia function that takes references as inputs, and stores the result in the incoming argument, like this:
```julia
function predict(state::Ref, kf::Ref, u0::Ref)
    state2 = predict(state[], kf[], u0[])
    state[] = state2 # Store result in the incoming state
    0
end
```
This wrapper simply unpacks the incoming references from the C program and calls the standard julia method. On the C-side, this looks like:
```c
extern int julia_predict(float *state, float *kf, float *u0); // Forward declaration
julia_predict(state, kf, u0); // This updates state
```
See [test.c](https://github.com/baggepinnen/static_kalman/blob/main/test.c) for the full C code.

I compiled with
```bash
LD_LIBRARY_PATH=/home/fredrikb/Desktop/semi_tmp/static_kalman/predict/
export LD_LIBRARY_PATH

gcc test.c  -Lpredict -lpredict -o test`
```

Running
```bash
/home/fredrikb/Desktop/semi_tmp/static_kalman> ./test
0.000000 7.000000 3.000000 1.000000 1.000000 2.000000 
```
results in the same numerical values as the present in the Julia object `state2`:
```julia
julia> state2
State{Float32, 2, 4}(Float32[0.0, 7.0], Float32[3.0 1.0; 1.0 2.0])
```