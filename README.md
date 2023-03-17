# static_kalman
See [static_kalman.jl](https://github.com/baggepinnen/static_kalman/blob/main/static_kalman.jl)

This repo demonstrates how to use [StaticCompiler.jl](https://github.com/tshort/StaticCompiler.jl) to statically compile a Julia function in a wat that makes it callable from a C program.

The example makes use of StaticArrays.jl for stack-allocated arrays with compile-time known dimensions.

The compilation is in the example performed by the function `compile`. This returns a Julia function that calls the compiled code. If you instead would like to obtain a binary, such as an executable or a shared library for integration with C or python code, switch `compile` to `compile_executable` or `compile_shlib` accordingly.

Before calling `compile`, a check using `@code_warntype` is performed. This macro will print the result of type inference in the terminal, and if any type is printed in red, type inference has failed to infer a concrete type somewhere in your program. Static compilation requires that all types are inferred correctly, and any issue reported by `@code_warntype` thus must be addressed before calling `compile`. The packages [JET.jl](https://github.com/aviatesk/JET.jl) and [Cthulhu.jl](https://github.com/JuliaDebug/Cthulhu.jl) contain additional utilities to help you check for inference problems.