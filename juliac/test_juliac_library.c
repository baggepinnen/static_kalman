#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>
// #include <julia.h>
#include "juliac_library.h"

// Path to julia binary folder
#define JULIA_PATH "/home/fredrikb/.julia/juliaup/julia-1.12.1+0.x64.linux.gnu/bin/" // NOTE: modify this path

// Path to juliac compiled shared object file (created by JuliaC with --bundle flag)
#define LIB_PATH "/home/fredrikb/repos/static_kalman/juliac/build/lib/juliac_library.so" // NOTE: modify this path

int main() {

    // Load the shared library
    printf("Loading juliac_library.so\n");
    void *lib_handle = dlopen(LIB_PATH, RTLD_LAZY);
    if (!lib_handle) {
        fprintf(stderr, "Error: Unable to load library %s\n", dlerror());
        exit(EXIT_FAILURE);
    }
    printf("Loaded juliac_library.so\n");

    // Locate the julia functions function
    printf("Finding symbols\n");
    jl_init_with_image_t jl_init_with_image = (jl_init_with_image_t)dlsym(lib_handle, "jl_init_with_image");

    step_t step = (step_t) dlsym(lib_handle, "step");


    if (jl_init_with_image == NULL || step == NULL) {
        char *error = dlerror();
        fprintf(stderr, "Error: Unable to find symbol: %s\n", error);
        exit(EXIT_FAILURE);
    }
    printf("Found all symbols!\n");

    // Init julia
    jl_init_with_image(JULIA_PATH, LIB_PATH);

    double u[2] = {12.0, -4000.0}; // An example input 
    double y[13][4] = {            // Example measurements 
        {0.8, 0.5, 134.14, 130.0},
        {0.8304508541133587, 0.5423616926909802, 133.80997934840482, 129.76840470493872},
        {0.854892297493315, 0.5782281314517668, 133.54640662626636, 129.5155624202255},
        {0.8746437457867196, 0.6084737766569043, 133.32925448275373, 129.26821401262438},
        {0.8907452727876486, 0.633900466771663, 133.14587223820504, 129.03817975156318},
        {0.9040048795670813, 0.6552330791453617, 132.98796828751938, 128.82957204749871},
        {0.9150448154006944, 0.6731151515486679, 132.8498993400077, 128.64268376909286},
        {0.9243421626387639, 0.6881082328903065, 132.72768227128452, 128.47605832886518},
        {0.9322622557942015, 0.7006950712183633, 132.61841065082953, 128.327568528832},
        {0.9390851841989382, 0.7112855724397544, 132.51990122348073, 128.19495855955526},
        {0.9450263199915339, 0.7202243221926231, 132.4304733276588, 128.07609857958306},
        {0.9502519861820532, 0.7277986869515046, 132.34880675639488, 127.9690883613439},
        {0.9548913110600739, 0.7342468095077114, 132.27384704105046, 127.87228422929715}
    };
    double x[4] = {0, 0, 0, 0};

    for (int t = 0; t < 12; t++) { // Call the step function a number of times
        int result = step(x, u, y[t]);
        for (int i = 0; i < 4; i++) {
            printf("t = %d, x[%d] = %f\n", t, i, x[i]);
        }
    }

    return 0;
}




// Compile this C program using a command like the one below, modified to suit your paths
// gcc -o state_estimation_program test_juliac_library.c -I /home/fredrikb/.julia/juliaup/julia-1.12.1+0.x64.linux.gnu/include/julia/ -L/home/fredrikb/.julia/juliaup/julia-1.12.1+0.x64.linux.gnu/lib -ljulia -ldl