#ifndef JULIAC_LIBRARY_H
#define JULIAC_LIBRARY_H

#ifdef __cplusplus
extern "C" {
#endif

// Type definitions for function pointers
typedef void (*jl_init_with_image_t)(const char *bindir, const char *sysimage);
typedef int (*step_t)(double *x, double *u, double *y);

#ifdef __cplusplus
}
#endif

#endif // JULIAC_LIBRARY_H