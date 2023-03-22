#include <stdio.h>
extern float * julia_predict(float *state, float *kf, float *u0);
// extern float* julia_predict(float state[], float kf[], float u0[]);

float state[] = { -0.79493254f, 0.10277778f, 0.75f, 0.25f, 0.25f, 1.75f };
float kf[] = {1.0f,0.0f,1.0f,1.0f,0.0f,1.0f,1.0f,0.0f,0.0f,1.0f,0.0f,0.0f,1.0f,1.0f};
float u0[] = {0.3677553f};
int main(){
    float *out;
    out = julia_predict(state, kf, u0);
    for (int i = 0; i < 6; i++) {
        printf("%.6f ", out[i]);
    }
}
