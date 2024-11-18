#include <stdio.h>
extern int julia_predict(float *state, float *kf, float *u0);

float state[] = {0, 0, 1, 0, 0, 1}; // This initializes the variable that in the julia example code is called "state". It's stored in the order [state.x; vec(state.R)]
float kf[] = {1.0f,0.0f,1.0f,1.0f,0.0f,1.0f,1.0f,0.0f,0.0f,1.0f,0.0f,0.0f,1.0f,1.0f};
float u0[] = {7.0f};

int main(){
    julia_predict(state, kf, u0); // This updates state
    for (int i = 0; i < 6; i++) {
        printf("%.6f ", state[i]);
    }
    printf("\n");
}
