#ifndef _TOOLS_H
#define _TOOLS_H

#define Bool uint8_t
#define True 1
#define False 0
static inline int abs(int x) { return (x > 0) ? x : -x; }
static inline int sgn(int x) { return (x > 0) - (x < 0); }
static inline float lpf(float input, float prev_output, float alpha) {
    return alpha * input + (1.0f - alpha) * prev_output;
}
static inline float fmaxf(float a, float b) { return (a > b) ? a : b; }
static inline float fabsf(float x) { return (x > 0.0f) ? x : -x; }
#endif