#ifndef PATH_H
#define PATH_H

#define PATH_1_NUM_DEF (61)

typedef struct
{
    float x;
    float y;
    float theta;
    float plan_v;
}path_t;

extern path_t path_1[PATH_1_NUM_DEF];

#endif