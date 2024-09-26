#ifndef TYPES_H
#define TYPES_H


#define R2D(x) ((x) * 5.729577951308232087679815e+1)
#define D2R(x) ((x) * 1.745329251994329576923690e-2)

#include <Eigen/Dense>

typedef struct _VehicleState2D_
{
    double t;
    double x;
    double y;
    double yaw;
    double v;
    double w;
} VehicleState2D;

typedef struct _ObservedVehicleState2D_
{
    double t;
    double delta_t;
    double x;
    double y;
    double yaw;
    double vx;
    double vy;
    double ax;
    double ay;
    double w;
    int    motion_type;
} ObservedVehicleState2D;


#endif // TYPES_H