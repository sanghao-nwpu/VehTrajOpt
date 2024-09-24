#ifndef TYPES_H
#define TYPES_H

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
    double x;
    double y;
    double yaw;
    double rel_vx;
    double rel_vy;
    double rel_ax;
    double rel_ay;
    double w;
    int    motion_type;
} ObservedVehicleState2D;


#endif // TYPES_H