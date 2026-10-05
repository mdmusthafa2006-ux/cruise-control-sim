#ifndef CAR_H
#define CAR_H

#include <math.h>

#define CAR_MASS   1200.0   /* kg */
#define GRAV       9.81
#define AIR_RHO    1.2
#define CAR_CD     0.32
#define CAR_AREA   2.2      /* m^2 */
#define CAR_CRR    0.012
#define F_MAX      3000.0   /* max engine force at wheels, N */

static double clampd(double x, double lo, double hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

/* v in m/s, throttle 0..1, grade in radians. Returns new speed. */
static double car_step(double v, double throttle, double dt, double grade) {
    throttle = clampd(throttle, 0.0, 1.0);
    double f_engine = throttle * F_MAX;
    double f_drag   = 0.5 * AIR_RHO * CAR_CD * CAR_AREA * v * v;
    double f_roll   = CAR_CRR * CAR_MASS * GRAV;
    double f_grade  = CAR_MASS * GRAV * sin(grade);
    double a = (f_engine - f_drag - f_roll - f_grade) / CAR_MASS;
    v += a * dt;
    return v < 0.0 ? 0.0 : v;
}

#endif
