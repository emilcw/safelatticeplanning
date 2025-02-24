// Runge-Kutta 4 integrator. Functional and template-based, requires C++11.
// Modified by Olov Andersson <olov.a.andersson@liu.se>
#pragma once

#include <functional>
#include <Eigen/Geometry>


template <unsigned int N, typename T>
class RK4
{
public:
    using VectorN = Eigen::Matrix<T, N, 1>;

    // Constructor, function ref to \dot{x}(t)
    RK4(std::function<VectorN(float, const VectorN &)> dot_f)
        : dot_f(dot_f) {}

    // Forward-integrate, time t, current state x, and step size h 
    VectorN integrate(float t, VectorN &x, float h)
    {
        VectorN k1, k2, k3, k4;
        k1 = dot_f(t, x);
        k2 = dot_f(t + h / 2, x + (h / 2) * k1);
        k3 = dot_f(t + h / 2, x + (h / 2) * k2);
        k4 = dot_f(t + h, x + h * k3);

        return x + (k1 + 2 * k2 + 2 * k3 + k4) * h / 6;
    }

private:
    std::function<VectorN(float, const VectorN &)> dot_f;
};
