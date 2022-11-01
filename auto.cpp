#include <iostream>
#include <autodiff/forward/dual.hpp>
using namespace autodiff;

// to calculate μ: coefficient for mass and air resistance = 1/2 * rho * A * C_D / M = 1/2 * 1.205 kg / m^3 * ( pi * 0.241 m * 0.241 m ) * ~0.5 / ( 0.2693 kg ) ≈ 0.2041 m^-1 
#define μ -0.2041 // m^-1
#define Φ M_PI / 4 // rad
#define tz 2.64 // m
#define g -9.8 // m / s^2

#define dt 1e-3 // s
#define acceptableError 1e-2 // m
#define learningRate 1e-2 // m^-1

// https://en.wikipedia.org/wiki/Projectile_motion#Trajectory_of_a_projectile_with_Newton_drag
// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
dual getShotError(const dual tx, const dual ty, const dual rvx, const dual rvy, const dual θ, const dual v)
{
  dual 
    x = 0, y = 0, z = 0,
    vx = v*cos(θ)*cos(Φ) + rvx, vy = v*sin(θ)*cos(Φ) + rvy, vz = v*sin(Φ);
  while(vz > 0 || z > tz)
  {
    dual k1x = vx;
    dual k1y = vy;
    dual k1z = vz;
    dual k1vx = μ*vx*sqrt(vx*vx+vy*vy+vz*vz);
    dual k1vy = μ*vy*sqrt(vx*vx+vy*vy+vz*vz);
    dual k1vz = μ*vz*sqrt(vx*vx+vy*vy+vz*vz) + g;

    dual k2x = vx+dt*k1vx/2;
    dual k2y = vy+dt*k1vy/2;
    dual k2z = vz+dt*k1vz/2;
    dual k2vx = μ*(vx+dt*k1vx/2)*sqrt((vx+dt*k1vx/2)*(vx+dt*k1vx/2)+(vy+dt*k1vy/2)*(vy+dt*k1vy/2)+(vz+dt*k1vz/2)*(vz+dt*k1vz/2));
    dual k2vy = μ*(vy+dt*k1vy/2)*sqrt((vx+dt*k1vx/2)*(vx+dt*k1vx/2)+(vy+dt*k1vy/2)*(vy+dt*k1vy/2)+(vz+dt*k1vz/2)*(vz+dt*k1vz/2));
    dual k2vz = μ*(vz+dt*k1vz/2)*sqrt((vx+dt*k1vx/2)*(vx+dt*k1vx/2)+(vy+dt*k1vy/2)*(vy+dt*k1vy/2)+(vz+dt*k1vz/2)*(vz+dt*k1vz/2)) + g;

    dual k3x = vx+dt*k2vx/2;
    dual k3y = vy+dt*k2vy/2;
    dual k3z = vz+dt*k2vz/2;
    dual k3vx = μ*(vx+dt*k2vx/2)*sqrt((vx+dt*k2vx/2)*(vx+dt*k2vx/2)+(vy+dt*k2vy/2)*(vy+dt*k2vy/2)+(vz+dt*k2vz/2)*(vz+dt*k2vz/2));
    dual k3vy = μ*(vy+dt*k2vy/2)*sqrt((vx+dt*k2vx/2)*(vx+dt*k2vx/2)+(vy+dt*k2vy/2)*(vy+dt*k2vy/2)+(vz+dt*k2vz/2)*(vz+dt*k2vz/2));
    dual k3vz = μ*(vz+dt*k2vz/2)*sqrt((vx+dt*k2vx/2)*(vx+dt*k2vx/2)+(vy+dt*k2vy/2)*(vy+dt*k2vy/2)+(vz+dt*k2vz/2)*(vz+dt*k2vz/2)) + g;

    dual k4x = vx+dt*k3vx;
    dual k4y = vy+dt*k3vy;
    dual k4z = vz+dt*k3vz;
    dual k4vx = μ*(vx+dt*k3vx)*sqrt((vx+dt*k3vx)*(vx+dt*k3vx)+(vy+dt*k3vy)*(vy+dt*k3vy)+(vz+dt*k3vz)*(vz+dt*k3vz));
    dual k4vy = μ*(vy+dt*k3vy)*sqrt((vx+dt*k3vx)*(vx+dt*k3vx)+(vy+dt*k3vy)*(vy+dt*k3vy)+(vz+dt*k3vz)*(vz+dt*k3vz));
    dual k4vz = μ*(vz+dt*k3vz)*sqrt((vx+dt*k3vx)*(vx+dt*k3vx)+(vy+dt*k3vy)*(vy+dt*k3vy)+(vz+dt*k3vz)*(vz+dt*k3vz)) + g;

    x += dt*(k1x+2*k2x+2*k3x+k4x)/6;
    y += dt*(k1y+2*k2y+2*k3y+k4y)/6;
    z += dt*(k1z+2*k2z+2*k3z+k4z)/6;
    vx += dt*(k1vx+2*k2vx+2*k3vx+k4vx)/6;
    vy += dt*(k1vy+2*k2vy+2*k3vy+k4vy)/6;
    vz += dt*(k1vz+2*k2vz+2*k3vz+k4vz)/6;
  }

  return sqrt((x-tx)*(x-tx)+(y-ty)*(y-ty));
}

std::pair<dual, dual> getBestShot(dual tx, dual ty, dual rvx, dual rvy, dual θ, dual v)
{
  // gradient decent
  while(getShotError(tx, ty, rvx, rvy, θ, v) > acceptableError)
  {
    dual dθ = derivative(getShotError, wrt(θ), at(tx, ty, rvx, rvy, θ, v));
    dual dv = derivative(getShotError, wrt(v), at(tx, ty, rvx, rvy, θ, v));
    θ -= learningRate*dθ;
    v -= learningRate*dv;
    std::cout << "θ: " << θ << " v: " << v << " error: " << getShotError(tx, ty, rvx, rvy, θ, v) << std::endl;
  }
  return std::make_pair(θ, v);
}

int main()
{
  // test for 5,5,1,-5,0,10
  std::pair<dual, dual> bestShot = getBestShot(5, 5, 1, -5, 0, 10);
  std::cout << "θ: " << bestShot.first << " v: " << bestShot.second << std::endl;
}