#include <iostream>
#include <chrono>
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

#define dx(vx) (vx)
#define dy(vy) (vy)
#define dz(vz) (vz)
#define dvx(vx, vy, vz) (μ*(vx)*sqrt((vx)*(vx)+(vy)*(vy)+(vz)*(vz)))
#define dvy(vx, vy, vz) (μ*(vy)*sqrt((vx)*(vx)+(vy)*(vy)+(vz)*(vz)))
#define dvz(vx, vy, vz) (μ*(vz)*sqrt((vx)*(vx)+(vy)*(vy)+(vz)*(vz)) + g)

// #define k1x dx(vx)
// #define k1y dy(vy)
// #define k1z dz(vz)
// #define k1vx dvx(vx, vy, vz)
// #define k1vy dvy(vx, vy, vz)
// #define k1vz dvz(vx, vy, vz)

// #define k2x dx(vx+dt*k1vx/2)
// #define k2y dy(vy+dt*k1vy/2)
// // #define k2z dz(vz+dt*k1vz/2)
// #define k2vx dvx(vx+dt*k1vx/2, vy+dt*k1vy/2, vz+dt*k1vz/2)
// #define k2vy dvy(vx+dt*k1vx/2, vy+dt*k1vy/2, vz+dt*k1vz/2)
// #define k2vz dvz(vx+dt*k1vx/2, vy+dt*k1vy/2, vz+dt*k1vz/2)

// #define k3x dx(vx+dt*k2vx/2)
// #define k3y dy(vy+dt*k2vy/2)
// #define k3z dz(vz+dt*k2vz/2)
// #define k3vx dvx(vx+dt*k2vx/2, vy+dt*k2vy/2, vz+dt*k2vz/2)
// #define k3vy dvy(vx+dt*k2vx/2, vy+dt*k2vy/2, vz+dt*k2vz/2)
// #define k3vz dvz(vx+dt*k2vx/2, vy+dt*k2vy/2, vz+dt*k2vz/2)

// #define k4x dx(vx+dt*k3vx)
// #define k4y dy(vy+dt*k3vy)
// #define k4z dz(vz+dt*k3vz)
// #define k4vx dvx(vx+dt*k3vx, vy+dt*k3vy, vz+dt*k3vz)
// #define k4vy dvy(vx+dt*k3vx, vy+dt*k3vy, vz+dt*k3vz)
// #define k4vz dvz(vx+dt*k3vx, vy+dt*k3vy, vz+dt*k3vz)

// https://en.wikipedia.org/wiki/Projectile_motion#Trajectory_of_a_projectile_with_Newton_drag
// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
dual getShotError(const dual tx, const dual ty, const dual rvx, const dual rvy, const dual θ, const dual v)
{
  dual 
    x = 0, y = 0, z = 0,
    vx = v*cos(θ)*cos(Φ) + rvx, vy = v*sin(θ)*cos(Φ) + rvy, vz = v*sin(Φ);
  while(vz > 0 || z > tz)
  {
    dual k1x = dx(vx);
    dual k1y = dy(vy);
    dual k1z = dz(vz);
    dual k1vx = dvx(vx, vy, vz);
    dual k1vy = dvy(vx, vy, vz);
    dual k1vz = dvz(vx, vy, vz);
    
    dual k2x = dx(vx+dt*k1vx/2);
    dual k2y = dy(vy+dt*k1vy/2);
    dual k2z = dz(vz+dt*k1vz/2);
    dual k2vx = dvx(vx+dt*k1vx/2, vy+dt*k1vy/2, vz+dt*k1vz/2);
    dual k2vy = dvy(vx+dt*k1vx/2, vy+dt*k1vy/2, vz+dt*k1vz/2);
    dual k2vz = dvz(vx+dt*k1vx/2, vy+dt*k1vy/2, vz+dt*k1vz/2);

    dual k3x = dx(vx+dt*k2vx/2);
    dual k3y = dy(vy+dt*k2vy/2);
    dual k3z = dz(vz+dt*k2vz/2);
    dual k3vx = dvx(vx+dt*k2vx/2, vy+dt*k2vy/2, vz+dt*k2vz/2);
    dual k3vy = dvy(vx+dt*k2vx/2, vy+dt*k2vy/2, vz+dt*k2vz/2);
    dual k3vz = dvz(vx+dt*k2vx/2, vy+dt*k2vy/2, vz+dt*k2vz/2);

    dual k4x = dx(vx+dt*k3vx);
    dual k4y = dy(vy+dt*k3vy);
    dual k4z = dz(vz+dt*k3vz);
    dual k4vx = dvx(vx+dt*k3vx, vy+dt*k3vy, vz+dt*k3vz);
    dual k4vy = dvy(vx+dt*k3vx, vy+dt*k3vy, vz+dt*k3vz);
    dual k4vz = dvz(vx+dt*k3vx, vy+dt*k3vy, vz+dt*k3vz);

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
  // test time the shot error for 5,5,1,-5,0,10
  auto start = std::chrono::high_resolution_clock::now();
  std::cout << "error: " << getShotError(5, 5, 1, -5, 0, 10) << std::endl;
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  std::cout << "time: " << duration.count() << " microseconds" << std::endl;
}