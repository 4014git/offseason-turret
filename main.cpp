#include "ascent/Ascent.h"
#include <iostream>
#include <math.h>

#define shooterPhi M_PI / 4 // rad
#define targetHeight 2.64 // m
// to calculate airC: coefficient for mass and air resistance = 1/2 * rho * A * C_D / M = 1/2 * 1.205 kg / m^3 * ( pi * 0.241 m * 0.241 m ) * ~0.5 / ( 0.2693 kg ) ≈ 0.2041 m^-1 
#define airC -0.2041 // m^-1
#define g -9.8 // m / s^2

#define dt 1e-3 // s

void shotStep(const asc::state_t& x, asc::state_t& xd, const double){
  xd[0] = x[3];
  xd[1] = x[4];
  xd[2] = x[5];
  xd[3] = airC*x[3]*sqrt(x[3]*x[3]+x[4]*x[4]+x[5]*x[5]);
  xd[4] = airC*x[4]*sqrt(x[3]*x[3]+x[4]*x[4]+x[5]*x[5]);
  xd[5] = airC*x[5]*sqrt(x[3]*x[3]+x[4]*x[4]+x[5]*x[5]) + g;
  std::cout << x[0] << " " << x[1] << " " << x[2] << std::endl;
}

// https://en.wikipedia.org/wiki/Projectile_motion#Trajectory_of_a_projectile_with_Newton_drag
// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods#Implicit_Runge%E2%80%93Kutta_methods
// state_t = { x, y, z, Vx, Vy, Vz }
double getShotError(double targetX, double targetY, double RVx, double RVy, double shooterTheta, double shooterVelocity)
{
  asc::state_t s = { 0.0, 0.0, 0.0, shooterVelocity * cos(shooterTheta) * cos(shooterPhi) + RVx, shooterVelocity * sin(shooterTheta) * cos(shooterPhi) + RVy, shooterVelocity * sin(shooterPhi) };
  double t = 0.0;

  asc::RK4 integrator;
  asc::Recorder recorder;

  while(s[5] > 0 || s[2] > targetHeight){
    recorder({t, s[0], s[1], s[2], s[3], s[4], s[5]});
    integrator(shotStep, s, t, dt);
  }

  recorder.csv("shot", { "t", "x", "y", "z", "Vx", "vY", "vZ" });

  return sqrt((s[0]-targetX)*(s[0]-targetX) + (s[1]-targetY)*(s[1]-targetY));
}

// double getShotError(double targetX, double targetY, double RVx, double RVy, double shooterTheta, double shooterVelocity){
//   return shooterTheta * shooterTheta + shooterVelocity * shooterVelocity;
// }

// std::pair<double, double> getBestShot(double targetX, double targetY, double RVx, double RVy, double shooterTheta, double shooterVelocity)
// {
//   while(getShotError(targetX, targetY, RVx, RVy, shooterTheta, shooterVelocity) > acceptableError)
//   {
//     double gradX = ( getShotError(targetX, targetY, RVx, RVy, shooterTheta - d, shooterVelocity) - getShotError(targetX, targetY, RVx, RVy, shooterTheta + d, shooterVelocity)) / ( 2 * d );
//     double gradY = ( getShotError(targetX, targetY, RVx, RVy, shooterTheta, shooterVelocity - d) - getShotError(targetX, targetY, RVx, RVy, shooterTheta, shooterVelocity + d)) / ( 2 * d );
//     shooterTheta += gradientCoefficient * gradX;
//     shooterVelocity += gradientCoefficient * gradY;

//     std::cout << getShotError(targetX, targetY, RVx, RVy, shooterTheta, shooterVelocity) << std::endl;
//   }

//   return std::make_pair(shooterTheta, shooterVelocity);
// }

int main() {
  // double x = 5;
  // double y = 5;
  // double RVx = 1;
  // double RVy = -5;
  // double theta = 0;
  // double velocity = 10;
  
  // std::pair<double, double> bestShot = getBestShot(x, y, RVx, RVy, theta, velocity);

  std::cout << getShotError(5,5,1,-5,0,10);
}