#include <iostream>
#include <math.h>

#define shooterPhi M_PI / 4 // rad
#define targetHeight 2.64 // m
// to calculate airC: coefficient for mass and air resistance = 1/2 * rho * A * C_D / M = 1/2 * 1.205 kg / m^3 * ( pi * 0.241 m * 0.241 m ) * ~0.5 / ( 0.2693 kg ) ≈ 0.2041 m^-1 
#define airC 0.2041 // m^-1
#define g 9.8 // m / s^2

#define dT 1e-3 // s

#define d 1e-5 // m/s
#define acceptableError 1e-2 // m
#define gradientCoefficient 1e-2


// https://en.wikipedia.org/wiki/Projectile_motion#Trajectory_of_a_projectile_with_Newton_drag
// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods#Implicit_Runge%E2%80%93Kutta_methods
// 
double getShotError(double targetX, double targetY, double RVx, double RVy, double shooterTheta, double shooterVelocity)
{
  double Vx = shooterVelocity * cos(shooterTheta) * cos(shooterPhi) + RVx;
  double Vy = shooterVelocity * sin(shooterTheta) * cos(shooterPhi) + RVy;
  double Vz = shooterVelocity * sin(shooterPhi);
  double x = 0;
  double y = 0;
  double z = 0;
  while(Vz > 0 || z > targetHeight)
  {
    // a⃗ = -airC*V*V⃗
    // dV⃗ = -airC*V*V⃗ * dT
    double dVx = -airC*sqrt(Vx*Vx + Vy*Vy + Vz*Vz)*Vx * dT;
    double dVy = -airC*sqrt(Vx*Vx + Vy*Vy + Vz*Vz)*Vy * dT;
    double dVz = (-airC*sqrt(Vx*Vx + Vy*Vy + Vz*Vz)*Vz - g) * dT;

    // trapazoidal integration
    x += (Vx + dVx/2) * dT;
    y += (Vy + dVy/2) * dT;
    z += (Vz + dVz/2) * dT;

    Vx += dVx;
    Vy += dVy;
    Vz += dVz;
  }

  return sqrt((x-targetX)*(x-targetX) + (y-targetY)*(y-targetY));
}

// double getShotError(double targetX, double targetY, double RVx, double RVy, double shooterTheta, double shooterVelocity){
//   return shooterTheta * shooterTheta + shooterVelocity * shooterVelocity;
// }

std::pair<double, double> getBestShot(double targetX, double targetY, double RVx, double RVy, double shooterTheta, double shooterVelocity)
{
  while(getShotError(targetX, targetY, RVx, RVy, shooterTheta, shooterVelocity) > acceptableError)
  {
    double gradX = ( getShotError(targetX, targetY, RVx, RVy, shooterTheta - d, shooterVelocity) - getShotError(targetX, targetY, RVx, RVy, shooterTheta + d, shooterVelocity)) / ( 2 * d );
    double gradY = ( getShotError(targetX, targetY, RVx, RVy, shooterTheta, shooterVelocity - d) - getShotError(targetX, targetY, RVx, RVy, shooterTheta, shooterVelocity + d)) / ( 2 * d );
    shooterTheta += gradientCoefficient * gradX;
    shooterVelocity += gradientCoefficient * gradY;

    std::cout << getShotError(targetX, targetY, RVx, RVy, shooterTheta, shooterVelocity) << std::endl;
  }

  return std::make_pair(shooterTheta, shooterVelocity);
}

int main() {
  double x = 5;
  double y = 5;
  double RVx = 1;
  double RVy = -5;
  double theta = 0;
  double velocity = 10;
  
  std::pair<double, double> bestShot = getBestShot(x, y, RVx, RVy, theta, velocity);
}