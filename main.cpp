#include <chrono>
#include <iostream>
#include <math.h>

#define shooterPhi M_PI / 4 // rad
#define targetHeight 2.64 // m
// to calculate airC: coefficient for mass and air resistance = 1/2 * rho * A * C_D / M = 1/2 * 1.205 kg / m^3 * ( pi * 0.241 m * 0.241 m ) * ~0.5 / ( 0.2693 kg ) ≈ 0.2041 m^-1 
#define airC -0.2041 // m^-1
#define g -9.8 // m / s^2

#define dt 1e-3 // s

double getShotError(double targetX, double targetY, double RVx, double RVy, double shooterTheta, double shooterVelocity)
{
  double 
    x = 0, y = 0, z = 0,
    Vx = shooterVelocity * cos(shooterTheta) * cos(shooterPhi) + RVx, Vy = shooterVelocity * sin(shooterTheta) * cos(shooterPhi) + RVy, Vz = shooterVelocity * sin(shooterPhi);
  while(Vz > 0 || z > targetHeight)
  {
    double k1x = Vx;
    double k1y = Vy;
    double k1z = Vz;
    double k1Vx = airC*Vx*sqrt(Vx*Vx+Vy*Vy+Vz*Vz);
    double k1Vy = airC*Vy*sqrt(Vx*Vx+Vy*Vy+Vz*Vz);
    double k1Vz = airC*Vz*sqrt(Vx*Vx+Vy*Vy+Vz*Vz) + g;

    double k2x = Vx+dt*k1Vx/2;
    double k2y = Vy+dt*k1Vy/2;
    double k2z = Vz+dt*k1Vz/2;
    double k2Vx = airC*(Vx+dt*k1Vx/2)*sqrt((Vx+dt*k1Vx/2)*(Vx+dt*k1Vx/2)+(Vy+dt*k1Vy/2)*(Vy+dt*k1Vy/2)+(Vz+dt*k1Vz/2)*(Vz+dt*k1Vz/2));
    double k2Vy = airC*(Vy+dt*k1Vy/2)*sqrt((Vx+dt*k1Vx/2)*(Vx+dt*k1Vx/2)+(Vy+dt*k1Vy/2)*(Vy+dt*k1Vy/2)+(Vz+dt*k1Vz/2)*(Vz+dt*k1Vz/2));
    double k2Vz = airC*(Vz+dt*k1Vz/2)*sqrt((Vx+dt*k1Vx/2)*(Vx+dt*k1Vx/2)+(Vy+dt*k1Vy/2)*(Vy+dt*k1Vy/2)+(Vz+dt*k1Vz/2)*(Vz+dt*k1Vz/2)) + g;

    double k3x = Vx+dt*k2Vx/2;
    double k3y = Vy+dt*k2Vy/2;
    double k3z = Vz+dt*k2Vz/2;
    double k3Vx = airC*(Vx+dt*k2Vx/2)*sqrt((Vx+dt*k2Vx/2)*(Vx+dt*k2Vx/2)+(Vy+dt*k2Vy/2)*(Vy+dt*k2Vy/2)+(Vz+dt*k2Vz/2)*(Vz+dt*k2Vz/2));
    double k3Vy = airC*(Vy+dt*k2Vy/2)*sqrt((Vx+dt*k2Vx/2)*(Vx+dt*k2Vx/2)+(Vy+dt*k2Vy/2)*(Vy+dt*k2Vy/2)+(Vz+dt*k2Vz/2)*(Vz+dt*k2Vz/2));
    double k3Vz = airC*(Vz+dt*k2Vz/2)*sqrt((Vx+dt*k2Vx/2)*(Vx+dt*k2Vx/2)+(Vy+dt*k2Vy/2)*(Vy+dt*k2Vy/2)+(Vz+dt*k2Vz/2)*(Vz+dt*k2Vz/2)) + g;

    double k4x = Vx+dt*k3Vx;
    double k4y = Vy+dt*k3Vy;
    double k4z = Vz+dt*k3Vz;
    double k4Vx = airC*(Vx+dt*k3Vx)*sqrt((Vx+dt*k3Vx)*(Vx+dt*k3Vx)+(Vy+dt*k3Vy)*(Vy+dt*k3Vy)+(Vz+dt*k3Vz)*(Vz+dt*k3Vz));
    double k4Vy = airC*(Vy+dt*k3Vy)*sqrt((Vx+dt*k3Vx)*(Vx+dt*k3Vx)+(Vy+dt*k3Vy)*(Vy+dt*k3Vy)+(Vz+dt*k3Vz)*(Vz+dt*k3Vz));
    double k4Vz = airC*(Vz+dt*k3Vz)*sqrt((Vx+dt*k3Vx)*(Vx+dt*k3Vx)+(Vy+dt*k3Vy)*(Vy+dt*k3Vy)+(Vz+dt*k3Vz)*(Vz+dt*k3Vz)) + g;

    x += dt*(k1x+2*k2x+2*k3x+k4x)/6;
    y += dt*(k1y+2*k2y+2*k3y+k4y)/6;
    z += dt*(k1z+2*k2z+2*k3z+k4z)/6;
    Vx += dt*(k1Vx+2*k2Vx+2*k3Vx+k4Vx)/6;
    Vy += dt*(k1Vy+2*k2Vy+2*k3Vy+k4Vy)/6;
    Vz += dt*(k1Vz+2*k2Vz+2*k3Vz+k4Vz)/6;
  }

  return sqrt((x-targetX)*(x-targetX)+(y-targetY)*(y-targetY));
}

int main()
{
  clock_t start = clock();
  std::cout << getShotError(5,5,1,-5,0,10) << std::endl;
  clock_t end = clock();
  std::cout << "Time: " << (end-start)/(double)CLOCKS_PER_SEC << std::endl;
}