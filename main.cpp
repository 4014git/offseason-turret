#include "open3d/Open3D.h"
#include <math.h>
#define shooterPhi M_PI / 4 // rad
#define targetHeight 2.64 // m
// to calculate airC: coefficient for mass and air resistance = 1/2 * rho * A * C_D / M = 1/2 * 1.205 kg / m^3 * ( pi * 0.241 m * 0.241 m ) * ~0.5 / ( 0.2693 kg ) ≈ 0.2041 m^-1 
#define airC 0.2041 // m^-1
#define dT .01 // s
#define dV .001 // m/s
#define dTheta .001 // rad
#define g 9.81 // m / s^2
#define acceptableError .01 // m
#define gradientCoefficient .001

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

void visualizeShot(double targetX, double targetY, double RVx, double RVy, double shooterTheta, double shooterVelocity){
  auto PointCloud = std::make_shared<open3d::geometry::PointCloud>();

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
    PointCloud->points_.push_back({x, y, z});

    Vx += dVx;
    Vy += dVy;
    Vz += dVz;
  }

  auto goal = open3d::geometry::TriangleMesh::CreateTorus(.61, 0.01, 16, 16);
  goal->Translate({targetX, targetY, targetHeight});

  auto robot = open3d::geometry::TriangleMesh::CreateBox();
  robot->Translate({-0.5,-0.5,-0.5});

  open3d::visualization::DrawGeometries({PointCloud, goal, robot});
}

std::pair<double, double> getBestShot(double targetX, double targetY, double RVx, double RVy, double shooterTheta, double shooterVelocity)
{
  double error = getShotError(targetX, targetY, RVx, RVy, shooterTheta, shooterVelocity);

  while(error > acceptableError)
  {
    shooterTheta += error * gradientCoefficient * (error - getShotError(targetX, targetY, RVx, RVy, shooterTheta + dTheta, shooterVelocity)) / dTheta;
    shooterVelocity += error * gradientCoefficient * (error - getShotError(targetX, targetY, RVx, RVy, shooterTheta, shooterVelocity + dV)) / dV;
    error = getShotError(targetX, targetY, RVx, RVy, shooterTheta, shooterVelocity);
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
  visualizeShot(x, y, RVx, RVy, bestShot.first, bestShot.second);
}