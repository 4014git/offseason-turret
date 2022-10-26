#include "open3d/Open3D.h"
#include <math.h>
#include <iostream>
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
  // auto PointCloud = std::make_shared<open3d::geometry::PointCloud>();

  double Vx = shooterVelocity * cos(shooterTheta) * cos(shooterPhi) + RVx;
  double Vy = shooterVelocity * sin(shooterTheta) * cos(shooterPhi) + RVy;
  double Vz = shooterVelocity * sin(shooterPhi);
  double x = 0;
  double y = 0;
  double z = 0;
  while(Vz > 0 || z > targetHeight)
  {
    double V = sqrt(Vx*Vx + Vy*Vy + Vz*Vz);
    // a_D = airC*V*V
    // aX = -a_D * Vx / V = -airC*V*Vx
    double dVx = -airC*V*Vx * dT;
    double dVy = -airC*V*Vy * dT;
    double dVz = -airC*V*Vz * dT - g * dT;
    Vx += dVx;
    Vy += dVy;
    Vz += dVz;
    // trapazoidal integration
    x += (Vx + dVx/2) * dT;
    y += (Vy + dVy/2) * dT;
    z += (Vz + dVz/2) * dT;
    // PointCloud->points_.push_back({x, y, z});
  }

  // auto goal = open3d::geometry::TriangleMesh::CreateTorus(.61, 0.01, 16, 16);
  // goal->Translate({targetX, targetY, targetHeight});

  // auto robot = open3d::geometry::TriangleMesh::CreateBox();
  // robot->Translate({-0.5,-0.5,-0.5});

  // open3d::visualization::DrawGeometries({PointCloud, goal, robot});

  return sqrt((x-targetX)*(x-targetX) + (y-targetY)*(y-targetY));
}

std::pair<double, double> getBestShot(double targetX, double targetY, double RVx, double RVy, double shooterTheta, double shooterVelocity)
{
  double error = getShotError(targetX, targetY, RVx, RVy, shooterTheta, shooterVelocity);

  while(error > acceptableError)
  {
    shooterTheta += error * gradientCoefficient * (error - getShotError(targetX, targetY, RVx, RVy, shooterTheta + dTheta, shooterVelocity)) / dTheta;
    shooterVelocity += error * gradientCoefficient * (error - getShotError(targetX, targetY, RVx, RVy, shooterTheta, shooterVelocity + dV)) / dV;
    error = getShotError(targetX, targetY, RVx, RVy, shooterTheta, shooterVelocity);
    // std::cout << "error: " << error << " theta: " << shooterTheta << " velocity: " << shooterVelocity << std::endl;
  }

  return std::make_pair(shooterTheta, shooterVelocity);
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
    double V = sqrt(Vx*Vx + Vy*Vy + Vz*Vz);
    // a_D = airC*V*V
    // aX = -a_D * Vx / V = -airC*V*Vx
    double dVx = -airC*V*Vx * dT;
    double dVy = -airC*V*Vy * dT;
    double dVz = -airC*V*Vz * dT - g * dT;
    Vx += dVx;
    Vy += dVy;
    Vz += dVz;
    // trapazoidal integration
    x += (Vx + dVx/2) * dT;
    y += (Vy + dVy/2) * dT;
    z += (Vz + dVz/2) * dT;
    PointCloud->points_.push_back({x, y, z});
  }

  auto goal = open3d::geometry::TriangleMesh::CreateTorus(.61, 0.01, 16, 16);
  goal->Translate({targetX, targetY, targetHeight});

  auto robot = open3d::geometry::TriangleMesh::CreateBox();
  robot->Translate({-0.5,-0.5,-0.5});

  open3d::visualization::DrawGeometries({PointCloud, goal, robot});
}

int main() {
  std::cout << "calculating best shot" << std::endl;
  std::pair<double, double> bestShot = getBestShot(5, 5, 0, 0, 0, 10);
  std::cout << "visualizing best shot" << std::endl;
  visualizeShot(5, 5, 0, 0, bestShot.first, bestShot.second);
}