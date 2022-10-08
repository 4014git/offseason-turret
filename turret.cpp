#include <utility>
#include <math.h>
#define PI 3.14159265
#define shooterAngle PI / 4
#define targetHeight 5.0 // feet TODO: set this to real

int magnitude(std::pair<double, double> p)
{
  return sqrt(p.first * p.first + p.second * p.second);
}

int main()
{
}

double calculateYawAngle(std::pair<double, double> goal, std::pair<double, double> goalVel)
{
  std::pair<double, double> goalFuturePosition = std::make_pair(goal.first + goalVel.first * calcutareAirTime(magnitude(goal)), goal.second + goalVel.second * calcutareAirTime(magnitude(goal)));
  return std::atan2(goalFuturePosition.first, goalFuturePosition.second);
}

int calcutareAirTime(double distance)
{
  return sqrt((tan(shooterAngle) * distance - targetHeight) / 4.9);
}

double calculateExitVelocity(std::pair<double, double> vel, double distance, int airtime)
{
  return (distance / airtime - magnitude(vel)) / cos(shooterAngle);
}
