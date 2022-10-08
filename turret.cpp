#include <math.h>
#define PI 3.14159265
#define shooterAngle PI / 4
// #define targetHeight 5.0; // feet
const double targetHeight = 5.0; // feet WTF change to define later

class Vec3
{
public:
  double x, y, z;
  Vec3(double x, double y, double z)
  {
    this->x = x;
    this->y = y;
    this->z = z;
  }
  double distance(Vec3 other)
  {
    return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2) +
                pow(z - other.z, 2));
  }
  double magnitude() { return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)); }
  void add(Vec3 other)
  {
    x += other.x;
    y += other.y;
    z += other.z;
  }
};

int main() { return 0; }

int calculateTrajectory(Vec3 pos, Vec3 vel)
{
}

int calcutareAirTime(double distance)
{
  return sqrt((tan(shooterAngle) * distance - targetHeight) / 4.9);
}

double calculateExitVelocity(Vec3 vel, double distance, int airtime)
{
  return (distance / airtime - vel.magnitude()) / cos(shooterAngle);
}
