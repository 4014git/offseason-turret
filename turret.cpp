#include <math.h>
#define PI 3.14159265
#define shooterAngle PI / 4
// #define targetHeight 5.0; // feet
const double targetHeight = 5.0; // feet WTF change to define later

class threeVector
{
public:
    double x, y, z;
    threeVector(double x, double y, double z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    double distance(threeVector other)
    {
        return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2) + pow(z - other.z, 2));
    }
    void add(threeVector other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
    }
};

int main()
{
    return 0;
}

int calculateTrajectory(threeVector pos, threeVector vel)
{
}

int calcutareAirTime(double distance)
{
    return sqrt((tan(shooterAngle) * distance - targetHeight) / 4.9);
}

int calculateExitVelocity(double distance)
{
}