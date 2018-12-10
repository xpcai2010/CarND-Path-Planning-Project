#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
using namespace std;

struct car {
  bool exist = false;     // if car object is empty
  double id;              // car id
  double x;               // car x value
  double y;               // car y value
  double vx;              // car velocity in the x direction
  double vy;              // car velocity in the y direction
  double speed;           // car velocity 
  double s;               // frenet s coordinate
  double d;               // frenet d coordinate
  int lane;               // car's lane
  double check_car_s;     // where the car will be once the current path id done

};

class Vehicle {
public:

  vector<car> cars;               // vector of cars from sensor fustion
  double ref_vel = 0;             // ego car speed
  int lane = 1;                   // ego car lane
  int target_lane = 1;
  bool change_lane = false;       // flag whether need to change lane
  bool follow_lane = false;         // keep lane status
  double speed_front_car = 0;     // front car speed
  int lane_change_timeout = 0;   // time between two lane changes
  bool safe_dist = true;

  Vehicle();
  virtual ~Vehicle();
  void CarsAround(vector< vector<double> > sensor_fusion, int prev_size);
  car NearestCarAheadInLane(int lane, double car_s);
  bool SafeDistance(double car_s, int lane, double ahead, double behind);
 
};

Vehicle::Vehicle(){}
Vehicle::~Vehicle() {}

void Vehicle::CarsAround(vector< vector<double> > sensor_fusion, int prev_size)
{
    cars.clear();
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        double d = sensor_fusion[i][6];
        if (d >= 0.0 && d <= 12.0)
        {
            car c;
            c.id = sensor_fusion[i][0];
            c.x = sensor_fusion[i][1];
            c.y = sensor_fusion[i][2];
            c.vx = sensor_fusion[i][3];
            c.vy = sensor_fusion[i][4];
            c.s = sensor_fusion[i][5];
            c.d = sensor_fusion[i][6];
            c.speed = sqrt(c.vx * c.vx + c.vy * c.vy);
            c.check_car_s = c.s + (double)prev_size * 0.02 * c.speed; 
            c.lane = (int)floor(c.d/4.0);
            c.exist = true;
            cars.push_back(c);
            
        }
    }
}

car Vehicle::NearestCarAheadInLane(int lane, double car_s)
{
    double distance  = 100000; //large number
    car close_car;
    for (int i = 0; i < cars.size(); i++)
    {
        if ((cars[i].lane == lane) && (cars[i].check_car_s > car_s) && (cars[i].check_car_s - car_s < distance))
        {
            distance = cars[i].check_car_s - car_s;
            close_car = cars[i];
        }
    }
    return close_car;
}

bool Vehicle::SafeDistance(double car_s, int lane, double ahead, double behind)
{
    bool safe =  true;
    for (int i = 0; i < cars.size(); i++)
    {
        if ((cars[i].lane == lane) && (cars[i].check_car_s - car_s < ahead) && (cars[i].check_car_s - car_s > behind))
        {
            safe = false;
            break;
        }
    }
    safe_dist = safe;
    return safe;
}

#endif