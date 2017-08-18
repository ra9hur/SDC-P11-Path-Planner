#ifndef PATHPLANNER_H
#define PATHPLANNER_H
#include <vector>
#include <string>

using namespace std;

class PathPlanner {

    public: 

        // List of cars surrounding ego car
        vector<vector<double>> left_lane_front;
		vector<vector<double>> left_lane_back;
		vector<vector<double>> mid_lane_front;
		vector<vector<double>> mid_lane_back;
		vector<vector<double>> right_lane_front;
		vector<vector<double>> right_lane_back;

        // Maintain vehicle lane information
        // double d;
        bool CHG_LEFT;
        bool CHG_RIGHT;

        // Current lane of the vehicle
        char current_lane;
        int lane;

        // Constructor 
        PathPlanner(); 

        // Destructor
        ~PathPlanner() {}

        // Maintain a list of all the cars surrounding the one we drive, updated for each timesteps
        void MaintainListSurroundingCars(vector<vector<double>> sensor_fusion, double car_s);

        // Change ego car's speed to the average speed of cars in the lane
        double AverageLaneSpeed();
        
        // To check if surrounding cars are moving into the lane that ego car is currently on
        // If yes, stay cautious and slow down
        bool CheckOtherCarLC(double car_speed);
        bool OtherCarLCHelper(vector<vector<double>> lane_front, vector<vector<double>> lane_back, double car_speed);
        
        // Check speed change is required
        bool CheckChangeSpeed(double car_s, int path_size);
        
        // Get current lane of the ego vehicle
        char GetCurrentLane(double car_d);

        // Check if left lane change is possible
        bool PrepareLCL(double car_speed);

        // Check if right lane change is possible
        bool PrepareLCR(double car_speed);

        // Situations when lane changes from left_lane to right_lane or vice-versa is required
        bool IsLaneChangeSafe(vector<vector<double>> lane_front, vector<vector<double>> lane_back, double car_speed);

}; 

#endif /* PATHPLANNER_H_ */
