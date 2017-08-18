#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "PathPlanner.h"

using namespace std;

// Init the PathPlanner
PathPlanner::PathPlanner() {}



/************************* PREDICTION ***********************/

// Maintain a list of all the cars surrounding the one we drive, updated for each timesteps
void PathPlanner::MaintainListSurroundingCars(vector<vector<double>> sensor_fusion, double car_s){

	for (int i = 0; i < sensor_fusion.size(); i++) {

		double sdist = fabs(car_s - sensor_fusion[i][5]);
		double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
		double v  = sqrt(vx*vx + vy*vy)/2.24;
		sensor_fusion[i].push_back(v);		// lane[i][7]
		sensor_fusion[i].push_back(sdist);	// lane[i][8]

		double EPSILON = 0.8;

		// Maintain a list within 50m of s distance from the ego car
		if (sdist < 50.) {
			// Car on the left lane
			if (sensor_fusion[i][6] < 4.) {
				if (((sensor_fusion[i][6] - 2.) > EPSILON) && (current_lane == 'M')) {
					sensor_fusion[i].push_back(1);		// lane[i][9]
				} else {
					sensor_fusion[i].push_back(0);
				}

				if (sensor_fusion[i][5] > car_s) {
					left_lane_front.push_back(sensor_fusion[i]);
				} else {
					left_lane_back.push_back(sensor_fusion[i]);
				}
			}
			// Cars on right lane
			else if (sensor_fusion[i][6] > 8.) {
				if (((10. - sensor_fusion[i][6]) > EPSILON) && (current_lane == 'M')) {
					sensor_fusion[i].push_back(1);
				} else {
					sensor_fusion[i].push_back(0);
				}

				if (sensor_fusion[i][5] > car_s) {
					right_lane_front.push_back(sensor_fusion[i]);
				} else {
					right_lane_back.push_back(sensor_fusion[i]);
				}
			}
			// Cars on middle lane
			else {
				if (((6. - sensor_fusion[i][6]) > EPSILON) && (current_lane == 'L')) {
					sensor_fusion[i].push_back(1);
				} else {
					sensor_fusion[i].push_back(0);
				}
				if (((sensor_fusion[i][6] - 6.0) > EPSILON) && (current_lane == 'R')) {
					sensor_fusion[i].push_back(1);
				} else {
					sensor_fusion[i].push_back(0);
				}

				if (sensor_fusion[i][5] > car_s) {
					mid_lane_front.push_back(sensor_fusion[i]);
				} else {
					mid_lane_back.push_back(sensor_fusion[i]);
				}
			}
		}
	}
    cout << "left_front: " << left_lane_front.size() << "\tback: " << left_lane_back.size() << endl;
    cout << "mid_front: " << mid_lane_front.size() << "\tback: " << mid_lane_back.size() << endl;
    cout << "right_front: " << right_lane_front.size() << "\tback: " << right_lane_back.size() << endl;
}


// Helper function for OtherCarLC()
bool PathPlanner::OtherCarLCHelper(vector<vector<double>> lane_front, vector<vector<double>> lane_back, double car_speed) {

	bool OtherCarLC = false;

	if (lane_front.size() != 0) {
		for (int i=0; i<lane_front.size(); i++) {
			if (lane_front[i][9]) {
				OtherCarLC = true;
			}
		}
	}
	if (lane_back.size() != 0) {
		for (int i=0; i<lane_back.size(); i++) {
			if ((lane_back[i][9]) && (lane_back[i][8] < 20.) && (lane_back[i][7] > car_speed)) {
				OtherCarLC = true;
			}
		}
	}
	cout << "Other Car Lane Change: " << OtherCarLC << endl;
	return OtherCarLC;

}


// To check if surrounding cars are moving into the lane that ego car is currently on
// If yes, stay cautious and slow down
bool PathPlanner::CheckOtherCarLC(double car_speed) {

	bool OtherCarLC = false;

	if ((current_lane == 'L') || (current_lane == 'R')) {

		OtherCarLC = OtherCarLCHelper(mid_lane_front, mid_lane_back, car_speed);

	} else {

		OtherCarLC = OtherCarLCHelper(left_lane_front, left_lane_back, car_speed);

		OtherCarLC = OtherCarLCHelper(right_lane_front, right_lane_back, car_speed);

	}
	return OtherCarLC;
}


// Change ego car's speed to the average speed of cars in the lane
double PathPlanner::AverageLaneSpeed() {

	vector<vector<double>> lane_front;

	if (current_lane == 'L') {
		lane_front = left_lane_front;
	} else if (current_lane == 'R') {
		lane_front = right_lane_front;
	} else {
		lane_front = mid_lane_front;
	}

	double v = 0.;
	for (int i=0;i<lane_front.size();i++) {
		v += lane_front[i][7];
	}
	double avg_v = v / lane_front.size();
	return avg_v;
}


// Check speed change is required
bool PathPlanner::CheckChangeSpeed(double car_s, int path_size) {

	bool too_close = false;
	vector<vector<double>> lane_front;

	if (current_lane == 'L') {
		lane_front = left_lane_front;
	} else if (current_lane == 'R') {
		lane_front = right_lane_front;
	} else {
		lane_front = mid_lane_front;
	}

	// find ref_v to use
	for (int i=0; i < lane_front.size(); i++) {
		double vx = lane_front[i][3];
		double vy = lane_front[i][4];
		double check_speed = sqrt(vx*vx + vy*vy);
		double check_car_s = lane_front[i][5];

		// Predict car's position after 1 second
		// Check_speed converted to mps
		check_car_s += ((double)path_size*0.02*check_speed/2.24);
	
		// check s values > ego and s gap
		if ((check_car_s>car_s) && ((check_car_s-car_s)<30)) {

			//cout << "car_s: " << car_s << "\tocar_s: " << lane_front[i][5] << "\tcheck_car_s: " << check_car_s << endl;
			too_close = true;
		}
	}
	return too_close;
}


/******************** BEHAVIOURAL PLANNING ***********************/

// Get current lane of the ego vehicle
char PathPlanner::GetCurrentLane(double car_d) {

	double EPSILON = 0.4;

	cout << "CHG_LEFT: " << CHG_LEFT << "\tCHG_RIGHT: " << CHG_RIGHT << endl;

	if (fabs(car_d - 2.) < EPSILON) {
		current_lane = 'L';
		if (lane == 0) {
			CHG_LEFT = false;
			CHG_RIGHT = false;
		}
	} else if (fabs(car_d - 10.) < EPSILON) {
		current_lane = 'R';
		if (lane == 2) {
			CHG_LEFT = false;
			CHG_RIGHT = false;
		}
	} else if (fabs(car_d - 6.) < EPSILON) {
		current_lane = 'M';
		if (lane == 1) {
			CHG_LEFT = false;
			CHG_RIGHT = false;
		}
	}

	return current_lane;
}


// Situations when lane changes from left_lane to right_lane or vice-versa is required
bool PathPlanner::IsLaneChangeSafe(vector<vector<double>> lane_front, vector<vector<double>> lane_back, double car_speed) {

	bool LCSafe = false;

	double min_front_sdist = 1000.;
	double min_front_v;

	for (int i=0;i<lane_front.size();i++) {
		double v = lane_front[i][7];
		double front_sdist = lane_front[i][8];
	
		if (front_sdist < min_front_sdist) {
			min_front_sdist = front_sdist;
			min_front_v = v;
		}
	}

	double min_back_sdist = 1000.;
	double min_back_v;

	for (int i=0;i<lane_back.size();i++) {
		double v = lane_back[i][7];
		double back_sdist = lane_back[i][8];

		if (back_sdist < min_back_sdist) {
			min_back_sdist = back_sdist;
			min_back_v = v;
		}
	}
	
	//cout << "(car_speed < min_front_v): " << (car_speed < min_front_v) << "\tcar_speed > min_back_v ? " << (car_speed > min_back_v) << endl;
	cout << "Min front sdist: " << min_front_sdist << "\tMin back sdist: " << min_back_sdist << endl;

	// Car at the front: 0; 	Car at the back: 0
	if ((lane_front.size()==0) && (lane_back.size()==0)) {
		LCSafe = true;
	}
	// Car at the front: 0; 	Car at the back: >=1
	else if ((lane_front.size()==0) && (lane_back.size()!=0)) {
		//if ((car_speed > min_back_v) && (min_back_sdist > 20.) && (min_back_sdist > 20.)) {
		if (min_back_sdist > 25.) {
			LCSafe = true;
		}
	}

	// Car at the front: >=1; 	Car at the back: 0
	else if ((lane_front.size()!=0) && (lane_back.size()==0)) {
		//if ((car_speed < min_front_v) && (min_front_sdist > 10.) && (min_front_sdist > 10.)) {
		if (min_front_sdist > 15.) {
			LCSafe = true;
		}
	}

	// Car at the front: >=1; 	Car at the back: >=1
	else if ((lane_front.size()!=0) && (lane_back.size()!=0)) {
		//if ((car_speed < min_front_v) && (min_front_sdist > 10.) && (min_front_sdist > 10.) && (car_speed > min_back_v) && (min_back_dist > 20.) && (min_back_sdist > 20.)) {
		if ((min_front_sdist > 15.) && (min_back_sdist > 25.)) {
			LCSafe = true;
		}
	}

	return LCSafe;
}


// Check if left lane change is possible
bool PathPlanner::PrepareLCL(double car_speed) {

	CHG_LEFT = false;

	// Changing from mid-lane TO left-lane
	if ((current_lane == 'M') && (!CHG_RIGHT)) {
		
		// Check, if car at the front is changing lanes
		// Car_speed is set to a large number to disable checking for cars at the back
		bool OtherCarLC = OtherCarLCHelper(mid_lane_front, mid_lane_back, 100.);
		
		bool LCSafe = IsLaneChangeSafe(left_lane_front, left_lane_back, car_speed);
		
		cout << "Checking M TO L" << endl;
		if ((LCSafe) && (!OtherCarLC)) {
			CHG_LEFT = true;
			lane = 0;
    		cout << "PrepareLCL::LaneChange: " << lane << endl;
		}
	}
	// Changing from right-lane TO mid-lane
	else if ((current_lane == 'R') && (!CHG_RIGHT)) {

		// Check, if car at the front is changing lanes
		// Car_speed is set to a large number to disable checking for cars at the back
		bool OtherCarLC = OtherCarLCHelper(right_lane_front, right_lane_back, 100.);
		
        bool LCSafe = IsLaneChangeSafe(mid_lane_front, mid_lane_back, car_speed);
		
		cout << "Checking R TO M" << endl;
		if ((LCSafe) && (!OtherCarLC)) {
			CHG_LEFT = true;
			lane = 1;
    		cout << "PrepareLCL::LaneChange: " << lane << endl;
		}
	}
	return CHG_LEFT;
}


// Check if right lane change is possible
bool PathPlanner::PrepareLCR(double car_speed) {

	CHG_RIGHT = false;

	// Changing from mid-lane TO right-lane
	if ((current_lane == 'M') && (!CHG_LEFT)) {
		
		// Check, if car at the front is changing lanes
		// Car_speed is set to a large number to disable checking for cars at the back
		bool OtherCarLC = OtherCarLCHelper(mid_lane_front, mid_lane_back, 100.);
		
		bool LCSafe = IsLaneChangeSafe(right_lane_front, right_lane_back, car_speed);
		
		cout << "Checking M TO R" << endl;
		if ((LCSafe) && (!OtherCarLC)) {
			CHG_RIGHT = true;
			lane = 2;
    		cout << "PrepareLCR::LaneChange: " << lane << endl;
		}
	}
	// Changing from left-lane TO mid-lane
	else if ((current_lane == 'L') && (!CHG_LEFT)) {
		
		// Check, if car at the front is changing lanes
		// Car_speed is set to a large number to disable checking for cars at the back
		bool OtherCarLC = OtherCarLCHelper(left_lane_front, left_lane_back, 100.);
		
        bool LCSafe = IsLaneChangeSafe(mid_lane_front, mid_lane_back, car_speed);

		cout << "Checking L TO M" << endl;
		if ((LCSafe) && (!OtherCarLC)) {
			CHG_RIGHT = true;
			lane = 1;
    		cout << "PrepareLCR::LaneChange: " << lane << endl;
		}
	}
	return CHG_RIGHT;
}


