#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#define NUM_WAYPOINTS 7
#define WAYPOINT_DRIFT_RATE_MM_S (- 5.0f / 24.0f)

class Waypoints {
	private:
		float waypoints_x[NUM_WAYPOINTS] = {0, 69.83, 266.69, 432.97, 411.76, 355.68, 162.42};
		float waypoints_y[NUM_WAYPOINTS] = {0, 269.21, 215.27, 264.17, 102.55, -18.66, 85.01};
		unsigned long waypoint_drift_ts;

	public:
		float origin_y;
		int current;
		Waypoints() {}

	void initialise() {
		origin_y = 0;
		current = 0;
		waypoint_drift_ts = 0;
	}

	float current_x() {
		return waypoints_x[current];
	}

	float current_y() {
		return waypoints_y[current];
	}

	void doDrift() {
		// Drift waypoints' and origin's y coordinates by -5/24 mm every second
		if (millis() - waypoint_drift_ts < 1000) {return;}
		waypoint_drift_ts = millis();
		for (int i = 0; i < NUM_WAYPOINTS; i++) {
			waypoints_y[i] -= WAYPOINT_DRIFT_RATE_MM_S;
		}
		origin_y -= WAYPOINT_DRIFT_RATE_MM_S;
	}

	void increment() {
		if (current++ == 6) {
			current = 0;
		}
	}
};

#endif