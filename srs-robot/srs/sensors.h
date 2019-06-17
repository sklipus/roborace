#define DELTA_OF_SENSORS 0
#define PREV_VALUE_OF_SENSORS 1
#define DELTA_OF_SENSORS 0
#define PREV_VALUE_OF_SENSORS 1

class Sensors {
	public:
		static uint8_t data[NUMBER_OF_SENSORS];
		static int16_t deltaOfData[NUMBER_OF_SENSORS][2];
		static uint8_t get_voltage(uint8_t index);
		static void updateSensors();
		static uint8_t sensorsHistory[NUMBER_OF_SENSORS][MEDIAN_SIZE];
		static uint8_t calcMedian(uint8_t index, uint8_t newValue);
		static void updateDeltaOfSensors();
		static uint8_t maxDistance;
		static uint8_t minDistance;
		static void checkIfRobotIsBlocked();
};
