#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <phidget22.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "sonar_phidget/sonarData.h"
#include <signal.h>



bool quietMode = false;
uint32_t dataInterval = 250;
sonar_phidget::sonarData data;

PhidgetDistanceSensorHandle ch0;
PhidgetDistanceSensorHandle ch1;
PhidgetDistanceSensorHandle ch2;
PhidgetDistanceSensorHandle ch3;
PhidgetDistanceSensorHandle ch4;
PhidgetDistanceSensorHandle ch5;
PhidgetDistanceSensorHandle ch6;
PhidgetDistanceSensorHandle ch7;

void mySigintHandler(int sig);

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  ROS_INFO("Shutting down...\n");
  PhidgetDistanceSensor_delete(&ch0); // will NULL the pointer
  PhidgetDistanceSensor_delete(&ch1);
  PhidgetDistanceSensor_delete(&ch2);
  PhidgetDistanceSensor_delete(&ch3);
  PhidgetDistanceSensor_delete(&ch4);
  PhidgetDistanceSensor_delete(&ch5);
  ROS_INFO("Successfully shut down! \n");
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

static void CCONV onAttachHandler_ch0(PhidgetHandle phid, void *ctx) {
	ROS_INFO("Sonar attached to hub port 0\n");
	PhidgetDistanceSensor_setSonarQuietMode((PhidgetDistanceSensorHandle)phid, quietMode);
	PhidgetDistanceSensor_setDataInterval((PhidgetDistanceSensorHandle) phid, dataInterval);
}

static void CCONV onAttachHandler_ch1(PhidgetHandle phid, void *ctx) {
	ROS_INFO("Sonar attached to hub port 1\n");
	PhidgetDistanceSensor_setSonarQuietMode((PhidgetDistanceSensorHandle)phid, quietMode);
	PhidgetDistanceSensor_setDataInterval((PhidgetDistanceSensorHandle) phid, dataInterval);
}

static void CCONV onAttachHandler_ch2(PhidgetHandle phid, void *ctx) {
	ROS_INFO("Sonar attached to hub port 2\n");
	PhidgetDistanceSensor_setSonarQuietMode((PhidgetDistanceSensorHandle)phid, quietMode);
	PhidgetDistanceSensor_setDataInterval((PhidgetDistanceSensorHandle) phid, dataInterval);
}

static void CCONV onAttachHandler_ch3(PhidgetHandle phid, void *ctx) {
	ROS_INFO("Sonar attached to hub port 3\n");
	PhidgetDistanceSensor_setSonarQuietMode((PhidgetDistanceSensorHandle)phid, quietMode);
	PhidgetDistanceSensor_setDataInterval((PhidgetDistanceSensorHandle) phid, dataInterval);
}

static void CCONV onAttachHandler_ch4(PhidgetHandle phid, void *ctx) {
	ROS_INFO("Sonar attached to hub port 4\n");
	PhidgetDistanceSensor_setSonarQuietMode((PhidgetDistanceSensorHandle)phid, quietMode);
	PhidgetDistanceSensor_setDataInterval((PhidgetDistanceSensorHandle) phid, dataInterval);
}

static void CCONV onAttachHandler_ch5(PhidgetHandle phid, void *ctx) {
	ROS_INFO("Sonar attached to hub port 5\n");
	PhidgetDistanceSensor_setSonarQuietMode((PhidgetDistanceSensorHandle)phid, quietMode);
	PhidgetDistanceSensor_setDataInterval((PhidgetDistanceSensorHandle) phid, dataInterval);
}

static void CCONV onAttachHandler_ch6(PhidgetHandle phid, void *ctx) {
	ROS_INFO("Sonar attached to hub port 6\n");
	PhidgetDistanceSensor_setSonarQuietMode((PhidgetDistanceSensorHandle)phid, quietMode);
	PhidgetDistanceSensor_setDataInterval((PhidgetDistanceSensorHandle) phid, dataInterval);
}

static void CCONV onAttachHandler_ch7(PhidgetHandle phid, void *ctx) {
	ROS_INFO("Sonar attached to hub port 7\n");
	PhidgetDistanceSensor_setSonarQuietMode((PhidgetDistanceSensorHandle)phid, quietMode);
	PhidgetDistanceSensor_setDataInterval((PhidgetDistanceSensorHandle) phid, dataInterval);
}

static void CCONV
onDistanceChangeHandler_ch0(PhidgetDistanceSensorHandle ch, void *ctx, uint32_t distance) {
	data.sonar0 = distance;
	//ROS_INFO("Sonar 0 distance Changed: %d\n", data.sonar0);
}

static void CCONV
onDistanceChangeHandler_ch1(PhidgetDistanceSensorHandle ch, void *ctx, uint32_t distance) {
	data.sonar1 = distance;
	//ROS_INFO("Sonar 1 distance Changed: %d\n", data.sonar1);
}

static void CCONV
onDistanceChangeHandler_ch2(PhidgetDistanceSensorHandle ch, void *ctx, uint32_t distance) {
	data.sonar2 = distance;
	//ROS_INFO("Sonar 2 distance Changed: %d\n", data.sonar2);
}

static void CCONV
onDistanceChangeHandler_ch3(PhidgetDistanceSensorHandle ch, void *ctx, uint32_t distance) {
	data.sonar3 = distance;
	//printf("Sonar 3 distance Changed: %d\n", data.sonar3);
}

static void CCONV
onDistanceChangeHandler_ch4(PhidgetDistanceSensorHandle ch, void *ctx, uint32_t distance) {
	data.sonar4 = distance;
	//ROS_INFO("Sonar 4 distance Changed: %d\n", data.sonar4);
}

static void CCONV
onDistanceChangeHandler_ch5(PhidgetDistanceSensorHandle ch, void *ctx, uint32_t distance) {
	data.sonar5 = distance;
	//ROS_INFO("Sonar 5 distance Changed: %d\n", data.sonar5);
}

static void CCONV
onDistanceChangeHandler_ch6(PhidgetDistanceSensorHandle ch, void *ctx, uint32_t distance) {
	data.sonar6 = distance;
	//ROS_INFO("Sonar 4 distance Changed: %d\n", data.sonar4);
}

static void CCONV
onDistanceChangeHandler_ch7(PhidgetDistanceSensorHandle ch, void *ctx, uint32_t distance) {
	data.sonar7 = distance;
	//ROS_INFO("Sonar 5 distance Changed: %d\n", data.sonar5);
}

static void CCONV
onSonarReflectionsUpdateHandler_ch0(PhidgetDistanceSensorHandle ch, void *ctx, const uint32_t distances[8], const uint32_t amplitudes[8], uint32_t count){
	for(int i = 0; i < count; i++){
		data.distances0[i] = (uint32_t)distances[i];
		data.amplitudes0[i] = (uint32_t)amplitudes[i];
	}
}

static void CCONV
onSonarReflectionsUpdateHandler_ch1(PhidgetDistanceSensorHandle ch, void *ctx, const uint32_t distances[8], const uint32_t amplitudes[8], uint32_t count){
	for(int i = 0; i < count; i++){
		data.distances1[i] = (uint32_t)distances[i];
		data.amplitudes1[i] = (uint32_t)amplitudes[i];
	}
}

static void CCONV
onSonarReflectionsUpdateHandler_ch2(PhidgetDistanceSensorHandle ch, void *ctx, const uint32_t distances[8], const uint32_t amplitudes[8], uint32_t count){
	for(int i = 0; i < count; i++){
		data.distances2[i] = (uint32_t)distances[i];
		data.amplitudes2[i] = (uint32_t)amplitudes[i];
	}
}

static void CCONV
onSonarReflectionsUpdateHandler_ch3(PhidgetDistanceSensorHandle ch, void *ctx, const uint32_t distances[8], const uint32_t amplitudes[8], uint32_t count){
	for(int i = 0; i < count; i++){
		data.distances3[i] = (uint32_t)distances[i];
		data.amplitudes3[i] = (uint32_t)amplitudes[i];
	}
}

static void CCONV
onSonarReflectionsUpdateHandler_ch4(PhidgetDistanceSensorHandle ch, void *ctx, const uint32_t distances[8], const uint32_t amplitudes[8], uint32_t count){
	for(int i = 0; i < count; i++){
		data.distances4[i] = (uint32_t)distances[i];
		data.amplitudes4[i] = (uint32_t)amplitudes[i];
	}
}

static void CCONV
onSonarReflectionsUpdateHandler_ch5(PhidgetDistanceSensorHandle ch, void *ctx, const uint32_t distances[8], const uint32_t amplitudes[8], uint32_t count){
	for(int i = 0; i < count; i++){
		data.distances5[i] = (uint32_t)distances[i];
		data.amplitudes5[i] = (uint32_t)amplitudes[i];
	}
}

static void CCONV
onSonarReflectionsUpdateHandler_ch6(PhidgetDistanceSensorHandle ch, void *ctx, const uint32_t distances[8], const uint32_t amplitudes[8], uint32_t count){
	for(int i = 0; i < count; i++){
		data.distances6[i] = (uint32_t)distances[i];
		data.amplitudes6[i] = (uint32_t)amplitudes[i];
	}
}

static void CCONV
onSonarReflectionsUpdateHandler_ch7(PhidgetDistanceSensorHandle ch, void *ctx, const uint32_t distances[8], const uint32_t amplitudes[8], uint32_t count){
	for(int i = 0; i < count; i++){
		data.distances7[i] = (uint32_t)distances[i];
		data.amplitudes7[i] = (uint32_t)amplitudes[i];
	}
}

int
main(int argc, char **argv) {
  	ros::init(argc, argv, "sonar_phidget", ros::init_options::NoSigintHandler);
  	ros::NodeHandle n;
	signal(SIGINT, mySigintHandler);

	ros::Publisher pub = n.advertise<sonar_phidget::sonarData>("sonar_data", 100);
  	ros::Rate loop_rate(4);

	PhidgetDistanceSensor_create(&ch0);
	PhidgetDistanceSensor_create(&ch1);
	PhidgetDistanceSensor_create(&ch2);
	PhidgetDistanceSensor_create(&ch3);
	PhidgetDistanceSensor_create(&ch4);
	PhidgetDistanceSensor_create(&ch5);
	PhidgetDistanceSensor_create(&ch6);
	PhidgetDistanceSensor_create(&ch7);

	Phidget_setDeviceSerialNumber((PhidgetHandle) ch0, 497026);
	Phidget_setDeviceSerialNumber((PhidgetHandle) ch1, 497026);
	Phidget_setDeviceSerialNumber((PhidgetHandle) ch2, 497026);
	Phidget_setDeviceSerialNumber((PhidgetHandle) ch3, 497026);
	Phidget_setDeviceSerialNumber((PhidgetHandle) ch4, 497026);
	Phidget_setDeviceSerialNumber((PhidgetHandle) ch5, 497026);
	Phidget_setDeviceSerialNumber((PhidgetHandle) ch6, 497383);
	Phidget_setDeviceSerialNumber((PhidgetHandle) ch7, 497383);

	Phidget_setHubPort((PhidgetHandle)ch0, 0);
	Phidget_setHubPort((PhidgetHandle)ch1, 1);
	Phidget_setHubPort((PhidgetHandle)ch2, 2);
	Phidget_setHubPort((PhidgetHandle)ch3, 3);
	Phidget_setHubPort((PhidgetHandle)ch4, 4);
	Phidget_setHubPort((PhidgetHandle)ch5, 5);
	Phidget_setHubPort((PhidgetHandle)ch6, 0);
	Phidget_setHubPort((PhidgetHandle)ch7, 1);


	Phidget_setChannel((PhidgetHandle)ch0, 0);
	Phidget_setChannel((PhidgetHandle)ch1, 0);
	Phidget_setChannel((PhidgetHandle)ch2, 0);
	Phidget_setChannel((PhidgetHandle)ch3, 0);
	Phidget_setChannel((PhidgetHandle)ch4, 0);
	Phidget_setChannel((PhidgetHandle)ch5, 0);
	Phidget_setChannel((PhidgetHandle)ch6, 0);
	Phidget_setChannel((PhidgetHandle)ch7, 0);

	Phidget_setOnAttachHandler((PhidgetHandle)ch0, onAttachHandler_ch0, NULL);
	Phidget_setOnAttachHandler((PhidgetHandle)ch1, onAttachHandler_ch1, NULL);
	Phidget_setOnAttachHandler((PhidgetHandle)ch2, onAttachHandler_ch2, NULL);
	Phidget_setOnAttachHandler((PhidgetHandle)ch3, onAttachHandler_ch3, NULL);
	Phidget_setOnAttachHandler((PhidgetHandle)ch4, onAttachHandler_ch4, NULL);
	Phidget_setOnAttachHandler((PhidgetHandle)ch5, onAttachHandler_ch5, NULL);
	Phidget_setOnAttachHandler((PhidgetHandle)ch6, onAttachHandler_ch6, NULL);
	Phidget_setOnAttachHandler((PhidgetHandle)ch7, onAttachHandler_ch7, NULL);
	
	PhidgetDistanceSensor_setOnDistanceChangeHandler(ch0, onDistanceChangeHandler_ch0, NULL);
	PhidgetDistanceSensor_setOnDistanceChangeHandler(ch1, onDistanceChangeHandler_ch1, NULL);
	PhidgetDistanceSensor_setOnDistanceChangeHandler(ch2, onDistanceChangeHandler_ch2, NULL);
	PhidgetDistanceSensor_setOnDistanceChangeHandler(ch3, onDistanceChangeHandler_ch3, NULL);
	PhidgetDistanceSensor_setOnDistanceChangeHandler(ch4, onDistanceChangeHandler_ch4, NULL);
	PhidgetDistanceSensor_setOnDistanceChangeHandler(ch5, onDistanceChangeHandler_ch5, NULL);
	PhidgetDistanceSensor_setOnDistanceChangeHandler(ch6, onDistanceChangeHandler_ch6, NULL);
	PhidgetDistanceSensor_setOnDistanceChangeHandler(ch7, onDistanceChangeHandler_ch7, NULL);

	PhidgetDistanceSensor_setOnSonarReflectionsUpdateHandler(ch0, onSonarReflectionsUpdateHandler_ch0, NULL);
	PhidgetDistanceSensor_setOnSonarReflectionsUpdateHandler(ch1, onSonarReflectionsUpdateHandler_ch1, NULL);
	PhidgetDistanceSensor_setOnSonarReflectionsUpdateHandler(ch2, onSonarReflectionsUpdateHandler_ch2, NULL);
	PhidgetDistanceSensor_setOnSonarReflectionsUpdateHandler(ch3, onSonarReflectionsUpdateHandler_ch3, NULL);
	PhidgetDistanceSensor_setOnSonarReflectionsUpdateHandler(ch4, onSonarReflectionsUpdateHandler_ch4, NULL);
	PhidgetDistanceSensor_setOnSonarReflectionsUpdateHandler(ch5, onSonarReflectionsUpdateHandler_ch5, NULL);
	PhidgetDistanceSensor_setOnSonarReflectionsUpdateHandler(ch6, onSonarReflectionsUpdateHandler_ch6, NULL);
	PhidgetDistanceSensor_setOnSonarReflectionsUpdateHandler(ch7, onSonarReflectionsUpdateHandler_ch7, NULL);

	Phidget_open((PhidgetHandle)ch0);
	Phidget_open((PhidgetHandle)ch1);
	Phidget_open((PhidgetHandle)ch2);
	Phidget_open((PhidgetHandle)ch3);
	Phidget_open((PhidgetHandle)ch4);
	Phidget_open((PhidgetHandle)ch5);
	Phidget_open((PhidgetHandle)ch6);
	Phidget_open((PhidgetHandle)ch7);


  	while (ros::ok()){
  		pub.publish(data);

    	ROS_INFO("Published sonar0 %d, sonar1 %d, sonar2 %d, sonar3 %d, sonar4 %d, sonar5 %d, sonar6 %d, sonar7 %d\n", data.sonar0, data.sonar1, data.sonar2, data.sonar3, data.sonar4, data.sonar5, data.sonar6, data.sonar7);

    	ros::spinOnce();

    	loop_rate.sleep();
  	}
	return 0;	
}
