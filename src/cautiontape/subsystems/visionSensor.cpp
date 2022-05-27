#include "VisionSensor.hpp"
#include <iterator>

namespace lamaLib {
VisionSensor::VisionSensor(int vPort) : pros::Vision(vPort) {
	//Configures colour signatures
	//See: https://pros.cs.purdue.edu/v5/tutorials/topical/vision.html#setting-signatures
}

void VisionSensor::setSignatures(pros::vision_signature_s_t inputSigs[]){
	for(int i = 1; i<=7; i++){
		set_signature(i, &inputSigs[i]);
	}
}

int VisionSensor::getMiddle(int signature){
	//Gets the middle coordinate of the largest object with the specified colour signature
	//Returns coordinate out of 312(?) total camera resolution, e.g. middle coordinate would be 156
	pros::vision_object_s_t inp = get_by_sig(0, signature);
	return inp.x_middle_coord;
}

int VisionSensor::getWidth(int signature){
	//Returns the width of the largest object with the specified colour signature
	pros::vision_object_s_t inp = get_by_sig(0, signature);
	return inp.width;
}

int VisionSensor::getCount(){
	int inp = get_object_count();
	return inp;
}
}