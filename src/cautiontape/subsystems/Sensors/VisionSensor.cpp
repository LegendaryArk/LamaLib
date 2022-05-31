#include "VisionSensor.hpp"
#include <iterator>

namespace lamaLib {
Vision::Vision(int vPort) : pros::Vision(vPort) {
	//Configures colour signatures
	//See: https://pros.cs.purdue.edu/v5/tutorials/topical/vision.html#setting-signatures
}

void Vision::setSignatures(pros::vision_signature_s_t inputSigs[]){
	for(int i = 1; i<=7; i++){
		set_signature(i, &inputSigs[i]);
	}
}

int Vision::getMiddle(int signature){
	//Gets the middle coordinate of the largest object with the specified colour signature
	//Returns coordinate out of 312(?) total camera resolution, e.g. middle coordinate would be 156
	pros::vision_object_s_t inp = get_by_sig(0, signature);
	return inp.x_middle_coord;
}

int Vision::getWidth(int signature){
	//Returns the width of the largest object with the specified colour signature
	pros::vision_object_s_t inp = get_by_sig(0, signature);
	return inp.width;
}

int Vision::getCount(){
	int inp = get_object_count();
	return inp;
}
}