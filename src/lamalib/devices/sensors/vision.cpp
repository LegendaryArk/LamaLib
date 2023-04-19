#include "vision.hpp"

using namespace std;
using namespace lamaLib;

Vision::Vision(int port) : pros::Vision(port) {
  // Configures colour signatures
  // See:
  // https://pros.cs.purdue.edu/v5/tutorials/topical/vision.html#setting-signatures
}
Vision::Vision(pros::Vision vision) : pros::Vision(vision) {}

void Vision::setSignature(string key, pros::vision_signature_s_t signature) {
	if (signatures.size() > 7) {
		cerr << "There are too many signatures assigned to the vision sensor\n";
		return;
	}

	set_signature(signatures.size() + 1, &signature);
	signatures.insert_or_assign(key, signature);
}

int Vision::getMiddle(string key, int numLargest) {
  // Gets the middle coordinate of the largest object with the specified colour
  // signature Returns coordinate out of 316 total camera resolution, e.g.
  // middle coordinate would be 156
  pros::vision_object_s_t blob = get_by_sig(numLargest - 1, signatures.at(key).id);
  return blob.x_middle_coord;
}
int Vision::getWidth(string key, int numLargest) {
  // Returns the width of the largest object with the specified colour signature
  pros::vision_object_s_t blob = get_by_sig(numLargest - 1, signatures.at(key).id);
  return blob.width;
}
int Vision::getHeight(string key, int numLargest) {
	pros::vision_object_s_t blob = get_by_sig(numLargest - 1, signatures.at(key).id);
	return blob.height;
}
int Vision::getX(string key, int numLargest) {
	pros::vision_object_s_t blob = get_by_sig(numLargest - 1, signatures.at(key).id);
	return blob.left_coord;
}
int Vision::getY(string key, int numLargest) {
	pros::vision_object_s_t blob = get_by_sig(numLargest - 1, signatures.at(key).id);
	return blob.top_coord;
}

int Vision::getCount() {
  return get_object_count();
}

void Vision::setLedHex(int hex) {
	set_led(hex);
}
void Vision::setLedRGB(int red, int green, int blue) {
	int hex = RGB2COLOR(red, green, blue);
	setLedHex(hex);
}

void Vision::setOrigin(pros::vision_zero_e_t origin) {
	set_zero_point(origin);
}