#include "routes.hpp"

using namespace lamaLib;

void startRightMidGoal() {
	chassis.setPose({7.5, 1.2, -30});

	chassis.moveToPose({6.5, 5.5, -30}, 0, {}, {{1.45, 2.7}}, {0}, {0.0067, 0.002, 0.0025, 0});
	frontClaw.toggle();
	chassis.moveToPose({7.5, 5.5, -120}, 1, {}, {{1.45, 2.7}}, {0}, {0.0067, 0.002, 0.0025, 0}, true);
	backClaw.moveAbsolute(-50, 100);
	chassis.moveToPose({7.5, 2, -180}, 1, {}, {{1.45, 2.7}}, {0}, {0.0067, 0.002, 0.0025, 0});
	backClaw.moveAbsolute(100, 100);
	frontClaw.toggle();
	chassis.moveToPose({10.5, 2.5, -135}, 1, {}, {{1.45, 2.7}}, {0}, {0.0067, 0.002, 0.0025, 0}, true);
	backClaw.moveAbsolute(-50, 100);
	conveyor.moveVelocity(600);
	pros::delay(2000);
	chassis.turnAbsolute(-90, 1, {0.0067, 0.002, 0.0025, 0});
	// Vision search
	// Vision follow
}
void startRightLeftGoal() {
	chassis.setPose({7, 1.2, 0});

	chassis.moveToPose({7, 5.5, 0}, 0, {}, {{1.45, 2.7}}, {0}, {0.0067, 0.002, 0.0025, 0});
	frontClaw.toggle();
	chassis.moveToPose({8, 3, -90}, 0, {}, {{1.45, 2.7}}, {0}, {0.0067, 0.002, 0.0025, 0}, true);
	chassis.moveToPose({8.3, 3, -90}, 0, {}, {{1.45, 2.7}}, {0}, {0.0067, 0.002, 0.0025, 0}, true);
	backClaw.moveAbsolute(-50, 100);
	conveyor.moveVelocity(600);
	pros::delay(2000);
	chassis.turnAbsolute(-180, 1, {0.0067, 0.002, 0.0025, 0});
	frontClaw.toggle();
	chassis.turnAbsolute(-90, 1, {0.0067, 0.002, 0.0025, 0});
	// Vision search
	// Vision follow
}

void startLeftRightGoal() {
	chassis.setPose({2.8, 1.2, 20});

	chassis.moveToPose({3, 5.2, 20}, 0, {}, {{1.45, 2.7}}, {0}, {0.0067, 0.002, 0.0025, 0});
	frontClaw.toggle();
	chassis.moveToPose({3.5, 2.2, -20}, 0, {}, {{1.45, 2.7}}, {0}, {0.0067, 0.002, 0.0025, 0}, true);
	backClaw.moveAbsolute(-50, 100);
	conveyor.moveVelocity(600);
	pros::delay(2000);
	chassis.turnAbsolute(-90, 1, {0.0067, 0.002, 0.0025, 0});
	frontClaw.toggle();
	chassis.turnAbsolute(0, 1, {0.0067, 0.002, 0.0025, 0});
	// Vision search
	// Vision follow
}

void startRightFullAWP() {

}
void startLeftFullAWP() {

}

void skills() {
	
}