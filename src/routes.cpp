#include "routes.hpp"

using namespace lamaLib;

void startRightMidGoal() {
	chassis.setPose({7.5, 1.2, -30});

	chassis.moveToPose({6.5, 5.5, -30}, 0, {}, {{1.45, 2.7}}, {0}, {0.0067, 0.002, 0.0025, 0});
	frontClaw.toggle();
	chassis.moveToPose({7.5, 5.5, -120}, 1, {}, {{1.45, 2.2}}, {0}, {0.0067, 0.002, 0.0025, 0}, true);
	backClaw.moveAbsolute(-550, 100);
	chassis.moveToPose({7.5, 2, -180}, 1, {}, {{1.45, 2.7}}, {0}, {0.0067, 0.002, 0.0025, 0});
	backClaw.moveAbsolute(0, 100);
	frontClaw.toggle();
	chassis.moveToPose({10.5, 2.5, -135}, 1, {}, {{1.45, 2.7}}, {0}, {0.0067, 0.002, 0.0025, 0}, true);
	backClaw.moveAbsolute(-450, 100);
	conveyor.moveVelocity(420);
	chassis.moveToPose({10, 6, 0}, 0, {}, {{0.5, 0.5}}, {0}, {0.0067, 0.002, 0.0025, 0});
	conveyor.moveVelocity(0);
	chassis.moveToPose({8, 4, -90}, 0, {}, {{1.45, 2.7}}, {0}, {0.0067, 0.002, 0.0025, 0});
	// Vision search
	// Vision follow
}
void startRightLeftGoal() {
	chassis.setPose({7, 1.2, 0});

	chassis.moveToPose({7, 5.5, 0}, 0, {}, {{1.45, 2.7}}, {0}, {0.0067, 0.002, 0.0025, 0});
	frontClaw.toggle();
	chassis.moveToPose({8, 3, -90}, 0, {}, {{1.45, 2.7}}, {0}, {0.0067, 0.002, 0.0025, 0}, true);
	chassis.moveToPose({8.3, 3, -90}, 0, {}, {{1.45, 2.7}}, {0}, {0.0067, 0.002, 0.0025, 0}, true);
	frontArm.moveVelocity(30);
	backClaw.moveAbsolute(-450, 100);
	conveyor.moveVelocity(420);
	frontArm.moveVelocity(0);
	chassis.moveToPose({10, 6, 0}, 0, {}, {{0.5, 0.5}}, {0}, {0.0067, 0.002, 0.0025, 0});
	conveyor.moveVelocity(0);
	chassis.moveToPose({8, 4, -90}, 0, {}, {{1.45, 2.7}}, {0}, {0.0067, 0.002, 0.0025, 0});
	// Vision search
	// Vision follow
}

void startLeftRightGoal() {
	chassis.setPose({2.8, 1.2, 20});

	chassis.moveToPose({3, 5.2, 20}, 0, {}, {{1.45, 2.7}}, {0}, {0.0067, 0.002, 0.0025, 0});
	frontClaw.toggle();
	chassis.moveToPose({3.5, 2.2, -20}, 0, {}, {{1.45, 2.7}}, {0}, {0.0067, 0.002, 0.0025, 0}, true);
	backClaw.moveAbsolute(-50, 100);
	conveyor.moveVelocity(420);
	chassis.moveToPose({5, 4.5, 45}, 0, {{3.5, 3.5}}, {{1.45, 2.2}, {0.5, 0.5}}, {1.45}, {0.0067, 0.002, 0.0025, 0});
	conveyor.moveVelocity(0);
	chassis.moveToPose({2, 3, 90}, 0, {}, {{1.45, 2.7}}, {0}, {0.0067, 0.002, 0.0025, 0}, true);
	frontClaw.toggle();
	chassis.turnAbsolute(20, 1, {0.0067, 0.002, 0.0025, 0});
	// Vision search
	// Vision follow
}

void startRightFullAWP() {

}
void startLeftFullAWP() {

}

void skills() {
	
}