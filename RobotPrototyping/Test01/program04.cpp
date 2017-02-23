#include "stdafx.h"
using namespace robotlib;
#include "program04.h"

void Program04::Graph::render() {
	auto& window = prog->window;

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, window.width(), window.height(), 0, 1, -1);

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glBegin(GL_QUADS);
	glColor4f(1, 1, 1, 0.1f);
	glVertex3f(10, 10, 0);
	glVertex3f(screenSize.x + 10, 10, 0);
	glVertex3f(screenSize.x + 10, screenSize.y + 10, 0);
	glVertex3f(10, screenSize.y + 10, 0);
	glEnd();

	for (auto& channelEntry : channels) {
		auto& channel = *channelEntry.second;
		auto xrange = xRange();
		auto yrange = channel.yRange();
		if (xrange.second == xrange.first) xrange.second = xrange.first + 1;
		if (yrange.second == yrange.first) yrange.second = yrange.first + 1;
		auto mapperMaker = [](Range from, Range to) {
			return [=](double input) {
				return ((input - from.first) / (from.second - from.first)) 
					* (to.second - to.first) + to.first;
			};
		};
		auto xmap = mapperMaker(xrange, {10, screenSize.x + 10});
		auto ymap = mapperMaker(yrange, { screenSize.y + 10, 10});

		glBegin(GL_LINE_STRIP);
		glColor4f(channel.color.r, channel.color.g, channel.color.b, 0.5f);
		for (auto point : channel.getPoints()) {
			glVertex2f(xmap(point.x), ymap(point.y));
		}
		glEnd();
	}

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glEnable(GL_DEPTH_TEST); // TODO: push/pop this attribute
	glDisable(GL_BLEND);
}

void Program04::initPhysics() {
	typedef Physics::CRB CRB;

	robotlib::debugDrawMode = btIDebugDraw::DebugDrawModes::DBG_DrawWireframe;
	//| btIDebugDraw::DebugDrawModes::DBG_DrawFrames;

	//physics.create("createphysics.txt");

	auto ground = CRB::RigidBody();
	ground.shape = CRB::Shapes::Box(btVector3(10, 10, 1));
	ground.setPosition(btVector3(0, 0, -1));
	ground.mass = 0;
	ground.debugVisible = false;
	physics.create(ground);

	auto box = CRB::RigidBody();
	box.mass = 1;
	box.shape = CRB::Shapes::Box(btVector3(0.05, 0.05, 0.05));
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			for (int k = 0; k < 2; k++) {
				box.setPosition(btVector3(-0.9, 0, 0.2) + btVector3(i * 0.11f, j * 0.11f, k * 0.11f));
				physics.create(box);
			}

	auto root = CRB::RigidBody();
	root.id = "root";
	root.mass = 0;
	root.shape = CRB::Shapes::Box(btVector3(0.01, 0.01, 0.10));
	root.setPosition(btVector3(0, 0, 0.10));
	physics.create(root);

	auto bar = CRB::RigidBody();
	bar.id = "bar";
	bar.mass = 1;
	bar.shape = CRB::Shapes::Box(btVector3(0.20, 0.20, 0.01));
	bar.setPosition(btVector3(0, 0, 0.2 + 0.01));
	bar.canDeactivate = false;
	bar.debugVisible = false;
	bar.disableCollision = true;
	physics.create(bar);

	auto plate = CRB::RigidBody();
	plate.id = "plate";
	plate.mass = 1;
	plate.shape = CRB::Shapes::Box(btVector3(0.2, 0.2, 0.01));
	plate.setPosition(btVector3(0, 0, 0.2 + 0.02 + 0.01));
	plate.canDeactivate = false;
	physics.create(plate);

	auto firstHinge = CRB::HingeConstraint();
	firstHinge.id = "motor1";
	firstHinge.body1 = "root";
	firstHinge.body2 = "bar";
	firstHinge.localTransform1.setIdentity();
	firstHinge.localTransform1.setOrigin(btVector3(0, 0, 0.10 + 0.01));
	firstHinge.localTransform1.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_HALF_PI));
	firstHinge.localTransform2.setIdentity();
	firstHinge.localTransform2.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_HALF_PI));
	firstHinge.limits = { -0.5f, 0.5f };
	/*class VelFunc {
	public:
		btScalar totalTime = 0;
		btScalar operator() (btScalar timeStep) {
			totalTime += timeStep;
			return 5.0f * btSin(totalTime * SIMD_2_PI);
		}
	};
	firstHinge.velocityFunction = VelFunc();*/
	physics.create(firstHinge);
	motor1.reset(new Motor(&physics, dynamic_cast<btHingeConstraint*>(physics.getConstraint("motor1"))));

	{
		auto secondHinge = CRB::HingeConstraint();
		secondHinge.id = "motor2";
		secondHinge.body1 = "bar";
		secondHinge.body2 = "plate";
		auto& trans1 = secondHinge.localTransform1;
		auto& trans2 = secondHinge.localTransform2;
		trans1.setIdentity();
		trans1.setOrigin(btVector3(0, 0, 0.01 + 0.01));
		trans1.setRotation(btQuaternion(btVector3(0, 1, 0), SIMD_HALF_PI));
		trans2.setIdentity();
		trans2.setRotation(btQuaternion(btVector3(0, 1, 0), SIMD_HALF_PI));
		secondHinge.limits = { -0.5f, 0.5f };
		//secondHinge.velocityFunction = VelFunc();
		physics.create(secondHinge);
	}
	motor2.reset(new Motor(&physics, dynamic_cast<btHingeConstraint*>(physics.getConstraint("motor2"))));

	auto ball = CRB::RigidBody();
	ball.id = "ball";
	ball.shape = CRB::Shapes::Sphere(0.02);
	ball.mass = 0.1;
	ball.setPosition(btVector3(0, 0, 0.4));
	physics.create(ball);

	graph.screenSize = { 300, 100 };
	graph.setFixedRange("pos1", motor1->getAngularLimits());
	//graph.setFixedRange("pos2", motor2->getAngularLimits());
	//graph.showChannel("dx(pos1)");
}

void Program04::doPhysics()
{
	auto deltaTime = timer.stepTime();
	auto totalTime = timer.totalTime();

	auto targetVel = 5.0f * btSin(totalTime * SIMD_2_PI);
	//motor1->setTargetVelocity(targetVel);
	//motor2->setTargetVelocity(targetVel);
	//motor1->setTargetPosition(btSin(totalTime * SIMD_2_PI), 0.1);
	//motor2->setTargetPosition(btSin(totalTime * SIMD_2_PI), 0.1);

	auto ball = physics.getRigidBody("ball");
	auto ballPos = ball->getCenterOfMassPosition();

	auto limiter = [](double val, double low, double high) {
		if (val < low) return low;
		if (val > high) return high;
		return val;
	};

	double xpos = -ballPos.x();
	double ypos = -ballPos.y();

	xpos = limiter(xpos, -1, 1);
	ypos = limiter(ypos, -1, 1);
	motor1->setTargetPosition(xpos, 0.1);
	motor2->setTargetPosition(ypos, 0.1);

	physics.simulate(deltaTime);

	graph.setCurrentTime(timer.totalTime());
	graph.addDataPoint("pos1", motor1->getAngularPosition());
	//graph.addDataPoint("pos2", motor2->getAngularPosition());
	//graph.addDataPoint("tvel1", targetVel);

	/*auto ballPos = physics.getRigidBody("ball")->getCenterOfMassPosition();
	graph.addDataPoint("ballx", ballPos.x());
	graph.addDataPoint("bally", ballPos.y());
	graph.addDataPoint("ballz", ballPos.z());*/
}