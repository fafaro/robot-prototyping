#pragma once

class Program {
public:
	Timer                timer;
	Physics              physics;
	GLViewWindow         window;
	PickObjectController pickCtrl;
	bool                 pauseSimulation = true;

	Program() :
		pickCtrl(&window.camera, &physics, &window)
	{
		window.addKeyCallback('r', [this]() { printf("Restarting ...\n"); physics.restart(); });
		window.addKeyCallback('p', [this]() { pauseSimulation = !pauseSimulation; });
		window.addListener(&pickCtrl);
	}

	void initPhysics() {
		auto ground = Physics::CRB::RigidBody();
		//ground.setShape(new Physics::CRB::Shapes::Plane(btVector3(0, 0, 1)));
		ground.shape = Physics::CRB::Shapes::Box(btVector3(1, 1, 1));
		ground.setPosition(btVector3(0, 0, -1));
		ground.mass = 0;
		ground.debugVisible = false;
		physics.create(ground);

		{
			auto box3 = Physics::CRB::RigidBody();
			box3.shape = Physics::CRB::Shapes::Box(btVector3(0.05, 0.05, 0.05));
			box3.mass = 1;
			box3.setPosition(btVector3(-0.5, 0, 0.05));
			for (int i = 0; i < 5; i++) {
				box3.setPosition(btVector3(-0.5, 0, 0.05 + i * 0.1));
				physics.create(box3);
			}
		}

		auto createArm = [&](std::string const& name, btVector3 const& origin, bool stuck) {
			auto box1 = Physics::CRB::RigidBody();
			box1.shape = Physics::CRB::Shapes::Box(btVector3(0.04, 0.04, 0.15));
			box1.setPosition(btVector3(0, 0, 0.15) + origin);
			box1.mass = stuck ? 0 : 0.1;
			box1.id = name + "." + "box1";
			physics.create(box1);

			auto box2 = Physics::CRB::RigidBody();
			box2.shape = Physics::CRB::Shapes::Box(btVector3(0.04, 0.04, 0.15));
			box2.mass = 0.1;
			box2.setPosition(btVector3(0, 0.1, 0.45) + origin);
			//box2.transform.setRotation(btQuaternion(btVector3(1, 0, 0), 1.0));
			box2.id = name + "." + "box2";
			box2.canDeactivate = false;
			physics.create(box2);

			auto c = Physics::CRB::HingeConstraint();
			c.id = name + "." + "c1";
			c.body1 = name + "." + "box1";
			c.body2 = name + "." + "box2";
			c.localTransform1.setIdentity();
			c.localTransform1.setOrigin(btVector3(0, 0, 0.15));
			c.localTransform1.setRotation(btQuaternion(btVector3(0, 1, 0), btRadians(90)));
			c.localTransform2.setIdentity();
			c.localTransform2.setOrigin(btVector3(0, 0, -0.15));
			c.localTransform2.setRotation(btQuaternion(btVector3(0, 1, 0), btRadians(90)));
			c.limits = { btRadians(-160), btRadians(160) };
			physics.create(c);
		};
		createArm("arm1", btVector3(0, 0, 0), true);
		createArm("arm2", btVector3(0.5, 0, 0), false);

		static struct InternalTickData {
			Physics* physics;
			btScalar time;
		} internalTickData;

		internalTickData.physics = &physics;
		internalTickData.time = 0;

		auto& bphysics = *physics.getWorld();
		bphysics.setInternalTickCallback([](auto world, auto timestep) {
			auto& data = *static_cast<InternalTickData*>(world->getWorldUserInfo());
			auto doMotor = [&](std::string const& name) {
				auto c = static_cast<btHingeConstraint*>(data.physics->getConstraint(name + "." + "c1"));
				//c->setMotorTarget(0, timestep);
				c->enableAngularMotor(true, btSin(data.time), 0.5);
				//c->enableMotor(true);
				//c->setMotorTarget(btSin(data.time * 0.5) * 0.1, timestep);
			};
			doMotor("arm1");
			doMotor("arm2");
			printf("Internal tick %f\r\n", timestep);
			data.time += timestep;
		}, &internalTickData, true);
	}

	void renderPhysics()
	{
		physics.debugDraw();
	}

	void doPhysics()
	{
		physics.simulate(timer.stepTime());
	}

	void show()
	{
		window.renderFunction = [&]() {
			if (!pauseSimulation) doPhysics();
			renderPhysics();
		};
		window.run();
	}

	static int main() {
		Program program;
		program.initPhysics();
		program.show();
		return 0;
	}
};

