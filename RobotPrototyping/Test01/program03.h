#pragma once

int program03()
{
	Physics physics;
	{
		auto ground = Physics::CRB::RigidBody();
		//ground.setShape(new Physics::CRB::Shapes::Plane(btVector3(0, 0, 1)));
		ground.shape = Physics::CRB::Shapes::Box(btVector3(1, 1, 1));
		ground.setPosition(btVector3(0, 0, -1));
		ground.mass = 0;
		ground.id = "ground";
		physics.create(ground);

		auto box = Physics::CRB::RigidBody();
		box.shape = Physics::CRB::Shapes::Box(btVector3(0.1, 0.1, 0.1));
		box.setPosition(btVector3(0, 0, 10));
		box.mass = 1;
		box.id = "box";
		physics.create(box);
	}

	auto box = physics.getRigidBody("box");
	for (double t = 0; t < 10; t += 0.1) {
		btTransform trans;
		box->getMotionState()->getWorldTransform(trans);
		auto o = trans.getOrigin();

		printf("%f %f %f\n", o.x(), o.y(), o.z());

		physics.simulate(t);
	}
	getchar();
	return 0;
}
