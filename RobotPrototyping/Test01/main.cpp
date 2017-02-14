#include "stdafx.h"
#include "../RobotLib/robotlib.h"

using namespace robotlib;

#pragma region Program
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

#if 0
	void initPhysics() {
		auto ground = Physics::CRB::RigidBody();
		//ground.setShape(new Physics::CRB::Shapes::Plane(btVector3(0, 0, 1)));
		ground.setShape(new Physics::CRB::Shapes::Box(btVector3(1, 1, 1)));
		ground.setPosition(btVector3(0, 0, -1));
		ground.mass = 0;
		ground.debugVisible = false;
		physics.create(ground);

		{
			auto box3 = Physics::CRB::RigidBody();
			box3.setShape(new Physics::CRB::Shapes::Box(btVector3(0.05, 0.05, 0.05)));
			box3.mass = 1;
			box3.setPosition(btVector3(-0.5, 0, 0.05));
			for (int i = 0; i < 5; i++) {
				box3.setPosition(btVector3(-0.5, 0, 0.05 + i * 0.1));
				physics.create(box3);
			}
		}

		auto createArm = [&](std::string const& name, btVector3 const& origin, bool stuck) {
			auto box1 = Physics::CRB::RigidBody();
			box1.setShape(new Physics::CRB::Shapes::Box(btVector3(0.04, 0.04, 0.15)));
			box1.setPosition(btVector3(0, 0, 0.15) + origin);
			box1.mass = stuck ? 0 : 0.1;
			box1.id = name + "." + "box1";
			physics.create(box1);

			auto box2 = Physics::CRB::RigidBody();
			box2.setShape(new Physics::CRB::Shapes::Box(btVector3(0.04, 0.04, 0.15)));
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
#endif

	void initPhysics() {
		typedef Physics::CRB CRB;

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
		bar.mass = 0.3;
		bar.shape = CRB::Shapes::Box(btVector3(0.10, 0.01, 0.01));
		bar.setPosition(btVector3(0, 0, 0.2 + 0.01));
		physics.create(bar);

		auto plate = CRB::RigidBody();
		plate.id = "plate";
		plate.mass = 0.1;
		plate.shape = CRB::Shapes::Box(btVector3(0.15, 0.15, 0.01));
		plate.setPosition(btVector3(0, 0, 0.2 + 0.02 + 0.01));
		physics.create(plate);

		auto firstHinge = CRB::HingeConstraint();
		firstHinge.body1 = "root";
		firstHinge.body2 = "bar";
		firstHinge.localTransform1.setIdentity();
		firstHinge.localTransform1.setOrigin(btVector3(0, 0, 0.10 + 0.01));
		firstHinge.localTransform1.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_HALF_PI));
		firstHinge.localTransform2.setIdentity();
		firstHinge.localTransform2.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_HALF_PI));
		firstHinge.limits = { -0.3f, 0.3f };
		physics.create(firstHinge);

		auto secondHinge = CRB::HingeConstraint();
		secondHinge.body1 = "bar";
		secondHinge.body2 = "plate";
		secondHinge.localTransform1.setIdentity();
		secondHinge.localTransform1.setOrigin(btVector3(0, 0, 0.01 + 0.01));
		secondHinge.localTransform1.setRotation(btQuaternion(btVector3(0, 1, 0), SIMD_HALF_PI));
		secondHinge.localTransform2.setIdentity();
		secondHinge.localTransform2.setRotation(btQuaternion(btVector3(0, 1, 0), SIMD_HALF_PI));
		secondHinge.limits = { -0.3f, 0.3f };
		physics.create(secondHinge);

		auto ball = CRB::RigidBody();
		ball.shape = CRB::Shapes::Sphere(0.02);
		ball.mass = 0.1;
		ball.setPosition(btVector3(0, 0, 0.4));
		physics.create(ball);
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
#pragma endregion

#pragma region Main
void render(std::function<void()> r)
{
	GLViewWindow window;
	window.renderFunction = r;
	window.run();
}

int program02() {
	class URDFData {
	public:
		class Joint;

		class Geometry {
		public:
			class Box {
			public:
				glm::vec3 size;
			};
		};

		class Link
		{
		public:
			std::string name;
			Link *parent;
			std::list<Joint> joints;

			struct Collision {
				glm::mat4 origin;
				boost::any geometry;
			};
			boost::optional<Collision> collision;

			Link(std::string name) : name(name), parent(nullptr) {}
		};

		class Joint
		{
		public:
			Link *child;
			glm::mat4 origin;
			glm::vec3 axis;
		};

		class LinkPool : public std::map<std::string, Link*>
		{
		public:
			Link* get(std::string const& name)
			{
				auto x = this->find(name);
				if (x != this->end()) return x->second;
				return ((*this)[name] = new Link(name));
			}

			Link* getRoot()
			{
				for (auto& entry : *this)
					if (!entry.second->parent)
						return entry.second;
				return nullptr;
			}

			~LinkPool() {
				for (auto& entry : *this)
					delete entry.second;
				this->clear();
			}
		};
	};

	class URDFDoc : private tinyxml2::XMLDocument
	{
	public:
		class JointElement
		{
		private:
			tinyxml2::XMLElement *elem;
		public:
			JointElement(tinyxml2::XMLElement* elem) : elem(elem) {}
			std::string name() const { return elem->Attribute("name");  }
			std::string parent() const { 
				return elem->FirstChildElement("parent")->Attribute("link");
			}
			std::string child() const {
				return elem->FirstChildElement("child")->Attribute("link");
			}
			glm::mat4 origin() const {
				return originFromElement(elem->FirstChildElement("origin"));
			}

			boost::optional<glm::vec3> axis() const {
				if (auto axisElem = elem->FirstChildElement("axis"))
					return strToVec3(axisElem->Attribute("xyz"));
				return {};
			}
		};

		class LinkElement
		{
		public:
			tinyxml2::XMLElement *elem;
			LinkElement(tinyxml2::XMLElement *elem) : elem(elem) {}

			std::string name() const { return elem->Attribute("name"); }
			bool hasCollision() const { return elem->FirstChildElement("collision"); }
			auto getCollisionGeometry() const 
			{ 
				return elem
					->FirstChildElement("collision")
					->FirstChildElement("geometry")
					->FirstChildElement(); 
			}
			glm::mat4 getCollisionOrigin() const
			{
				return originFromElement(elem
					->FirstChildElement("collision")
					->FirstChildElement("origin"));
			}
		};

		URDFDoc(std::string const& s) 
		{
			LoadFile(s.c_str());
		}

	private:
		auto allElementsByTag(std::string const& name) {
			class Iterator {
			public:
				typedef tinyxml2::XMLElement element_type;
				tinyxml2::XMLElement *element;
				std::string name;

				Iterator(tinyxml2::XMLElement *p, std::string const& name) : element(p), name(name) {}
				void operator++() { element = element->NextSiblingElement(name.c_str()); }
				bool operator!=(Iterator const& rhs) const { return element != rhs.element; }
				auto operator*() { return element; }
			};
			class Iterable {
			public:
				typedef Iterator iterator_type;
				URDFDoc *doc;
				std::string name;

				Iterable(URDFDoc *doc, std::string const& name) : doc(doc), name(name) {}
				Iterator begin() {
					return Iterator(doc->RootElement()->FirstChildElement(name.c_str()), name);
				}
				Iterator end() {
					return Iterator(0, name);
				}
			} iterable(this, name);
			return iterable;
		}

	public:
		auto allJoints() { return allElementsByTag("joint"); }
		auto allLinks() { return allElementsByTag("link"); }

		static glm::vec3 strToVec3(std::string const& s) {
			double x, y, z;
			std::istringstream iss(s);
			iss >> x >> y >> z;
			return glm::vec3(x, y, z);
		}

		static glm::mat4 originFromElement(tinyxml2::XMLElement *elem) 
		{
			glm::mat4 transform;
			if (auto rpyText = elem->Attribute("rpy")) {
				auto rpy = strToVec3(rpyText);
				//transform = glm::eulerAngleXYZ(rpy.x, rpy.y, rpy.z);
				//transform = glm::yawPitchRoll(rpy.z, rpy.y, rpy.x);
				//transform = glm::yawPitchRoll(rpy.y, rpy.x, rpy.z);
				//rpy.x += glm::half_pi<float>();
				transform = (glm::mat4)quatFromRPY(rpy);
			}
			if (auto xyzText = elem->Attribute("xyz")) {
				transform[3] = glm::vec4(strToVec3(xyzText), 1);
			}
			return transform;
		}

		static glm::quat quatFromRPY(glm::vec3 const& rpy) 
		{
			glm::quat result;

			double phi = rpy.x / 2.0;
			double the = rpy.y / 2.0;
			double psi = rpy.z / 2.0;

			result.x = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
			result.y = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
			result.z = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);
			result.w = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);

			return glm::normalize(result);
		}
	};

	URDFDoc doc("../data/cyton_gamma_1500/cyton_gamma_1500.urdf");
	URDFData::LinkPool linkPool;

	for (auto x : doc.allJoints()) {
		auto joint = URDFDoc::JointElement(x);
		auto parent = linkPool.get(joint.parent());
		auto child = linkPool.get(joint.child());
		auto axis = joint.axis();
		parent->joints.push_back(URDFData::Joint{ child, joint.origin(), axis ? *axis : glm::vec3(0,0,0) });
		child->parent = parent;
	}
	auto rootLink = linkPool.getRoot();
	std::cout << rootLink->name;

	for (auto linkElemInner : doc.allLinks()) {
		auto linkElem = URDFDoc::LinkElement(linkElemInner);
		auto link = linkPool.get(linkElem.name());
		if (linkElem.hasCollision()) {
			auto collGeom = linkElem.getCollisionGeometry();
			if (collGeom->Name() == std::string("box")) {
				auto size = URDFDoc::strToVec3(collGeom->Attribute("size"));
				link->collision = URDFData::Link::Collision{ 
					linkElem.getCollisionOrigin(), 
					URDFData::Geometry::Box { size } 
				};
			}
		}
	}

	render([&]() {
		std::function<void(URDFData::Link*)> drawLink = [&](URDFData::Link *link) {
			glMatrixMode(GL_MODELVIEW);

			GLUtil::drawAxes();
			if (link->collision) {
				glPushMatrix();
				glMultMatrixf(glm::value_ptr(link->collision->origin));
				//glRotatef(90, 1, 0, 0);

				if (link->collision->geometry.type() == typeid(URDFData::Geometry::Box)) {
					auto box = boost::any_cast<URDFData::Geometry::Box>(link->collision->geometry);
					GLUtil::drawBox(box.size);
				}
				glPopMatrix();
			}

			for (auto joint : link->joints) {
				GLUtil::drawLine(glm::vec3(0, 0, 0), joint.origin[3]);

				glPushMatrix();
				glMultMatrixf(glm::value_ptr(joint.origin));
				GLUtil::drawLine(glm::vec3(0, 0, 0), joint.axis, glm::vec3(1, 0, 1));
				drawLink(joint.child);
				glPopMatrix();
			}
		};
		drawLink(rootLink);
	});
	return 0;
}

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

int main()
{
	if (!glfwInit()) throw std::exception("Unable to initialize glfw library.");

	int choice = 1;
	switch (choice) {
	case 1:
		return Program::main();
	case 2:
		return program02(); // URDF loader
	case 3: 
		return program03();
	}
}

#pragma endregion

