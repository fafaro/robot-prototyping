#include "stdafx.h"

class Physics {
private:
	class DebugDrawer : public btIDebugDraw {
	public:
		virtual void	drawLine(const btVector3& from, const btVector3& to, const btVector3& color) {
			glBegin(GL_LINES);
			glColor3f(color.x(), color.y(), color.z());
			glVertex3f(from.x(), from.y(), from.z());
			glVertex3f(to.x(), to.y(), to.z());
			glEnd();
		}
		virtual void	drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color) {}
		virtual void	reportErrorWarning(const char* warningString) {}
		virtual void	draw3dText(const btVector3& location, const char* textString) {}
		virtual void	setDebugMode(int debugMode) {}
		virtual int		getDebugMode() const {
			return DBG_DrawWireframe; //DBG_MAX_DEBUG_DRAW_MODE;
		}
	};

	btCollisionConfiguration *m_collisionConfiguration;
	btDispatcher *m_dispatcher;
	btBroadphaseInterface *m_broadphase;
	btConstraintSolver *m_solver;
	btDynamicsWorld *m_dynamicsWorld;
	DebugDrawer m_debugDrawer;
	std::vector<btRigidBody*> m_rigidBodies;

public:
	// Language for creating rigid bodies
	class CRB {
	public:
		class Shape {
		public:
			virtual ~Shape() {}
		};

		class RigidBody {
			std::unique_ptr<Shape> shape;
		public:
			RigidBody() {
				transform.setIdentity();
			}
			void setShape(Shape *shape) { this->shape.reset(shape); }
			Shape *getShape() const { return this->shape.get(); }
			void setPosition(btVector3 const& pos) { transform.setOrigin(pos); }
			btTransform transform;
			btScalar mass;
			std::string id;
		};

		class Shapes {
		public:
			class Box : public Shape {
			public:
				btVector3 size;
				Box(btVector3 const& size) {
					this->size = size;
				}
				virtual ~Box() {}
			};
			class Plane : public Shape {
			public:
				btVector3 normal;
				btScalar distance = 0;

				Plane(btVector3 const& normal = btVector3(0, 0, 1), btScalar distance = 0) 
				{ 
					this->normal = normal; 
					this->distance = distance; 
				}
				virtual ~Plane() {} 
			};
		};
	};

	Physics() {
		m_collisionConfiguration = new btDefaultCollisionConfiguration();
		m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
		m_broadphase = new btDbvtBroadphase();
		btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
		m_solver = sol;
		m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
		m_dynamicsWorld->setGravity(btVector3(0, 0, -9.8));

		/////create a few basic rigid bodies
		//btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
		////m_collisionShapes.push_back(groundShape);

		//btTransform groundTransform;
		//groundTransform.setIdentity();
		//groundTransform.setOrigin(btVector3(0, 0, -50));

		//{
		//	btScalar mass(0.);
		//	auto rb = createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
		//	rb->setCollisionFlags(btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT);
		//}


		//{
		//	//create a few dynamic rigidbodies
		//	// Re-using the same collision is better for memory usage and performance

		//	btBoxShape* colShape = new btBoxShape(btVector3(.1, .1, .1));


		//	//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		//	//m_collisionShapes.push_back(colShape);

		//	/// Create Dynamic Objects
		//	btTransform startTransform;
		//	startTransform.setIdentity();

		//	btScalar	mass(1.f);

		//	//rigidbody is dynamic if and only if mass is non zero, otherwise static
		//	bool isDynamic = (mass != 0.f);

		//	btVector3 localInertia(0, 0, 0);
		//	if (isDynamic)
		//		colShape->calculateLocalInertia(mass, localInertia);


		//	for (int k = 0;k < 3;k++)
		//	{
		//		for (int i = 0;i < 3;i++)
		//		{
		//			for (int j = 0;j < 3;j++)
		//			{
		//				startTransform.setOrigin(btVector3(
		//					btScalar(0.2*i),
		//					btScalar(.2*k),
		//					btScalar(2 + .2*j)));


		//				createRigidBody(mass, startTransform, colShape);


		//			}
		//		}
		//	}
		//}

	}

	~Physics() {

	}

	void create(CRB::RigidBody const& rbdata) {
		auto shapeLhs = rbdata.getShape();
		btCollisionShape *shapeRhs = nullptr;
		btRigidBody *rbRhs = nullptr;

		if (typeid(*shapeLhs) == typeid(CRB::Shapes::Box)) {
			auto boxShapeLhs = (CRB::Shapes::Box*)shapeLhs;
			shapeRhs = new btBoxShape(boxShapeLhs->size);
		}
		else if (typeid(*shapeLhs) == typeid(CRB::Shapes::Plane)) {
			auto planeShapeLhs = *(CRB::Shapes::Plane*)shapeLhs;
			shapeRhs = new btStaticPlaneShape(planeShapeLhs.normal, planeShapeLhs.distance);
		}
		else throw new std::exception("Shape not defined!");

		btVector3 localInertia(0, 0, 0);
		shapeRhs->calculateLocalInertia(rbdata.mass, localInertia);
		btDefaultMotionState* myMotionState = new btDefaultMotionState(rbdata.transform);
		btRigidBody::btRigidBodyConstructionInfo cInfo(rbdata.mass, myMotionState, shapeRhs, localInertia);
		btRigidBody* body = new btRigidBody(cInfo);
		m_dynamicsWorld->addRigidBody(body);
		m_rigidBodies.push_back(body);
		m_rigidBodiesById[rbdata.id] = body;
		//return body;
	}

	std::map<std::string, btRigidBody*> m_rigidBodiesById;
	btRigidBody *getRigidBody(std::string const& id) {
		return m_rigidBodiesById[id];
	}

	void simulate(double t) {
		m_dynamicsWorld->stepSimulation(t);
	}

	void debugDraw() {
		m_dynamicsWorld->setDebugDrawer(&m_debugDrawer);
		m_dynamicsWorld->debugDrawWorld();
	}

	class RestartState {
	public:
		btTransform transform;
	};
	std::map<btRigidBody*, RestartState> restartStates;

	void saveForRestart() {
		restartStates.clear();
		for (auto rb : m_rigidBodies) {
			if (rb->getInvMass() == 0) continue;
			auto state = restartStates[rb];
			rb->getMotionState()->getWorldTransform(state.transform);
		}
	}

	void restart() {
		for (auto s : restartStates) {
			auto& rb = *s.first;
			rb.forceActivationState(ACTIVE_TAG);

			/*rb->setWorldTransform(s.second.transform);
			rb->getMotionState()->setWorldTransform(s.second.transform);
			rb->setAngularVelocity(btVector3(0, 0, 0));
			rb->setLinearVelocity(btVector3(0, 0, 0));*/

			rb.clearForces();
			btVector3 zeroVector(0, 0, 0);
			rb.setLinearVelocity(zeroVector);
			rb.setAngularVelocity(zeroVector);

			/*btVector3 initialRelativePosition;
			initialRelativePosition.setValue(getInitialRelativePosition());

			btQuaternion initialOrientation;
			initialOrientation.setValue(getInitialOrientation());*/

			//initialTransform.setOrigin(position + initialRelativePosition);
			//initialTransform.setRotation(initialOrientation);

			auto& initialTransform = s.second.transform;
			//initialTransform.setOrigin(btVector3(0, 0, 5));
			rb.setWorldTransform(initialTransform);
			rb.getMotionState()->setWorldTransform(initialTransform);
		}
	}

private:
	btRigidBody*	createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, const btVector4& color = btVector4(1, 0, 0, 1))
	{
		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			shape->calculateLocalInertia(mass, localInertia);

		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

		btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

		btRigidBody* body = new btRigidBody(cInfo);

		body->setUserIndex(-1);
		m_dynamicsWorld->addRigidBody(body);
		return body;
	}
};

class IMouseListener {
public:
	enum Button {
		None, Left, Right, Middle
	};

	class Args {
	public:
		Button button;
		int x, y;
	};

	virtual void mouseDown(Args const& args) {}
	virtual void mouseUp(Args const& args) {}
	virtual void mouseMove(Args const& args) {}
	virtual void mouseWheel(double amount) {}
};

class GLWindow {
public:
	GLFWwindow* window;
	std::vector<IMouseListener*> mouseListeners;
	std::map<char, std::list<std::function<void()>>> keyListeners;

	GLWindow() {
		/* Initialize the library */
		if (!glfwInit())
			throw std::exception("Unable to initialize glfw library.");

		/* Create a windowed mode window and its OpenGL context */
		window = glfwCreateWindow(640, 480, "Bullet Test01", NULL, NULL);
		if (!window)
		{
			glfwTerminate();
			throw std::exception("Unable to create glfw window.");
		}

		/* Make the window's context current */
		glfwMakeContextCurrent(window);

		glfwSetWindowUserPointer(window, (void*)this);

		glfwSetCursorPosCallback(window, [](auto win, auto x, auto y) {
			GLWindow *This = (GLWindow*)glfwGetWindowUserPointer(win);
			for (auto l : This->mouseListeners) {
				l->mouseMove(IMouseListener::Args { IMouseListener::Button::None, (int)x, (int)y });
			}
		});
		glfwSetMouseButtonCallback(window, [](auto win, auto btn, auto state, auto mods) {
			GLWindow *This = (GLWindow*)glfwGetWindowUserPointer(win);
			for (auto l : This->mouseListeners) {
				auto btnOut = IMouseListener::Button::None;
				switch (btn) {
				case GLFW_MOUSE_BUTTON_LEFT:
					btnOut = IMouseListener::Button::Left;
					break;
				case GLFW_MOUSE_BUTTON_RIGHT:
					btnOut = IMouseListener::Button::Right;
					break;
				case GLFW_MOUSE_BUTTON_MIDDLE:
					btnOut = IMouseListener::Button::Middle;
					break;
				}
				switch (state) {
				case GLFW_PRESS:
					l->mouseDown(IMouseListener::Args{ btnOut, 0, 0 });
					break;
				case GLFW_RELEASE:
					l->mouseUp(IMouseListener::Args{ btnOut, 0, 0 });
					break;
				}
			}
		});
		glfwSetScrollCallback(window, [](auto win, auto x, auto y) {
			GLWindow *This = (GLWindow*)glfwGetWindowUserPointer(win);
			for (auto l : This->mouseListeners) {
				l->mouseWheel(y);
			}
		});

		glfwSetFramebufferSizeCallback(window, [](auto win, auto width, auto height) {
			glViewport(0, 0, width, height);
		});

		glfwSetCharCallback(window, [](auto win, auto key) {
			GLWindow *This = (GLWindow*)glfwGetWindowUserPointer(win);
			if (This->keyListeners.find(key) == This->keyListeners.end()) return;
			for (auto& cb : This->keyListeners[(char)key]) {
				cb();
			}
		});
	}

	~GLWindow() {
		glfwTerminate();
	}

	bool isClosed() {
		return glfwWindowShouldClose(window);
	}

	void swapBuffers() {
		/* Swap front and back buffers */
		glfwSwapBuffers(window);
	}

	void processEvents() {
		/* Poll for and process events */
		glfwPollEvents();
	}

	void addListener(IMouseListener *listener) {
		mouseListeners.push_back(listener);
	}

	void addKeyCallback(char key, std::function<void()> cb) {
		keyListeners[key].push_back(cb);
	}

	int width() const {
		int w, h;
		glfwGetWindowSize(window, &w, &h);
		return w;
	}
	int height() const {
		int w, h;
		glfwGetWindowSize(window, &w, &h);
		return h;
	}
};

class Camera {
public:
	btVector3 position, lookAt;
	btScalar yaw, pitch, distance;

	Camera() : 
		position(0, 0, 0), 
		lookAt(0, 0, 0) 
	{
		yaw = -90;
		pitch = 30;
		distance = btScalar(2);
		update();
	}

	void update() 
	{
		btVector3 r(distance, 0, 0);
		r = r.rotate(btVector3(0, -1, 0), btRadians(pitch));
		r = r.rotate(btVector3(0, 0, 1), btRadians(yaw));
		position = r;
	}
};

class CameraController : public IMouseListener {
private:
	Camera *camera;
	int prevx, prevy;
	bool buttonDown = false;

public:
	CameraController(Camera *cam) :
		prevx(-1),
		prevy(-1)
	{
		camera = cam;
	}

	virtual void mouseMove(Args const& args) override 
	{
		if (prevx == -1) {
			prevx = args.x; prevy = args.y;
		}

		int deltax = args.x - prevx;
		int deltay = args.y - prevy;
		prevx = args.x;
		prevy = args.y;

		if (buttonDown) {
			auto limit = [](btScalar x, btScalar a, btScalar b) -> btScalar {
				if (x < a) return a;
				if (x > b) return b;
				return x;
			};

			camera->yaw -= deltax;
			camera->pitch = limit(camera->pitch + deltay, -87, 87);
			camera->update();
		}
	}

	virtual void mouseDown(Args const& args) override 
	{
		if (args.button == Button::Middle) buttonDown = true;
	}

	virtual void mouseUp(Args const& args) override
	{
		if (args.button == Button::Middle) buttonDown = false;
	}

	virtual void mouseWheel(double amount) override 
	{
		camera->distance *= std::pow(1.2, -amount);
		camera->update();
	}
};

class Program {
public:
	Physics physics;

	Program() {

	}

	void initPhysics() {
		auto ground = Physics::CRB::RigidBody();
		//ground.setShape(new Physics::CRB::Shapes::Plane(btVector3(0, 0, 1)));
		ground.setShape(new Physics::CRB::Shapes::Box(btVector3(1, 1, 1)));
		ground.setPosition(btVector3(0, 0, -1));
		ground.mass = 0;
		physics.create(ground);

		auto box = Physics::CRB::RigidBody();
		box.setShape(new Physics::CRB::Shapes::Box(btVector3(0.1, 0.1, 0.1)));
		box.setPosition(btVector3(0, 0, 20));
		box.mass = 1;
		physics.create(box);

		// create box A at [0, 0, 1] size [0.1, 0.1, 1] mass 1kg orient [0, 0, 0, 1] spin [0, 0, 1, 1]
		// A is fixed.
		// create box B at [0, 0, 2]
		// hinge constraing connnecting A to B
		// set motor program to move sinusoidally.
		physics.simulate(0);
		physics.saveForRestart();
	}

	void show() {
		GLWindow window;

		window.addKeyCallback('r', [&]() { printf("Restarting ...\n"); physics.restart(); });

		Camera camera;
		{
			CameraController ctrl(&camera);
			window.addListener(&ctrl);
		}
		auto drawGrid = []() {
			glBegin(GL_LINES);
			for (int i = -10; i <= 10; i++) {
				float extents = 10 * 0.1;
				float x = i * 0.1;
				glColor3f(0.1, 0.1, 0.1);
				glVertex3f(-extents, x, 0);
				glVertex3f(extents, x, 0);
				glVertex3f(x, -extents, 0);
				glVertex3f(x, extents, 0);
			}
			glEnd();
		};
		auto drawAxes = []() {
			glBegin(GL_LINES);
			float size = 0.1f;
			glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(size, 0, 0);
			glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, size, 0);
			glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, size);
			glEnd();
		};

		auto setupProjections = [&]() {
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			gluPerspective(60.0, (double)window.width() / window.height(), 0.1, 100.0);
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			gluLookAt(camera.position.x(), camera.position.y(), camera.position.z(), camera.lookAt.x(), camera.lookAt.y(), camera.lookAt.z(), 0, 0, 1);
		};

		/* Loop until the user closes the window */
		while (!window.isClosed())
		{
			physics.simulate(0.001);
			setupProjections();

			glClearColor(0.0f, 0, 0, 0);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			drawGrid();
			drawAxes();
			physics.debugDraw();

			window.swapBuffers();
			window.processEvents();
		}
	}

	int main() {
		initPhysics();
 		show();
		return 0;
	}
};

int main()
{
	Program program;
	return program.main();
}

void simpleSimulation() 
{
	Physics physics;
	{
		auto ground = Physics::CRB::RigidBody();
		//ground.setShape(new Physics::CRB::Shapes::Plane(btVector3(0, 0, 1)));
		ground.setShape(new Physics::CRB::Shapes::Box(btVector3(1, 1, 1)));
		ground.setPosition(btVector3(0, 0, -1));
		ground.mass = 0;
		ground.id = "ground";
		physics.create(ground);

		auto box = Physics::CRB::RigidBody();
		box.setShape(new Physics::CRB::Shapes::Box(btVector3(0.1, 0.1, 0.1)));
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

}