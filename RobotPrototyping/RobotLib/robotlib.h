#pragma once

#pragma region Includes
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <map>
#include <list>
#include <memory>
#include <functional>

#include <boost/optional/optional.hpp>
#include <boost/any.hpp>

#include "btBulletDynamicsCommon.h"

#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>

#include <tinyxml2.h>
#pragma endregion

namespace robotlib
{
	void init();
	void render(std::function<void()> r);

#pragma region Conversions
	class ConvertVec3
	{
	public:
		glm::vec3 p;

		ConvertVec3(glm::vec3 const& p) : p(p) {}
		ConvertVec3(btVector3 const& p) : p(p.x(), p.y(), p.z()) {}
		operator glm::vec3&() { return p; }
		operator btVector3() { return btVector3(p.x, p.y, p.z); }
	};
#pragma endregion 

#pragma region Interfaces
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
#pragma endregion

#pragma region Classes

	static int debugDrawMode
		= btIDebugDraw::DebugDrawModes::DBG_DrawWireframe
		| btIDebugDraw::DebugDrawModes::DBG_DrawConstraints
		| btIDebugDraw::DebugDrawModes::DBG_DrawConstraintLimits;
	//DBG_MAX_DEBUG_DRAW_MODE

	class Point
	{
	public:
		btScalar x, y;
		Point() : x(0), y(0) {}
		Point(btScalar x, btScalar y) : x(x), y(y) {}
	};

	class Physics {
	public:
		// Language for creating rigid bodies
		class CRB {
		public:
			class Shape {
			public:
				virtual ~Shape() {}
			};

			class RigidBody {
			public:
				RigidBody() {
					transform.setIdentity();
				}

				void setPosition(btVector3 const& pos) { transform.setOrigin(pos); }

				boost::any shape;
				btTransform transform;
				btScalar mass = 1;
				std::string id = "default";
				bool debugVisible = true;
				bool canDeactivate = true;
				bool disableCollision = false;
			};

			class Shapes {
			public:
				class Box : public Shape {
				public:
					btVector3 size = btVector3(1, 1, 1);
					Box() {}
					Box(btVector3 const& size) {
						this->size = size;
					}
					virtual ~Box() {}
				};
				class Sphere : public Shape {
				public:
					btScalar radius = 1;
					Sphere() {}
					Sphere(btScalar radius) : radius(radius) {}
					virtual ~Sphere() {}
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

			class HingeConstraint {
			public:
				std::string id;
				std::string body1, body2;
				btTransform localTransform1, localTransform2;
				struct { btScalar lower, upper; } limits;
				std::function<btScalar(btScalar)> velocityFunction;
			};
		};

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
				return debugDrawMode;
			}
		};

		class CreateScript {
		public:
			struct Statement {
				enum { RB, C } type;
				int index;
			};
			std::vector<CRB::RigidBody> m_rbdata;
			std::vector<CRB::HingeConstraint> m_cdata;
			std::list<Statement> m_statements;

			void append(CRB::RigidBody const& rbdata) {
				m_rbdata.push_back(rbdata);
				m_statements.push_back(Statement{ Statement::RB, (int)m_rbdata.size() - 1 });
			}

			void append(CRB::HingeConstraint const& hingedata) {
				m_cdata.push_back(hingedata);
				m_statements.push_back(Statement{ Statement::C, (int)m_cdata.size() - 1 });
			}

			void run(Physics& physics) {
				physics.createScriptRunning = true;
				for (auto s : m_statements) {
					switch (s.type) {
					case Statement::RB:
						physics.create(m_rbdata[s.index]);
						break;
					case Statement::C:
						physics.create(m_cdata[s.index]);
						break;
					}
				}
				physics.createScriptRunning = false;
			}
		};

		btCollisionConfiguration*                    m_collisionConfiguration = nullptr;
		btDispatcher*                                m_dispatcher = nullptr;
		btBroadphaseInterface*                       m_broadphase = nullptr;
		btConstraintSolver*                          m_solver = nullptr;
		btDynamicsWorld*                             m_dynamicsWorld = nullptr;
		DebugDrawer                                  m_debugDrawer;
		std::vector<btRigidBody*>                    m_rigidBodies;
		std::vector<btTypedConstraint*>              m_constraints;
		std::map<std::string, btRigidBody*>          m_rigidBodiesById;
		std::map<std::string, btTypedConstraint*>    m_constraintsById;
		std::map<std::string, CRB::HingeConstraint*> m_constraintDataById;
		CreateScript                                 createScript;

	public:
		Physics(btDynamicsWorld *dynWorld = nullptr) {
			if (!dynWorld) {
				m_collisionConfiguration = new btDefaultCollisionConfiguration();
				m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
				m_broadphase = new btDbvtBroadphase();
				btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
				m_solver = sol;
				m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
				m_dynamicsWorld->setGravity(btVector3(0, 0, -9.8));
			}
			else {
				m_dynamicsWorld = dynWorld;
			}

			m_dynamicsWorld->setInternalTickCallback([](btDynamicsWorld *world, btScalar timeStep) {
				Physics *This = (Physics*)world->getWorldUserInfo();
				This->internalPreTick(timeStep);
			}, (void*)this, true);
		}

		~Physics() {

		}

		bool createScriptRunning = false;

		void create(CRB::RigidBody const& rbdata) {
			if (!createScriptRunning) createScript.append(rbdata);
			auto shapeLhs = rbdata.shape;
			btCollisionShape *shapeRhs = nullptr;

			if (shapeLhs.type() == typeid(CRB::Shapes::Box)) {
				auto boxShapeLhs = boost::any_cast<CRB::Shapes::Box>(shapeLhs);
				shapeRhs = new btBoxShape(boxShapeLhs.size);
			}
			else if (shapeLhs.type() == typeid(CRB::Shapes::Sphere)) {
				auto sphereShapeLhs = boost::any_cast<CRB::Shapes::Sphere>(shapeLhs);
				shapeRhs = new btSphereShape(sphereShapeLhs.radius);
			}
			else if (shapeLhs.type() == typeid(CRB::Shapes::Plane)) {
				auto planeShapeLhs = boost::any_cast<CRB::Shapes::Plane>(shapeLhs);
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

			if (!rbdata.debugVisible) {
				body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT);
			}

			if (!rbdata.canDeactivate) {
				body->setActivationState(DISABLE_DEACTIVATION);
			}

			if (rbdata.disableCollision) {
				body->setCollisionFlags(
					body->getCollisionFlags() | btCollisionObject::CollisionFlags::CF_NO_CONTACT_RESPONSE);
			}
		}

		void create(CRB::HingeConstraint const& hc) {
			if (!createScriptRunning) createScript.append(hc);
			auto body1 = m_rigidBodiesById[hc.body1];
			auto body2 = m_rigidBodiesById[hc.body2];
			auto c = new btHingeConstraint(*body1, *body2, hc.localTransform1, hc.localTransform2);
			//c->setAxis(btVector3(1, 0, 0));
			c->setLimit(hc.limits.lower, hc.limits.upper); // radians
			m_dynamicsWorld->addConstraint(c, true);
			m_constraints.push_back(c);
			m_constraintsById[hc.id] = c;
			c->setDbgDrawSize(0.1);
			c->enableMotor(true);

			auto hcdata = new CRB::HingeConstraint(hc);
			m_constraintDataById[hc.id] = hcdata;
			c->setUserConstraintPtr((void*)hcdata);
		}

		btRigidBody *getRigidBody(std::string const& id) {
			return m_rigidBodiesById[id];
		}

		btTypedConstraint *getConstraint(std::string const& id) {
			return m_constraintsById[id];
		}

		btDynamicsWorld *getWorld() { return m_dynamicsWorld; }

		void simulate(double t) {
			m_dynamicsWorld->stepSimulation(t);
		}

		#pragma region PreTick
	public:
		typedef std::function<void(double)> PreTickCallback;

	private:
		std::list<PreTickCallback> m_preTickCallbacks;
		std::map<int, decltype(m_preTickCallbacks)::iterator> m_preTickCallbackIds;
		int m_preTickCallbackNewId = 1;

	public:
		int registerPreTick(PreTickCallback const& cb) {
			m_preTickCallbacks.push_back(cb);
			int id = m_preTickCallbackNewId++;
			auto iter = m_preTickCallbacks.end();
			iter--;
			m_preTickCallbackIds[id] = iter;
			return id;
		}

		void deregisterPreTick(int id) {
			auto foundIter = m_preTickCallbackIds.find(id);
			if (foundIter != m_preTickCallbackIds.end()) {
				m_preTickCallbacks.erase(m_preTickCallbackIds[id]);
				m_preTickCallbackIds.erase(id);
			}
		}

		void internalPreTick(btScalar timeStep)
		{
			//std::cout << "PreTick " << timeStep << std::endl;
			//for (auto& cons : m_constraints) {
			//	CRB::HingeConstraint *hdata = (decltype(hdata))cons->getUserConstraintPtr();
			//	btHingeConstraint *hc = (decltype(hc))cons;
			//	if (hdata && hdata->velocityFunction) {
			//		hc->enableAngularMotor(true, hdata->velocityFunction(timeStep), 1);
			//	}
			//}

			for (auto cb : m_preTickCallbacks)
				cb(timeStep);
		}
		#pragma endregion

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
			return;
			restartStates.clear();
			for (auto rb : m_rigidBodies) {
				if (rb->getInvMass() == 0) continue;
				auto state = restartStates[rb];
				rb->getMotionState()->getWorldTransform(state.transform);
			}
		}

		void reset() {
			for (auto c : m_constraints) {
				m_dynamicsWorld->removeConstraint(c);
				delete c;
			}
			m_constraints.clear();

			for (auto& cdata : m_constraintDataById) {
				delete cdata.second;
			}
			m_constraintDataById.clear();

			for (auto rb : m_rigidBodies) {
				m_dynamicsWorld->removeRigidBody(rb);
				delete rb;
			}
			m_rigidBodies.clear();
			m_rigidBodiesById.clear();
		}

		void restart() {
			reset();
			createScript.run(*this);
			return;
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

	};

	class GLUtil
	{
	public:
		static glm::vec3 color;

		static void drawLine(glm::vec3 const& a, glm::vec3 const& b)
		{
			drawLine(a, b, color);
		}

		static void drawLine(glm::vec3 const& a, glm::vec3 const& b, glm::vec3 const& col)
		{
			glBegin(GL_LINES);
			glColor3f(col.x, col.y, col.z);
			glVertex3f(a.x, a.y, a.z);
			glVertex3f(b.x, b.y, b.z);
			glEnd();
		}

		static void drawPointStar(glm::vec3 const& p)
		{
			auto size = 0.01;
			glPushMatrix();
			glTranslatef(p.x, p.y, p.z);
			glBegin(GL_LINES);
			glColor3f(color.x, color.y, color.z);
			glVertex3f(size, 0, 0); glVertex3f(-size, 0, 0);
			glVertex3f(0, size, 0); glVertex3f(0, -size, 0);
			glVertex3f(0, 0, size); glVertex3f(0, 0, -size);
			glEnd();
			glPopMatrix();
		}

		static void drawAxes(float size = 0.1f)
		{
			glBegin(GL_LINES);
			glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(size, 0, 0);
			glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, size, 0);
			glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, size);
			glEnd();
		}

		static void drawBox(glm::vec3 const& size)
		{
			auto h = size * 0.5f;
			glBegin(GL_LINES);
			glColor3f(color.x, color.y, color.z);
			glVertex3f(-h.x, -h.y, -h.z);
			glVertex3f(h.x, -h.y, -h.z);

			glVertex3f(-h.x, h.y, -h.z);
			glVertex3f(h.x, h.y, -h.z);

			glVertex3f(-h.x, h.y, h.z);
			glVertex3f(h.x, h.y, h.z);

			glVertex3f(-h.x, -h.y, h.z);
			glVertex3f(h.x, -h.y, h.z);
			glEnd();

			glBegin(GL_LINE_LOOP);
			glVertex3f(-h.x, -h.y, -h.z);
			glVertex3f(-h.x, h.y, -h.z);
			glVertex3f(-h.x, h.y, h.z);
			glVertex3f(-h.x, -h.y, h.z);
			glEnd();

			glBegin(GL_LINE_LOOP);
			glVertex3f(h.x, -h.y, -h.z);
			glVertex3f(h.x, h.y, -h.z);
			glVertex3f(h.x, h.y, h.z);
			glVertex3f(h.x, -h.y, h.z);
			glEnd();
		}
	};

	class GLWindow {
	public:
		GLFWwindow* window;
		std::vector<IMouseListener*> mouseListeners;
		std::map<char, std::list<std::function<void()>>> keyListeners;

		GLWindow() {
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
					l->mouseMove(IMouseListener::Args{ IMouseListener::Button::None, (int)x, (int)y });
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

			glfwSetKeyCallback(window, [](GLFWwindow* window, int key, int scancode, int action, int mods) {
				if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
					glfwDestroyWindow(window);
			});
		}

		~GLWindow() {
			glfwTerminate();
		}

		bool isClosed() {
			return glfwWindowShouldClose(window) != 0;
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

		Point normalizeCoords(Point const& screenPoint) const
		{
			auto map = [](btScalar val, btScalar domMin, btScalar domMax, btScalar rangeMin, btScalar rangeMax) {
				return (((val - domMin) / (domMax - domMin)) * (rangeMax - rangeMin)) + rangeMin;
			};
			return Point(
				map(screenPoint.x, 0, width(), -1, 1),
				map(screenPoint.y, 0, height(), 1, -1));
		}
	};

	class Camera {
	public:
		btVector3 position, lookAt;
		btScalar yaw, pitch, distance;
		btScalar aspectRatio;
		glm::mat4 modelView, projection;

		Camera() :
			position(0, 0, 0),
			lookAt(0, 0, 0)
		{
			aspectRatio = 4.0 / 3.0;
			yaw = -90;
			pitch = 30;
			distance = btScalar(2);
			update();
			updateMatrices();
		}

		void update()
		{
			btVector3 r(distance, 0, 0);
			r = r.rotate(btVector3(0, -1, 0), btRadians(pitch));
			r = r.rotate(btVector3(0, 0, 1), btRadians(yaw));
			position = r;
		}

		void updateMatrices()
		{
			projection = glm::perspective((double)btRadians(60.0), (double)aspectRatio, 0.1, 100.0);
			modelView = glm::lookAt(glm::vec3(position.x(), position.y(), position.z()), glm::vec3(lookAt.x(), lookAt.y(), lookAt.z()), glm::vec3(0, 0, 1));
		}

		std::tuple<btVector3, btVector3> getRay(Point const& p) const
		{
			auto m = projection * modelView;
			m = glm::inverse(m);
			auto r = m  * glm::vec4(p.x, p.y, 0, 1);
			r.w = 1.0 / r.w;
			r.x *= r.w;
			r.y *= r.w;
			r.z *= r.w;
			return{ position, btVector3(r.x, r.y, r.z) };
		}

		btVector3 screenToWorld(Point const& p, btVector3 const& depthPoint)
		{
			auto normalizeW = [](glm::vec4& p) {
				p.x /= p.w;
				p.y /= p.w;
				p.z /= p.w;
				p.w = 1;
			};

			auto m = projection * modelView;
			auto screenDepth = (m * glm::vec4((glm::vec3)(ConvertVec3)depthPoint, 1));
			normalizeW(screenDepth);

			m = glm::inverse(m);
			auto r = m  * glm::vec4(p.x, p.y, screenDepth.z, 1);
			normalizeW(r);
			return (ConvertVec3)r;
		}
	};

	class CameraController : public IMouseListener
	{
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

	class PickObjectController : public IMouseListener
	{
	private:
		const IMouseListener::Button PICK_BUTTON = IMouseListener::Button::Left;

		Camera*   m_camera = nullptr;
		Physics*  m_physics = nullptr;
		GLWindow* m_window = nullptr;
		btVector3 m_point = btVector3(1, 0, 0);
		Point     m_cursorPos = Point(0, 0);

		btVector3    m_pickedPos = btVector3(0, 0, 0);
		btRigidBody* m_pickedBody = nullptr;
		btPoint2PointConstraint *m_pickedConstraint = nullptr;

	public:
		PickObjectController(Camera *camera, Physics *physics, GLWindow *window)
		{
			m_camera = camera;
			m_physics = physics;
			m_window = window;
		}

		virtual void mouseDown(Args const& args) override
		{
			if (args.button == PICK_BUTTON) {
				glm::vec3 hitPoint;
				btRigidBody *hitBody;
				if (hit(m_cursorPos, hitPoint, hitBody)) {
					if (!hitBody) return;
					bool movable = !hitBody->isStaticOrKinematicObject();
					if (!movable) return;

					m_pickedBody = hitBody;
					m_pickedPos = btVector3(hitPoint.x, hitPoint.y, hitPoint.z);

					createPointConstraint();
					printf("Movable!\r\n");
				}
			}
		}

		virtual void mouseUp(Args const& args) override
		{
			if (m_pickedConstraint) {
				m_physics->getWorld()->removeConstraint(m_pickedConstraint);
				delete m_pickedConstraint;
				m_pickedConstraint = nullptr;
			}
		}

		virtual void mouseMove(Args const& args) override
		{
			m_cursorPos = Point(args.x, args.y);
			if (m_pickedConstraint) {
				auto newp = m_camera->screenToWorld(m_window->normalizeCoords(m_cursorPos), m_pickedPos);
				m_pickedConstraint->setPivotB(newp);
			}
		}

		void createPointConstraint() {
			//m_pickedBody = body;
			//m_savedState = m_pickedBody->getActivationState();
			//m_pickedBody->setActivationState(DISABLE_DEACTIVATION);
			m_pickedBody->activate();

			btVector3 localPivot = m_pickedBody->getCenterOfMassTransform().inverse() * m_pickedPos;
			m_pickedConstraint = new btPoint2PointConstraint(*m_pickedBody, localPivot);
			m_pickedConstraint->setDbgDrawSize(0.01);
			m_physics->getWorld()->addConstraint(m_pickedConstraint, true);
			//m_pickedConstraint = p2p;
			//btScalar mousePickClamping = 30.f;
			//p2p->m_setting.m_impulseClamp = mousePickClamping;
			//p2p->m_setting.m_tau = 0.001f;
			//	m_oldPickingPos = rayToWorld;
			//	m_hitPos = pickPos;
			//	m_oldPickingDist = (pickPos - rayFromWorld).length();
			//}
		}

		bool hit(Point const& pos, glm::vec3& hitPoint, btRigidBody*& hitBody)
		{
			auto npos = m_window->normalizeCoords(pos);
			auto ray = m_camera->getRay(npos);
			auto& from = std::get<0>(ray);
			auto& to = std::get<1>(ray);
			to = from + (to - from) * 100;

			auto world = m_physics->getWorld();
			btCollisionWorld::ClosestRayResultCallback RayCallback(from, to);
			world->rayTest(from, to, RayCallback);
			if (RayCallback.hasHit()) {
				btVector3 End = RayCallback.m_hitPointWorld;
				btVector3 Normal = End + RayCallback.m_hitNormalWorld;
				btRigidBody *Body = btRigidBody::upcast(const_cast<btCollisionObject*>(RayCallback.m_collisionObject));

				hitPoint = glm::vec3(End.x(), End.y(), End.z());
				hitBody = Body;
				return true;
			}
			return false;
		}
	};

	class Timer
	{
	private:
		uint64_t freq;
		uint64_t prevTick;
		uint64_t firstTick;

	public:
		Timer()
		{
			freq = glfwGetTimerFrequency();
			prevTick = 0;
		}

		double stepTime()
		{
			if (prevTick == 0) {
				prevTick = glfwGetTimerValue();
				firstTick = prevTick;
				return 0;
			}

			auto currentTick = glfwGetTimerValue();
			double t = (double(currentTick) - prevTick) / freq;
			prevTick = currentTick;
			return t;
		}

		double totalTime()
		{
			return (double(prevTick) - firstTick) / freq;
		}
	};

	class GLViewWindow : public GLWindow
	{
	public:
		std::function<void()> renderFunction;
		Camera camera;
		CameraController cameraController;

		GLViewWindow()
			: cameraController(&camera)
		{
			addListener(&cameraController);
		}

		void run()
		{
			while (!isClosed()) {
				setupProjections();

				glClearColor(0.3f, 0.3f, 0.3f, 0);
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				glEnable(GL_DEPTH_TEST);
				drawGrid();
				drawAxes();

				if (renderFunction) renderFunction();

				swapBuffers();
				processEvents();
			}
		}

	private:
		void drawGrid()
		{
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
		}

		void drawAxes()
		{
			glBegin(GL_LINES);
			float size = 0.1f;
			glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(size, 0, 0);
			glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, size, 0);
			glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, size);
			glEnd();
		}

		void setupProjections() {
			camera.aspectRatio = (double)width() / height();
			camera.updateMatrices();
			glMatrixMode(GL_PROJECTION);
			glLoadMatrixf(glm::value_ptr(camera.projection));
			glMatrixMode(GL_MODELVIEW);
			glLoadMatrixf(glm::value_ptr(camera.modelView));
		};
	};
#pragma endregion
}
