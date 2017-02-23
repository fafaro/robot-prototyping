#include "stdafx.h"
#include "../RobotLib/robotlib.h"

std::string readFile(std::ifstream& in) {
	std::stringstream sstr;
	sstr << in.rdbuf();
	return sstr.str();
}

class MyLogger : public ErrorLogger
{
	virtual void reportError(const char* error) {}
	virtual void reportWarning(const char* warning) {}
	virtual void printMessage(const char* msg) {}
};

class MyDebugDrawer : public btIDebugDraw
{
public:
	virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) 
	{
	}

	virtual void	drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color) {}

	virtual void	reportErrorWarning(const char* warningString) {}

	virtual void	draw3dText(const btVector3& location, const char* textString) {}

	virtual void	setDebugMode(int debugMode) {}

	virtual int		getDebugMode() const { return 0; }
};

class MyParameterInterface : public CommonParameterInterface
{
public:
	virtual ~MyParameterInterface() {}
	virtual void registerSliderFloatParameter(SliderParams& params) {}
	virtual void registerButtonParameter(ButtonParams& params) {}
	virtual void registerComboBox(ComboBoxParams& params) {}

	virtual void syncParameters() {}
	virtual void removeAllParameters() {}
	virtual void setSliderValue(int sliderIndex, double sliderValue) {}
};

class MyGUIHelper : public GUIHelperInterface
{
public:
	MyParameterInterface m_paramInterface;

	MyGUIHelper() {}
	virtual ~MyGUIHelper() {}

	virtual void createRigidBodyGraphicsObject(btRigidBody* body, const btVector3& color) {}

	virtual void createCollisionObjectGraphicsObject(btCollisionObject* obj, const btVector3& color) {}

	virtual void createCollisionShapeGraphicsObject(btCollisionShape* collisionShape) {}

	virtual void syncPhysicsToGraphics(const btDiscreteDynamicsWorld* rbWorld) {}

	virtual void render(const btDiscreteDynamicsWorld* rbWorld) {}

	virtual void createPhysicsDebugDrawer(btDiscreteDynamicsWorld* rbWorld) {
		auto debugDraw = new MyDebugDrawer();
		rbWorld->setDebugDrawer(debugDraw);
	}

	virtual int	registerTexture(const unsigned char* texels, int width, int height) { return 0; }
	virtual int registerGraphicsShape(const float* vertices, int numvertices, const int* indices, int numIndices, int primitiveType, int textureId) { return 0; }
	virtual int registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling) { return 0; }
	virtual void removeAllGraphicsInstances() {}

	virtual Common2dCanvasInterface* get2dCanvasInterface() { return nullptr; }

	virtual CommonParameterInterface* getParameterInterface() { return &m_paramInterface; }

	virtual CommonRenderInterface* getRenderInterface() { return nullptr; }

	virtual CommonGraphicsApp* getAppInterface() {
		return nullptr;
	}

	virtual void setUpAxis(int axis) {}

	virtual void resetCamera(float camDist, float pitch, float yaw, float camPosX, float camPosY, float camPosZ) {}

	virtual void copyCameraImageData(const float viewMatrix[16], const float projectionMatrix[16],
		unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels,
		float* depthBuffer, int depthBufferSizeInPixels,
		int* segmentationMaskBuffer, int segmentationMaskBufferSizeInPixels,
		int startPixelIndex, int destinationWidth, int destinationHeight, int* numPixelsCopied) {}

	virtual void autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld) {}

	virtual void drawText3D(const char* txt, float posX, float posZY, float posZ, float size) {}
};

int main()
{
	MyGUIHelper guiHelper;
	CommonExampleOptions options(&guiHelper);
	//options.m_fileName = "../../data/cyton_gamma_1500/cyton_gamma_1500.urdf";
	options.m_fileName = "../../data/r2d2.urdf";
	auto example = ImportURDFCreateFunc(options);
	example->initPhysics();

	auto dynWorld = (static_cast<CommonMultiBodyBase*>(example))->m_dynamicsWorld;
	dynWorld->setGravity(btVector3(0, 0, -10));

	bool pauseSimulation = true;
	robotlib::init();
	robotlib::Physics physics(dynWorld);
	robotlib::GLViewWindow view;
	robotlib::Timer timer;
	robotlib::PickObjectController pickCtrl(&view.camera, &physics, &view);
	view.addListener(&pickCtrl);
	view.addKeyCallback('p', [&]() { pauseSimulation = !pauseSimulation; });
	view.renderFunction = [&]() {
		if (!pauseSimulation) physics.simulate(timer.stepTime());
		physics.debugDraw();
	};

	robotlib::Physics::CRB::RigidBody rb;
	rb.shape = robotlib::Physics::CRB::Shapes::Box(btVector3(1, 1, 1));
	rb.mass = 0;
	rb.setPosition(btVector3(0, 0, -1));
	physics.create(rb);

	view.run();

	return 0;
}

void parseUrdf() 
{
	MyLogger loggie;
	UrdfParser parser;
	bool forceFixedBase = true;

	const char *path = "../../data/cyton_gamma_1500/cyton_gamma_1500.urdf";
	std::string xml_string = readFile(std::ifstream(path));

	parser.setParseSDF(false);
	bool result = parser.loadUrdf(xml_string.c_str(), &loggie, forceFixedBase);
	std::cout << (result ? "All good!" : "Whoops!") << std::endl;
	//std::cin.get();
}