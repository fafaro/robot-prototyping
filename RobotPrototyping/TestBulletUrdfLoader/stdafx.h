#pragma once

#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <map>
#include <list>
#include <memory>
#include <functional>
#include <fstream>

#include <boost/optional/optional.hpp>
#include <boost/any.hpp>

#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>

#include <tinyxml2.h>

#include "btBulletDynamicsCommon.h"

#include "../../bullet3/examples/Importers/ImportURDFDemo/UrdfParser.h"
#include "../../bullet3/examples/Importers/ImportURDFDemo/URDF2Bullet.h"

//#include "../../bullet3/src/LinearMath\btIDebugDraw.h"
#include "../../bullet3/examples/CommonInterfaces/CommonParameterInterface.h"
#include "../../bullet3/examples/CommonInterfaces/CommonGUIHelperInterface.h"
#include "../../bullet3/examples/CommonInterfaces/CommonMultiBodyBase.h"
#include "../../bullet3/examples/CommonInterfaces/CommonExampleInterface.h"
#include "../../bullet3/examples/Importers/ImportURDFDemo/ImportURDFSetup.h"

