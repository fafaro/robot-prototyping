#pragma once

#pragma warning(disable: 4305 4244)  // float conversions

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
#include <boost/circular_buffer.hpp>

#include "btBulletDynamicsCommon.h"

#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>

#include <tinyxml2.h>

#include "../RobotLib/robotlib.h"
