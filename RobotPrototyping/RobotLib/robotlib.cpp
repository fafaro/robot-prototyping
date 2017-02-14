#include "stdafx.h"
#include "robotlib.h"

namespace robotlib
{
	void init()
	{
		if (!glfwInit()) throw std::exception("Unable to initialize glfw library.");
	}

	glm::vec3 GLUtil::color = glm::vec3(1, 1, 1);
}