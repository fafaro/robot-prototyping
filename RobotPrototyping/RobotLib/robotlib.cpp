#include "stdafx.h"
#include "robotlib.h"

namespace robotlib
{
	void init()
	{
		if (!glfwInit()) throw std::exception("Unable to initialize glfw library.");
	}

	void render(std::function<void()> r)
	{
		GLViewWindow window;
		window.renderFunction = r;
		window.run();
	}


	glm::vec3 GLUtil::color = glm::vec3(1, 1, 1);
}