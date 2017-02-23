#include "stdafx.h"
using namespace robotlib;
#include "program01.h"
#include "program02.h"
#include "program03.h"
#include "program04.h"

int main()
{
	if (!glfwInit()) throw std::exception("Unable to initialize glfw library.");

	int choice = 4;
	switch (choice) {
	case 1:
		return Program::main(); // Motor demo with two simple arms
	case 2:
		return program02(); // URDF loader
	case 3: 
		return program03();
	case 4:
		return Program04::main(); // Plate-on-stem ball balance mech-controller
	}
}

