#include "graphics/GraphicsApplication.h"

#include <iostream>

int main(int argc, char* argv[]) {


	if (argc != 2) {
		std::cout << "Provide path to shader directory\n";
		return -1;
	}

	auto& ga = GraphicsApplication::get();
	ga.shaderPath = argv[1];
	ga.mainLoop();
}
