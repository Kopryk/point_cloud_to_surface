#include "graphics/GraphicsApplication.h"

int main() {

	auto& ga = GraphicsApplication::get();
	ga.mainLoop();
}
