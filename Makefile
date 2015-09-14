poly2tri.dylib: src/binding.cpp src/binding.hpp
	clang++ -framework OpenGL -framework Cocoa -framework IOKit -O3 -ffast-math \
		-DP2T_BUILD_SHARED -Ivendor/poly2tri-cpp/poly2tri -dynamiclib \
		-fvisibility=hidden \
		vendor/poly2tri-cpp/poly2tri/*/*.cc src/binding.cpp -opoly2tri.dylib
