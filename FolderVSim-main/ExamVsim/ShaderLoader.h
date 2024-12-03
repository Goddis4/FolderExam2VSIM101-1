#ifndef SHADER_LOADER_H
#define SHADER_LOADER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>

class ShaderLoader {
public:
	static std::string LoadShaderFromFile(const std::string& filePath);
};

#endif
