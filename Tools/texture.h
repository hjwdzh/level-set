#ifndef TEXTURE_HPP
#define TEXTURE_HPP

#include <map>
#include <vector>
#include <string>
using namespace std;

// Load a .BMP file using our custom loader
GLuint loadBMP_custom(const char * imagepath);

//// Since GLFW 3, glfwLoadTexture2D() has been removed. You have to use another texture loading library, 
//// or do it yourself (just like loadBMP_custom and loadDDS)
//// Load a .TGA file using GLFW's own loader
//GLuint loadTGA_glfw(const char * imagepath);

// Load a .DDS file using GLFW's own loader

class TexManager {
public:
	static map<string,GLuint> nameManager;
	static vector<GLuint> texResource;
	static GLuint createRenderTexture(const char* name) {
		string n = string(name);
		map<string,GLuint>::iterator it = nameManager.find(n);
		GLuint texid;
		if (it == nameManager.end()) {
			texid = loadBMP_custom(name);
			texResource.push_back(texid);
			texid = (int)texResource.size() - 1;
			nameManager.insert(pair<string,GLuint>(n, texid));
		} else {
			texid = it->second;
		}
		return texResource[texid];
	}
};

#endif