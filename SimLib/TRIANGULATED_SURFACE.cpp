#include "TRIANGULATED_SURFACE.h"
#include "INTERSECTION.h"
#include <algorithm>
#include <iostream>

using namespace SimLib;
using namespace std;

template<class T>
bool TRIANGULATED_SURFACE<T>::loadOBJ(const char * path){
	printf("Loading OBJ file %s...\n", path);

	std::vector<unsigned int> vertexIndices;
	std::vector<unsigned int> normalIndices;
	std::vector<unsigned int> texIndices;
	std::vector<VECTOR<T,3> > temp_normal;
	std::vector<VECTOR<T,2> > temp_tex;

	vertices.clear();

	FILE * file;
#ifdef _WINDOWS_PLATFORM_
	fopen_s(&file, path, "r");
#else
    file = fopen(path, "r");
#endif
	if( file == NULL ){
		printf("Impossible to open the file ! Are you in the right path ? See Tutorial 1 for details\n");
		getchar();
		return false;
	}

	while( 1 ){

		char lineHeader[128];
		// read the first word of the line
#ifdef _WINDOWS_PLATFORM_
		int res = fscanf_s(file, "%s", lineHeader);
#else
        int res = fscanf(file, "%s", lineHeader);
#endif
		if (res == EOF)
			break; // EOF = End Of File. Quit the loop.

		// else : parse lineHeader
		
		if ( strcmp( lineHeader, "v" ) == 0 ){
			VECTOR<T, 3> vertex;
#ifdef _WINDOWS_PLATFORM_
			fscanf_s(file, "%f %f %f\n", &vertex(1), &vertex(2), &vertex(3) );
#else
			fscanf(file, "%f %f %f\n", &vertex(1), &vertex(2), &vertex(3) );
#endif
			vertices.push_back(vertex);
		}else if (strcmp(lineHeader, "vn") == 0) {
            VECTOR<T, 3> normal;
#ifdef _WINDOWS_PLATFORM_
			fscanf_s(file, "%f %f %f\n", &normal(1), &normal(2), &normal(3) );
#else
			fscanf(file, "%f %f %f\n", &normal(1), &normal(2), &normal(3) );
#endif
			temp_normal.push_back(normal);
        }else if (strcmp(lineHeader, "vt") == 0) {
            VECTOR<T, 2> tex;
#ifdef _WINDOWS_PLATFORM_
            fscanf_s(file, "%f %f\n", &tex(1), &tex(2));
#else
            fscanf(file, "%f %f\n", &tex(1), &tex(2));
#endif
            temp_tex.push_back(tex);
        }
        else if( strcmp( lineHeader, "f" ) == 0 ){
			unsigned int vertexIndex[3], normalIndex[3], texIndex[3];
#ifdef _WINDOWS_PLATFORM_
			int matches = fscanf_s(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &texIndex[0], &normalIndex[0], &vertexIndex[1], &texIndex[1], &normalIndex[1], &vertexIndex[2], &texIndex[2], &normalIndex[2] );
#else
			int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &texIndex[0], &normalIndex[0], &vertexIndex[1], &texIndex[1], &normalIndex[1], &vertexIndex[2], &texIndex[2], &normalIndex[2] );
#endif
			if (matches != 9){
				printf("File can't be read by our simple parser :-( Try exporting with other options\n");
				return false;
			}
			vertexIndices.push_back(vertexIndex[0]);
			vertexIndices.push_back(vertexIndex[1]);
			vertexIndices.push_back(vertexIndex[2]);
            normalIndices.push_back(normalIndex[0]);
            normalIndices.push_back(normalIndex[1]);
            normalIndices.push_back(normalIndex[2]);
            texIndices.push_back(texIndex[0]);
            texIndices.push_back(texIndex[1]);
            texIndices.push_back(texIndex[2]);
		}else{
			// Probably a comment, eat up the rest of the line
			char stupidBuffer[1000];
			fgets(stupidBuffer, 1000, file);
		}

	}

	// For each vertex of each triangle
	for( unsigned int i=0; i<vertexIndices.size(); i += 3){
		triangle_list.push_back(TRIANGLE<T>(vertices[vertexIndices[i]-1],vertices[vertexIndices[i+1]-1],vertices[vertexIndices[i+2]-1],temp_normal[normalIndices[i]-1],temp_normal[normalIndices[i+1]-1],temp_normal[normalIndices[i+2]-1],temp_tex[texIndices[i]-1],temp_tex[texIndices[i+1]-1],temp_tex[texIndices[i+2]-1]));
	}

	return true;
}

template<class T>
Vector3d TRIANGULATED_SURFACE<T>::Update_Bounding_Box_And_Gravity_Center() {
	if (triangle_list.size() == 0) {
		bounding_box = RANGE<VECTOR<T, 3> >();
		return Vector3d();
	}
    gravity_center = TV();
    for (int i = 0; i < vertices.size(); ++i)
        gravity_center += vertices[i];
    gravity_center *= (1.0 / vertices.size());
    for (int i = 0; i < vertices.size(); ++i)
        vertices[i] -= gravity_center;
    for (int i = 0; i < triangle_list.size(); ++i) {
        triangle_list[i].a -= gravity_center;
        triangle_list[i].b -= gravity_center;
        triangle_list[i].c -= gravity_center;
    }
	bounding_box = triangle_list[0].Bounding_Box();
	for (int i = 1; i < triangle_list.size(); ++i) {
		bounding_box.include(triangle_list[i].Bounding_Box());
    }
    Vector3d ret(gravity_center(1),gravity_center(2),gravity_center(3));
    gravity_center = TV();
    return ret;
}

template<class T>
bool TRIANGULATED_SURFACE<T>::Test_Inside_Using_Ray(RAY<T>& ray) {
    std::vector<T> intersects;
    for (int i = 0; i < triangle_list.size(); ++i) {
        if (INTERSECTION<T>::Intersects(ray, triangle_list[i])) {
            intersects.push_back(ray.intersect);
        }
    }
    std::sort(intersects.begin(), intersects.end());
    int t = 1, i = 1;
    for (; i < intersects.size(); ++i) {
        if (intersects[i] - intersects[i - 1] > 1e-4) {
            ++t;
        }
    }
    return (t % 2 == 1);
}

template<class T>
bool TRIANGULATED_SURFACE<T>::Inside(const TV& point) {
    if (!bounding_box.Contain(point))
        return false;
    RAY<T> ray(point,TV(0,0,1),true);
    return Test_Inside_Using_Ray(ray);
}



template class SimLib::TRIANGULATED_SURFACE<float>;