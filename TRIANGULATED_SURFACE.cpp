#include "TRIANGULATED_SURFACE.h"
#include "INTERSECTION.h"
#include <algorithm>

using namespace SimLib;

template<class T>
bool TRIANGULATED_SURFACE<T>::loadOBJ(char * path){
	printf("Loading OBJ file %s...\n", path);

	std::vector<unsigned int> vertexIndices;
	std::vector<VECTOR<T,3> > temp_vertices; 


	FILE * file = fopen(path, "r");
	if( file == NULL ){
		printf("Impossible to open the file ! Are you in the right path ? See Tutorial 1 for details\n");
		getchar();
		return false;
	}

	while( 1 ){

		char lineHeader[128];
		// read the first word of the line
		int res = fscanf(file, "%s", lineHeader);
		if (res == EOF)
			break; // EOF = End Of File. Quit the loop.

		// else : parse lineHeader
		
		if ( strcmp( lineHeader, "v" ) == 0 ){
			VECTOR<T, 3> vertex;
			fscanf(file, "%f %f %f\n", &vertex(1), &vertex(2), &vertex(3) );
			temp_vertices.push_back(vertex);
		}else if ( strcmp( lineHeader, "f" ) == 0 ){
			unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
			int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &uvIndex[0], &normalIndex[0], &vertexIndex[1], &uvIndex[1], &normalIndex[1], &vertexIndex[2], &uvIndex[2], &normalIndex[2] );
			if (matches != 9){
				printf("File can't be read by our simple parser :-( Try exporting with other options\n");
				return false;
			}
			vertexIndices.push_back(vertexIndex[0]);
			vertexIndices.push_back(vertexIndex[1]);
			vertexIndices.push_back(vertexIndex[2]);
		}else{
			// Probably a comment, eat up the rest of the line
			char stupidBuffer[1000];
			fgets(stupidBuffer, 1000, file);
		}

	}

	// For each vertex of each triangle
	for( unsigned int i=0; i<vertexIndices.size(); i += 3){

		triangle_list.push_back(TRIANGLE<T>(temp_vertices[vertexIndices[i]-1],temp_vertices[vertexIndices[i+1]-1],temp_vertices[vertexIndices[i+2]-1]));
	}

	return true;
}

template<class T>
void TRIANGULATED_SURFACE<T>::Update_Bounding_Box() {
	if (triangle_list.size() == 0) {
		bounding_box = RANGE<VECTOR<T, 3> >();
		return;
	}
	bounding_box = triangle_list[0].Bounding_Box();
	for (int i = 1; i < triangle_list.size(); ++i)
		bounding_box.include(triangle_list[i].Bounding_Box());
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