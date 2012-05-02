#include "clouds/ply_pipeline.h"

#include <iostream>

int main(int argc, char *argv[]){
	if(argc>1)
	{
		process_raw(argv[1]);
	}
	else
		process_raw("/home/henrybrad/Desktop/kinfu_tests/test11/mesh.vtk");
}
