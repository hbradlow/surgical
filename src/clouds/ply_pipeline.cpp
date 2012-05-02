#include "ply_pipeline.h"
#include "get_table.h"


//vtk stuff
#include <vtkSmartPointer.h>
#include <vtkGenericDataObjectReader.h>
#include <vtkCellArray.h>
#include <vtkCleanPolyData.h>
#include <vtkPolyData.h>
#include <vtkFloatArray.h>

//pcl stuff
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/apps/dominant_plane_segmentation.h>

#include "clouds/plane_finding.h"

//bullet
#include "simulation/config_bullet.h"

#include <vector>
#include <fstream>

using namespace std;

void process_raw(char* filename){
	string line = filename; 
	vtkSmartPointer<vtkGenericDataObjectReader> reader = vtkSmartPointer<vtkGenericDataObjectReader>::New();
	reader->SetFileName(line.c_str());
	reader->Update();


	vtkPoints* point_array;
	vtkCellArray* cell_array;
	vtkPointSet* output;

	vtkSmartPointer<vtkCleanPolyData> cleaner =
	vtkSmartPointer<vtkCleanPolyData>::New();
	cleaner->SetInputConnection (reader->GetOutputPort());
	cleaner->Update();
	output = cleaner->GetOutput();

	//read the points and cells from the corresponding file format
	if(reader->IsFilePolyData()){
		point_array = ((vtkPolyData*)cleaner->GetOutput())->GetPoints();
		cell_array = ((vtkPolyData*)cleaner->GetOutput())->GetPolys();
	}
	else{
		cout << "Not poly data" << endl;
		point_array = ((vtkPolyData*)cleaner->GetOutput())->GetPoints();
		cell_array = ((vtkPolyData*)cleaner->GetOutput())->GetPolys();
		//point_array = cleaner->GetUnstructuredGridOutput()->GetPoints();
		//cell_array = cleaner->GetUnstructuredGridOutput()->GetCells();
	}
	cout << "Size of set: " << point_array->GetNumberOfPoints() << endl;

	//filter out only the data from the cell array
	float *ft = ((vtkFloatArray*)point_array->GetData())->GetPointer(0);
	int *tt = (int*)((vtkIdTypeArray*)cell_array->GetPointer());
	int *t = (int*)malloc((((cell_array->GetSize()))/4)*3*sizeof(int));
	for(int i = 0; i<((cell_array->GetSize()))/4; i++)
	{
		t[i*3+0] = *(tt+i*8+2);
		t[i*3+1] = *(tt+i*8+4);
		t[i*3+2] = *(tt+i*8+6);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb (new pcl::PointCloud<pcl::PointXYZRGB>(30,30));
	for(int i = 0; i<point_array->GetNumberOfPoints(); i++){
		pcl::PointXYZRGB p;
		p.x = ft[i*3];
		p.y = ft[i*3+1];
		p.z = ft[i*3+2];
		p.r = 255; 
		p.g = 0;
		p.b = 0;
		cloudrgb->points.push_back(p);
	}
	for(int iter = 0; iter<3; iter++){
		pcl::apps::DominantPlaneSegmentation<pcl::PointXYZRGB> dps;
		dps.setInputCloud(cloudrgb);
		vector<ColorCloudPtr> clusters;
		dps.compute_table_plane();
		Eigen::Vector4f coeff;
		dps.getTableCoefficients(coeff);
		cout << "COEFF: " << coeff << endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>(30,30));
		for(int i = 0; i<cloudrgb->points.size(); i++){
			pcl::PointXYZRGB p;
			p.x = cloudrgb->points[i].x;
			p.y = cloudrgb->points[i].y;
			p.z = cloudrgb->points[i].z;
			p.r = 255; 
			p.g = 255;
			p.b = 255;
			if(abs(coeff.x()*p.x+coeff.y()*p.y+coeff.z()*p.z+coeff.w())>.1f)
			{
				tmp->points.push_back(p);
			}
		}
		cloudrgb = tmp;
	}
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	   viewer.showCloud (cloudrgb);
	      while (!viewer.wasStopped ())
			     {
					    }


	vector<Eigen::Vector3f> corners;
	Eigen::Vector3f normal;
	getTable(cloudrgb,corners,normal,0);

	Eigen::Vector3f c0 = corners[1]-corners[0];
	Eigen::Vector3f c1 = corners[3]-corners[0];




	//remove duplicate verticies
	int mVertexCount = (int)((point_array->GetNumberOfPoints())/1.0);
	int dupVertices[mVertexCount];
	int dupVerticesCount = 0;
	int i,j;
	int newIndexes[mVertexCount];
	for(i=0; i < mVertexCount; i++)
	{
	 btVector3 v1 =  btVector3(ft[i*3],ft[i*3+1],ft[i*3+2]);
	 dupVertices[i] = -1;
	 newIndexes[i] = i - dupVerticesCount;

		Eigen::Vector3f p0 = Eigen::Vector3f(ft[i*3],ft[i*3+1],ft[i*3+2])-corners[0];

		if(p0.dot(normal)<.01 || !(p0.dot(c0)>0 
			&& p0.dot(c1)>0 && p0.dot(c0)<c0.dot(c0) 
			&& p0.dot(c1)<c1.dot(c1)))
		{
			dupVertices[i] = -2;
			dupVerticesCount++;
		}

		/*
		 for(j=0; j < i; j++)
		 {
			btVector3 v2 =  btVector3(ft[j*3],ft[j*3+1],ft[j*3+2]);;

			Eigen::Vector3f p0 = Eigen::Vector3f(ft[i*3],ft[i*3+1],ft[i*3+2])-corners[0];

			if(p0.dot(normal)<.06 || !(p0.dot(c0)>0 && p0.dot(c1)>0 && p0.dot(c0)<c0.dot(c0) && p0.dot(c1)<c1.dot(c1)))
			{
				dupVertices[i] = -2;
				dupVerticesCount++;
				break;
			}
			if (v1 == v2) {
			   dupVertices[i] = j;
			   dupVerticesCount++;
			   break;
			}
		 }*/
	}
	printf("dupVerticesCount %d\n", dupVerticesCount);

	int newVertexCount = mVertexCount - dupVerticesCount;
	printf("newVertexCount %d\n", newVertexCount);
	btScalar *vertices = (btScalar*)malloc(newVertexCount * 3 *sizeof(btScalar));
	for(i=0, j=0; i < mVertexCount; i++)
	{
	 if (dupVertices[i] == -1) {
		btVector3 v =  btVector3(ft[i*3],ft[i*3+1],ft[i*3+2]);;
		vertices[j++] = v.getX();
		vertices[j++] = v.getY();
		vertices[j++] = v.getZ();
	 }
	}

	int mIndexCount  = ((cell_array->GetSize()))*3.0/4.0;
	int *indexes = new int[mIndexCount];
	int maxIdx = -1;
	int idx, idxDup;
	int idx1, idxDup1;
	int idx2, idxDup2;
	for(i=0; i < mIndexCount; i+=3)
	{
	 idx = t[i];
	 idx1 = t[i+1];
	 idx2 = t[i+2];
	 idxDup = dupVertices[idx];
	 idxDup1 = dupVertices[idx1];
	 idxDup2 = dupVertices[idx2];
		if(idxDup == -2 || idxDup1 == -2 || idxDup2 == -2)
		{
			indexes[i] = 0;
			indexes[i+1] = 0;
			indexes[i+2] = 0;
		}
		else{
			idx = idxDup == -1 ? idx : idxDup;
			 indexes[i] = newIndexes[idx];
			idx1= idxDup1== -1 ? idx1: idxDup1;
			 indexes[i+1] = newIndexes[idx1];
			idx2= idxDup2== -1 ? idx2: idxDup2;
			 indexes[i+2] = newIndexes[idx2];
		}
			 if(indexes[i]>maxIdx)
				 maxIdx = indexes[i];
			 if(indexes[i+1]>maxIdx)
				 maxIdx = indexes[i+1];
			 if(indexes[i+2]>maxIdx)
				 maxIdx = indexes[i+2];
	}
	int ntriangles = mIndexCount / 3;


	//Transform the points onto the table
	Eigen::Vector3f middle = (corners[0] + corners[1] + corners[2] + corners[3])/4.0;

	Eigen::Vector3f oldSide = (corners[1]-corners[0]);
	Eigen::Vector3f side(0,1,0);

	Eigen::Vector3f center(24,0,11.2); //This is the center of the table in bullet, change this to match the correct center

	Eigen::Vector3f oldUp((corners[1]-corners[0]).cross(corners[3]-corners[0]));
	Eigen::Vector3f up(0,0,1); //This is the direction of the table in bullet, change this to match the correct orientation of the table float scale = 35;

	float scale = 30;

	for(int i = 0; i<=maxIdx; i++){
		Eigen::Vector3f v(vertices[i*3],vertices[i*3+1],vertices[i*3+2]);

		v -= middle;
		Eigen::AngleAxis<float> aa(acos(oldUp.dot(up)/(oldUp.norm()*up.norm())),oldUp.cross(up));
		v *= scale;
		v = aa*v;
		v += center;

		vertices[i*3] = v.x(); 
		vertices[i*3+1] = v.y(); 
		vertices[i*3+2] = v.z(); 
	}


	writePly("out_raw.ply",vertices,indexes,maxIdx+1,ntriangles);
}

void writePly(char *filename, float* vertices,int* cells,int numVertices,int numCells){
	ofstream fout;
	fout.open(filename);
	fout << "ply" << "\n";
	fout << "format ascii 1.0" << "\n";
	fout << "element vertex " << numVertices << "\n";
	fout << "property float x" << "\n";
	fout << "property float y" << "\n";
	fout << "property float z" << "\n";
	fout << "element face " << numCells << "\n";
	fout << "property list uchar int vertex_index" << "\n";
	fout << "end_header" << "\n";
	for(int i = 0; i<numVertices; i++){
		fout << vertices[i*3] << " " << vertices[i*3+1] << " " << vertices[i*3+2] << "\n";
	}
	for(int i = 0; i<numCells; i++){
		fout << "3 " << cells[i*3] << " " << cells[i*3+1] << " " << cells[i*3+2] << "\n";
	}
	fout << flush;
	fout.close();
}
