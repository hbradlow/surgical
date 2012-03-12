#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include "simulation/config_viewer.h"
#include <openrave/kinbody.h>
#include "robots/pr2.h"


#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkParticleReader.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkGenericDataObjectReader.h>
#include <vtkStructuredGrid.h>
#include <vtkFloatArray.h>
#include <vtkUnstructuredGrid.h>
#include <vtkRectilinearGrid.h>
#include <vtkStructuredPoints.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>

BulletSoftObject::Ptr createMesh(btSoftBodyWorldInfo& worldInfo,string filename,const btVector3 &center) {
/*	cout << "Center " << (center.getX()) << endl;
	cout << "Center " << (center.getY()) << endl;
	cout << "Center " << (center.getZ()) << endl;
	string line = "/home/henrybrad/Desktop/testply2.vtk";
	vtkSmartPointer<vtkGenericDataObjectReader> reader = 
	vtkSmartPointer<vtkGenericDataObjectReader>::New();
	reader->SetFileName(line.c_str());
	reader->Update();

	btTriangleMesh *mTriMesh = new btTriangleMesh();

	if(reader->IsFilePolyData()){
		vtkSmartPointer<vtkPolyData> poly = reader->GetPolyDataOutput();
	
		//poly->Print(cout);
		cout << poly->GetNumberOfPoints() << endl;
	}
		cout <<"Not poly data"<<endl;
		reader->GetUnstructuredGridOutput()->GetPoints()->Print(cout);
		reader->GetUnstructuredGridOutput()->GetCells()->Print(cout);
		cout << "Point 4: " << *((vtkFloatArray*)reader->GetUnstructuredGridOutput()->GetPoints()->GetData())->GetPointer(3) << endl;
		cout << "Faces: " << *((vtkIdTypeArray*)reader->GetUnstructuredGridOutput()->GetCells()->GetData())->GetPointer(0) << endl;
		float *ft = ((vtkFloatArray*)reader->GetUnstructuredGridOutput()->GetPoints()->GetData())->GetPointer(0);
		int *tt = (int*)((vtkIdTypeArray*)reader->GetUnstructuredGridOutput()->GetCells()->GetPointer());
		int t[((reader->GetUnstructuredGridOutput()->GetCells()->GetSize()))/4][3];
		vector<btVector3> points;
		for(int i = 0; i<((reader->GetUnstructuredGridOutput()->GetCells()->GetSize()))/4; i++)
		{
			t[i][0] = *(tt+i*8+2);
			t[i][1] = *(tt+i*8+4);
			t[i][2] = *(tt+i*8+6);
			cout << t[i][0] << " "<< t[i][1] << " "<< t[i][2]<<endl;

			btVector3 v0(*(ft+t[i][0]),*(ft+t[i][0]+1),*(ft+t[i][0]+2));
			btVector3 v1(*(ft+t[i][1]),*(ft+t[i][1]+1),*(ft+t[i][1]+2));
			btVector3 v2(*(ft+t[i][2]),*(ft+t[i][2]+1),*(ft+t[i][2]+2));
    			// Then add the triangle to the mesh:
			points.push_back(v0);
			points.push_back(v1);
			points.push_back(v2);
    			mTriMesh->addTriangle(v0,v1,v2);
		}
		for(int i = 0; i<(int)((reader->GetUnstructuredGridOutput()->GetPoints()->GetNumberOfPoints())/1.0); i++)
		{
			*(ft+i*3) *= 20;
			*(ft+i*3+1) *= 20;
			*(ft+i*3+2) *= 40;
			*(ft+i*3) += -15;
			*(ft+i*3+1) += -20;
			*(ft+i*3+2) += 5;
			//*(ft+i*3) += .5*(center.getX());
			//*(ft+i*3+1) += .5*(center.getY());
			//*(ft+i*3+2) += .5*(center.getX());
		}
			
		cout << ((reader->GetUnstructuredGridOutput()->GetCells()->GetPointer())) << endl;
*/
//		btSoftBody *psb = btSoftBodyHelpers::CreateFromTriMesh(worldInfo,(btScalar*)((vtkFloatArray*)reader->GetUnstructuredGridOutput()->GetPoints()->GetData())->GetPointer(0),
//				&t[0][0]/*(int*)((vtkIdTypeArray*)reader->GetUnstructuredGridOutput()->GetCells()->GetData())->GetPointer(0)*/,((reader->GetUnstructuredGridOutput()->GetCells()->GetSize()))/4/*(int)(((reader->GetUnstructuredGridOutput()->GetCells()->GetSize()))/3.0)*/,true);
	/*

		float p[] = {0,0,0,
				0,1,0,
				1,0,0,
				0,1,0,
				1,0,0,
				1,1,0};
		int t[][3] = {{0,1,2},{3,4,5}};

		for(int i = 0; i<6; i++)
		{
			p[i*3] += center.getX();
			p[i*3+1] += center.getX();
			p[i*3+2] += center.getX();
		}
		btSoftBody *psb = btSoftBodyHelpers::CreateFromTriMesh(env->bullet->softBodyWorldInfo,&p[0],&t[0][0],2);
*/

//		cout << "Done making the soft body" << endl;

//		btCollisionShape *mTriMeshShape = new btBvhTriangleMeshShape(mTriMesh,true);
		//return btRigidBody::btRigidBody(10,NULL,mTriMeshShape);
		btSoftBody *psb = btSoftBodyHelpers::CreateFromTetGenData(
        worldInfo,"~/Downloads/socket.1.ele","~/Downloads/socket.1.face","~/Downloads/socket.1.node",true,true,true);

    psb->m_cfg.piterations = 2;
    psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS
        | btSoftBody::fCollision::CL_RS
        | btSoftBody::fCollision::CL_SELF;
    //    | btSoftBody::fCollision::SDF_RS;
    psb->m_cfg.kDF = 1.0;
    psb->getCollisionShape()->setMargin(0.4);
    btSoftBody::Material *pm = psb->appendMaterial();
    pm->m_kLST = 0.1;
    psb->generateBendingConstraints(2, pm);
    psb->randomizeConstraints();
    psb->setTotalMass(150);

    		return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}
void gen(Scene &scene, btScalar s, btScalar z) {
	btSoftBody*		psb=btSoftBodyHelpers::CreatePatch(scene.env->bullet->softBodyWorldInfo,
        btVector3(-s,-s,z),
		btVector3(+s,-s,z),
		btVector3(-s,+s,z),
		btVector3(+s,+s,z),
		31, 31,
		0/*1+2+4+8*/, true);

	psb->getCollisionShape()->setMargin(0.4);
	btSoftBody::Material* pm=psb->appendMaterial();
	pm->m_kLST		=	0.4;
//	pm->m_flags		-=	btSoftBody::fMaterial::DebugDraw;
	psb->generateBendingConstraints(2, pm);
	psb->setTotalMass(150);
    //scene.env->add(BulletSoftObject::Ptr(new BulletSoftObject(psb)));
    scene.env->add(createMesh(scene.env->bullet->softBodyWorldInfo,"test",btVector3(+s,-s,z)));
}

int main(int argc, char *argv[]) {
    GeneralConfig::scale = 10.;    

    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.read(argc, argv);

    Scene scene;
//    gen(scene, 1, 0.1);
    gen(scene, 5, 30);

    scene.startViewer();
    scene.startLoop();
    return 0;
}
