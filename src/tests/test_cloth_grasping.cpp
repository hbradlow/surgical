#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <openrave/kinbody.h>
#include "robots/pr2.h"

#include "perception/make_bodies.h"

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
#include <string>
#include <vector>

#include <vtkCellArray.h>
 
#include <sstream>

#include <Wm5Core.h>
#include <Wm5BSplineCurveFit.h>

#include "simulation/rope.h"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

#include "clouds/get_table.h"

BulletSoftObject::Ptr makeTowel(const vector<btVector3>& points, btSoftBodyWorldInfo& worldInfo) {
  btVector3 offset(0,0,.01*METERS);
  btSoftBody* psb=btSoftBodyHelpers::CreatePatch(worldInfo,
                                                 points[0]+offset,
                                                 points[1]+offset,
                                                 points[3]+offset,
                                                 points[2]+offset,
                                                 45, 31,
                                                 0/*1+2+4+8*/, true);
  cout << "points[0] " << points[0].x() << " " << points[0].y() << " " << points[0].z() << endl;
  psb->getCollisionShape()->setMargin(.01*METERS);
  btSoftBody::Material* pm=psb->appendMaterial();
  pm->m_kLST            =       5*0.1;
  pm->m_kAST = 5*0.1;
  //    pm->m_flags             -=      btSoftBody::fMaterial::DebugDraw;
  psb->generateBendingConstraints(2, pm);
  psb->setTotalMass(1);

  return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}
// I've only tested this on the PR2 model
class PR2SoftBodyGripperAction : public Action {
    RaveRobotKinematicObject::Manipulator::Ptr manip;
    dReal startVal, endVal;
    vector<int> indices;
    vector<dReal> vals;

    // min/max gripper dof vals
    static const float CLOSED_VAL = 0.03f, OPEN_VAL = 0.54f;

    KinBody::LinkPtr leftFinger, rightFinger;
    const btTransform origLeftFingerInvTrans, origRightFingerInvTrans;

    // the point right where the fingers meet when the gripper is closed
    // (in the robot's initial pose)
    const btVector3 centerPt;

    // vector normal to the direction that the gripper fingers move in the manipulator frame
    // (on the PR2 this points back into the arm)
    const btVector3 closingNormal;

    // points straight down in the PR2 initial position (manipulator frame)
    const btVector3 toolDirection;

    // the target softbody
    btSoftBody *psb;

    btTransform getManipRot() const {
        btTransform trans(manip->getTransform());
        trans.setOrigin(btVector3(0, 0, 0));
        return trans;
    }

    // Returns the direction that the specified finger will move when closing
    // (manipulator frame)
    btVector3 getClosingDirection(bool left) const {
        return (left ? 1 : -1) * toolDirection.cross(closingNormal);
    }

    // Finds some innermost point on the gripper
    btVector3 getInnerPt(bool left) const {
        btTransform trans(manip->robot->getLinkTransform(left ? leftFinger : rightFinger));
        // this assumes that the gripper is symmetric when it is closed
        // we get an innermost point on the gripper by transforming a point
        // on the center of the gripper when it is closed
        const btTransform &origInv = left ? origLeftFingerInvTrans : origRightFingerInvTrans;
        return trans * origInv * centerPt;
        // actually above, we can just cache origInv * centerPt
    }

    // Returns true is pt is on the inner side of the specified finger of the gripper
    bool onInnerSide(const btVector3 &pt, bool left) const {
        // then the innerPt and the closing direction define the plane
        return (getManipRot() * getClosingDirection(left)).dot(pt - getInnerPt(left)) > 0;
    }

    // Fills in the rcontacs array with contact information between psb and pco
    static void getContactPointsWith(btSoftBody *psb, btCollisionObject *pco, btSoftBody::tRContactArray &rcontacts) {
        // custom contact checking adapted from btSoftBody.cpp and btSoftBodyInternals.h
        struct Custom_CollideSDF_RS : btDbvt::ICollide {
            Custom_CollideSDF_RS(btSoftBody::tRContactArray &rcontacts_) : rcontacts(rcontacts_) { }

            void Process(const btDbvtNode* leaf) {
                btSoftBody::Node* node=(btSoftBody::Node*)leaf->data;
                DoNode(*node);
            }

            void DoNode(btSoftBody::Node& n) {
                const btScalar m=n.m_im>0?dynmargin:stamargin;
                btSoftBody::RContact c;
                if (!n.m_battach && psb->checkContact(m_colObj1,n.m_x,m,c.m_cti)) {
                    const btScalar  ima=n.m_im;
                    const btScalar  imb= m_rigidBody? m_rigidBody->getInvMass() : 0.f;
                    const btScalar  ms=ima+imb;
                    if(ms>0) {
                        // there's a lot of extra information we don't need to compute
                        // since we just want to find the contact points
#if 0
                        const btTransform&      wtr=m_rigidBody?m_rigidBody->getWorldTransform() : m_colObj1->getWorldTransform();
                        static const btMatrix3x3        iwiStatic(0,0,0,0,0,0,0,0,0);
                        const btMatrix3x3&      iwi=m_rigidBody?m_rigidBody->getInvInertiaTensorWorld() : iwiStatic;
                        const btVector3         ra=n.m_x-wtr.getOrigin();
                        const btVector3         va=m_rigidBody ? m_rigidBody->getVelocityInLocalPoint(ra)*psb->m_sst.sdt : btVector3(0,0,0);
                        const btVector3         vb=n.m_x-n.m_q; 
                        const btVector3         vr=vb-va;
                        const btScalar          dn=btDot(vr,c.m_cti.m_normal);
                        const btVector3         fv=vr-c.m_cti.m_normal*dn;
                        const btScalar          fc=psb->m_cfg.kDF*m_colObj1->getFriction();
#endif
                        c.m_node        =       &n;
#if 0
                        c.m_c0          =       ImpulseMatrix(psb->m_sst.sdt,ima,imb,iwi,ra);
                        c.m_c1          =       ra;
                        c.m_c2          =       ima*psb->m_sst.sdt;
                        c.m_c3          =       fv.length2()<(btFabs(dn)*fc)?0:1-fc;
                        c.m_c4          =       m_colObj1->isStaticOrKinematicObject()?psb->m_cfg.kKHR:psb->m_cfg.kCHR;
#endif
                        rcontacts.push_back(c);
#if 0
                        if (m_rigidBody)
                                m_rigidBody->activate();
#endif
                    }
                }
            }
            btSoftBody*             psb;
            btCollisionObject*      m_colObj1;
            btRigidBody*    m_rigidBody;
            btScalar                dynmargin;
            btScalar                stamargin;
            btSoftBody::tRContactArray &rcontacts;
        };

        Custom_CollideSDF_RS  docollide(rcontacts);              
        btRigidBody*            prb1=btRigidBody::upcast(pco);
        btTransform     wtr=pco->getWorldTransform();

        const btTransform       ctr=pco->getWorldTransform();
        const btScalar          timemargin=(wtr.getOrigin()-ctr.getOrigin()).length();
        const btScalar          basemargin=psb->getCollisionShape()->getMargin();
        btVector3                       mins;
        btVector3                       maxs;
        ATTRIBUTE_ALIGNED16(btDbvtVolume)               volume;
        pco->getCollisionShape()->getAabb(      pco->getWorldTransform(),
                mins,
                maxs);
        volume=btDbvtVolume::FromMM(mins,maxs);
        volume.Expand(btVector3(basemargin,basemargin,basemargin));             
        docollide.psb           =       psb;
        docollide.m_colObj1 = pco;
        docollide.m_rigidBody = prb1;

        docollide.dynmargin     =       basemargin+timemargin;
        docollide.stamargin     =       basemargin;
        psb->m_ndbvt.collideTV(psb->m_ndbvt.m_root,volume,docollide);
    }

    // adapted from btSoftBody.cpp (btSoftBody::appendAnchor)
    static void appendAnchor(btSoftBody *psb, btSoftBody::Node *node, btRigidBody *body, btScalar influence=1) {
        btSoftBody::Anchor a;
        a.m_node = node;
        a.m_body = body;
        a.m_local = body->getWorldTransform().inverse()*a.m_node->m_x;
        a.m_node->m_battach = 1;
        a.m_influence = influence;
        psb->m_anchors.push_back(a);
    }

    // Checks if psb is touching the inside of the gripper fingers
    // If so, attaches anchors to every contact point
    void attach(bool left) {
        btRigidBody *rigidBody =
            manip->robot->associatedObj(left ? leftFinger : rightFinger)->rigidBody.get();
        btSoftBody::tRContactArray rcontacts;
        getContactPointsWith(psb, rigidBody, rcontacts);
        cout << "got " << rcontacts.size() << " contacts\n";
        for (int i = 0; i < rcontacts.size(); ++i) {
            const btSoftBody::RContact &c = rcontacts[i];
            KinBody::LinkPtr colLink = manip->robot->associatedObj(c.m_cti.m_colObj);
            if (!colLink) continue;
            const btVector3 &contactPt = c.m_node->m_x;
            if (onInnerSide(contactPt, left)) {
                appendAnchor(psb, c.m_node, rigidBody);
                cout << "\tappending anchor\n";
            }
        }
    }

public:
    typedef boost::shared_ptr<PR2SoftBodyGripperAction> Ptr;
    PR2SoftBodyGripperAction(RaveRobotKinematicObject::Manipulator::Ptr manip_,
                  const string &leftFingerName,
                  const string &rightFingerName,
                  float time) :
            Action(time), manip(manip_), vals(1, 0),
            leftFinger(manip->robot->robot->GetLink(leftFingerName)),
            rightFinger(manip->robot->robot->GetLink(rightFingerName)),
            origLeftFingerInvTrans(manip->robot->getLinkTransform(leftFinger).inverse()),
            origRightFingerInvTrans(manip->robot->getLinkTransform(rightFinger).inverse()),
            centerPt(manip->getTransform().getOrigin()),
            indices(manip->manip->GetGripperIndices()),
            closingNormal(manip->manip->GetClosingDirection()[0],
                          manip->manip->GetClosingDirection()[1],
                          manip->manip->GetClosingDirection()[2]),
            toolDirection(util::toBtVector(manip->manip->GetLocalToolDirection())) // don't bother scaling
    {
        if (indices.size() != 1)
            cout << "WARNING: more than one gripper DOF; just choosing first one" << endl;
        setCloseAction();
    }

    void setEndpoints(dReal start, dReal end) { startVal = start; endVal = end; }
    dReal getCurrDOFVal() const {
        vector<dReal> v;
        manip->robot->robot->GetDOFValues(v);
        return v[indices[0]];
    }
    void setOpenAction() { setEndpoints(getCurrDOFVal(), OPEN_VAL); } // 0.54 is the max joint value for the pr2
    void setCloseAction() { setEndpoints(getCurrDOFVal(), CLOSED_VAL); }
    void toggleAction() {
        if (endVal == CLOSED_VAL)
            setOpenAction();
        else if (endVal == OPEN_VAL)
            setCloseAction();
    }

    // Must be called before the action is run!
    void setTarget(btSoftBody *psb_) { psb = psb_; }

    void releaseAllAnchors() {
        psb->m_anchors.clear();
    }

    void reset() {
        Action::reset();
        releaseAllAnchors();
    }

    void step(float dt) {
        if (done()) return;
        stepTime(dt);

        float frac = fracElapsed();
        vals[0] = (1.f - frac)*startVal + frac*endVal;
        manip->robot->setDOFValues(indices, vals);

        if (vals[0] == CLOSED_VAL) {
            attach(true);
            attach(false);
        }
    }
};


struct CustomScene : public Scene {
    PR2SoftBodyGripperAction::Ptr leftAction, rightAction;
    BulletInstance::Ptr bullet2;
    OSGInstance::Ptr osg2;
    Fork::Ptr fork;
    RaveRobotKinematicObject::Ptr origRobot, tmpRobot;
    PR2Manager pr2m;

    CustomScene() : pr2m(*this) { }

    BulletSoftObject::Ptr createCloth(btScalar s, const btVector3 &center);
    BulletSoftObject::Ptr createMesh(string filename, const btVector3 &center,vector<btVector3> *ctrlPts);
    void createFork();
    void swapFork();

    void run();
};

class CustomKeyHandler : public osgGA::GUIEventHandler {
    CustomScene &scene;
public:
    CustomKeyHandler(CustomScene &scene_) : scene(scene_) { }
    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
};

bool CustomKeyHandler::handle(const osgGA::GUIEventAdapter &ea,osgGA::GUIActionAdapter &) {
    switch (ea.getEventType()) {
    case osgGA::GUIEventAdapter::KEYDOWN:
        switch (ea.getKey()) {
        case 'a':
            scene.leftAction->reset();
            scene.leftAction->toggleAction();
            scene.runAction(scene.leftAction, BulletConfig::dt);
            break;
        case 's':
            scene.rightAction->reset();
            scene.rightAction->toggleAction();
            scene.runAction(scene.rightAction, BulletConfig::dt);
            break;
        case 'f':
            scene.createFork();
            break;
        case 'g':
            scene.swapFork();
            break;
        }
        break;
    }
    return false;
}

BulletSoftObject::Ptr CustomScene::createCloth(btScalar s, const btVector3 &center) {
    const int divs = 45;

    btSoftBody *psb = btSoftBodyHelpers::CreatePatch(
        env->bullet->softBodyWorldInfo,
        center + btVector3(-s,-s,0),
        center + btVector3(+s,-s,0),
        center + btVector3(-s,+s,0),
        center + btVector3(+s,+s,0),
        divs, divs,
        0, true);

    psb->m_cfg.piterations = 2;
    psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS
        | btSoftBody::fCollision::CL_RS;
    //    | btSoftBody::fCollision::CL_SELF;
    psb->m_cfg.kDF = 1.0;
    psb->getCollisionShape()->setMargin(0.05);
    btSoftBody::Material *pm = psb->appendMaterial();
    pm->m_kLST = 0.1;
    psb->generateBendingConstraints(2, pm);
    psb->randomizeConstraints();
    psb->setTotalMass(1, true);
    psb->generateClusters(0);

/*    for (int i = 0; i < psb->m_clusters.size(); ++i) {
        psb->m_clusters[i]->m_selfCollisionImpulseFactor = 0.1;
    }*/

    return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}

BulletSoftObject::Ptr CustomScene::createMesh(string filename,const btVector3 &center,vector<btVector3> *ctrlPts) {
	cout << "Center " << (center.getX()) << endl;
	cout << "Center " << (center.getY()) << endl;
	cout << "Center " << (center.getZ()) << endl;
	string line = "/home/henrybrad/Desktop/rope_surface.vtk";
	vtkSmartPointer<vtkGenericDataObjectReader> reader = 
	vtkSmartPointer<vtkGenericDataObjectReader>::New();
	reader->SetFileName(line.c_str());
	reader->Update();

	btTriangleMesh *mTriMesh = new btTriangleMesh();


	vtkPoints* point_array;
	vtkCellArray* cell_array;
	if(reader->IsFilePolyData()){
		point_array = reader->GetPolyDataOutput()->GetPoints();
		cell_array = reader->GetPolyDataOutput()->GetPolys();
	}
	else{
		point_array = reader->GetUnstructuredGridOutput()->GetPoints();
		cell_array = reader->GetUnstructuredGridOutput()->GetCells();
	}
		cout <<"Not poly data"<<endl;
		point_array->Print(cout);
		cell_array->Print(cout);
		cout << "Point 4: " << *((vtkFloatArray*)point_array->GetData())->GetPointer(3) << endl;
		cout << "Faces: " << *((vtkIdTypeArray*)cell_array->GetData())->GetPointer(0) << endl;
		float *ft = ((vtkFloatArray*)point_array->GetData())->GetPointer(0);
		int *tt = (int*)((vtkIdTypeArray*)cell_array->GetPointer());
		int t[((cell_array->GetSize()))/4][3];
		vector<btVector3> points;
		for(int i = 0; i<((cell_array->GetSize()))/4; i++)
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
		float xave = 0;
		float yave = 0;
		float zave = 0;
		int num = 0;
		for(int i = 0; i<(int)((point_array->GetNumberOfPoints())/1.0); i++)
		{
			xave += *(ft+i*3);
			yave += *(ft+i*3+1);
			zave += *(ft+i*3+2);
			num++;
		}
		xave = xave/num;
		yave = yave/num;
		zave = zave/num;
		for(int i = 0; i<(int)((point_array->GetNumberOfPoints())/1.0); i++)
		{
			*(ft+i*3) -= xave;
			*(ft+i*3+1) -= yave;
			*(ft+i*3+2) -= zave;
		}
			
		cout << ((cell_array->GetPointer())) << endl;


		cout << "Done making the soft body" << endl;
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
         for(j=0; j < i; j++)
         {
            btVector3 v2 =  btVector3(ft[j*3],ft[j*3+1],ft[j*3+2]);;
            if (v1 == v2) {
               dupVertices[i] = j;
               dupVerticesCount++;
               break;
            }
         }
      }
      printf("dupVerticesCount %d\n", dupVerticesCount);
      
      int newVertexCount = mVertexCount - dupVerticesCount;
      printf("newVertexCount %d\n", newVertexCount);
      btScalar vertices[newVertexCount * 3];
      for(i=0, j=0; i < mVertexCount; i++)
      {
         if (dupVertices[i] == -1) {
            btVector3 v =  btVector3(ft[i*3],ft[i*3+1],ft[i*3+2]);;
            vertices[j++] = v.getX();
            vertices[j++] = v.getY();
            vertices[j++] = v.getZ();
         }
      }
      
	int mIndexCount  = ((cell_array->GetSize()))*3.0/4;
      int indexes[mIndexCount];
      int idx, idxDup;
      for(i=0; i < mIndexCount; i++)
      {
         idx = t[i/3][i%3];
         idxDup = dupVertices[idx];
         printf("dup %d\n", idxDup);
         idx = idxDup == -1 ? idx : idxDup;
         indexes[i] = newIndexes[idx];
      }
      int ntriangles = mIndexCount / 3;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/henrybrad/Desktop/kinfu_tests/test2/cloud.pcd", *cloud) == -1) //* load the file
  	{
  	  PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
  	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb(new pcl::PointCloud<pcl::PointXYZRGB>(cloud->width,cloud->height));

	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
		pcl::PointXYZRGB p = pcl::PointXYZRGB(0,0,0);		
		p.x = cloud->points[i].x;
		p.y = cloud->points[i].y;
		p.z = cloud->points[i].z;
		cloudrgb->points.push_back(p);
	}
	

	vector<Eigen::Vector3f> corners;
	Eigen::Vector3f normal;
	getTable(cloudrgb,corners,normal);

	pcl::PointCloud<pcl::PointXYZRGB> cr(cloud->width,cloud->height);

	float minx = corners[0].x();
	float maxx = corners[0].x();
	float miny = corners[0].y();
	float maxy = corners[0].y();
	float minz = corners[0].z();
	float maxz = corners[0].z();
	for(int i = 1; i<corners.size(); i++)
	{
		if(corners[i].x()<minx)
			minx = corners[i].x();
		if(corners[i].x()>maxx)
			maxx = corners[i].x();
		if(corners[i].y()<miny)
			miny = corners[i].y();
		if(corners[i].y()>maxy)
			maxy = corners[i].y();
		if(corners[i].z()<minz)
			minz = corners[i].z();
		if(corners[i].z()>maxz)
			maxz = corners[i].z();
	}

	Eigen::Vector4f min(minx,miny,minz,1);
	Eigen::Vector4f max(maxx,maxy,maxz,1);

	pcl::CropBox<pcl::PointXYZRGB> cropper;
	cropper.setInputCloud(cloudrgb);
	cropper.setMin(min);
	cropper.setMax(max);
	cropper.filter(cr);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb2(new pcl::PointCloud<pcl::PointXYZRGB>(cr));
/*
	for (size_t i = 0; i < cloudrgb->points.size (); ++i)
	{
		pcl::PointXYZRGB p = cloudrgb->points[i];
		Eigen::Vector3f v1;	
		Eigen::Vector3f v2;	
		Eigen::Vector3f v3;	
		Eigen::Vector3f v4;	
		v1 = Eigen::Vector3f(p.x,p.y,p.z) - corners[0];
		v2 = Eigen::Vector3f(p.x,p.y,p.z) - corners[1];
		v3 = Eigen::Vector3f(p.x,p.y,p.z) - corners[2];
		v4 = Eigen::Vector3f(p.x,p.y,p.z) - corners[3];
		Eigen::Vector3f total = v1+v2+v3+v4;
		Eigen::Vector3f cross = normal.cross(total);
		if(cross.norm()<1.5)	
			cloudrgb2->points.push_back(p);
	}
*/
	for (size_t i = 0; i < cloudrgb->points.size (); ++i)
	{
		cloudrgb->points[i].x += .1;
		cloudrgb->points[i].y += .1;
		cloudrgb->points[i].z += .1;
		cloudrgb->points[i].r += 255;
	}

	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   	viewer.showCloud (cloudrgb);
   	viewer.showCloud (cloudrgb2,"cloud2");
	while (!viewer.wasStopped ())
  	{
  	}
	


	btSoftBody *psb = btSoftBodyHelpers::CreateFromTriMesh(env->bullet->softBodyWorldInfo,vertices, indexes, ntriangles);

	const char * ele = "/home/henrybrad/Desktop/tetgen1.4.3/rope_surface.1.ele";
	FILE *fp;
	long len;
	char *ele_buf;
	fp=fopen(ele,"rb");
	fseek(fp,0,SEEK_END); //go to end
	len=ftell(fp); //get position at end (length)
	fseek(fp,0,SEEK_SET); //go to beg.
	ele_buf=(char *)malloc(len); //malloc buffer
	fread(ele_buf,len,1,fp); //read into buffer
	fclose(fp);
	
	const char * face = "/home/henrybrad/Desktop/tetgen1.4.3/rope_surface.1.face";
	char *face_buf;
	fp=fopen(face,"rb");
	fseek(fp,0,SEEK_END); //go to end
	len=ftell(fp); //get position at end (length)
	fseek(fp,0,SEEK_SET); //go to beg.
	face_buf=(char *)malloc(len); //malloc buffer
	fread(face_buf,len,1,fp); //read into buffer
	fclose(fp);
	cout << "Read Files" << endl;

	const char * node = "/home/henrybrad/Desktop/tetgen1.4.3/rope_surface.1.node";
	char *node_buf;
	fp=fopen(node,"rb");
	fseek(fp,0,SEEK_END); //go to end
	len=ftell(fp); //get position at end (length)
	fseek(fp,0,SEEK_SET); //go to beg.
	node_buf=(char *)malloc(len); //malloc buffer
	fread(node_buf,len,1,fp); //read into buffer
	fclose(fp);
	cout << "Read Files" << endl;

	//btSoftBody *psb = btSoftBodyHelpers::CreateFromTetGenData(env->bullet->softBodyWorldInfo,ele_buf,face_buf,node_buf,false,true,true);

    psb->m_cfg.piterations = 20;
	psb->setVolumeMass(1);
    psb->m_cfg.collisions = btSoftBody::fCollision::CL_RS
	| btSoftBody::fCollision::CL_SS;
        //| btSoftBody::fCollision::CL_SELF;
    psb->m_cfg.kDF = .1;
    //psb->m_cfg.kSSHR_CL = 1.;
    //psb->m_cfg.kSRHR_CL = 1.;
    //psb->m_cfg.kSKHR_CL = 1.;
    //psb->getCollisionShape()->setMargin(0.04);
    btSoftBody::Material *pm = psb->appendMaterial();
    pm->m_kLST = .5;
    psb->generateBendingConstraints(2, pm);
//    psb->randomizeConstraints();
	psb->scale(btVector3(35,35,35));
//	psb->translate(btVector3(-52,-51,15));
	psb->translate(btVector3(24,0,24));
    psb->setTotalMass(.001, true);
    psb->generateClusters(0);


/*	Wm5::BSplineCurveFitf b(6,newVertexCount,vertices,5,50);
	for(float i = 0; i<1; i+=.003){
		float f[3];
		b.GetPosition(i,&f[0]);

		f[0] *= 20;
		f[0] += 20;
		f[1] *= 20;
		f[1] += 0;
		f[2] *= 20;
		f[2] += 24;
		
		ctrlPts->push_back(btVector3(f[0],f[1],f[2]));
		cout << f[0] << " " << f[1] << " " << f[2] << endl;
	}
*/
    		return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}

void CustomScene::createFork() {
    bullet2.reset(new BulletInstance);
    bullet2->setGravity(BulletConfig::gravity);
    osg2.reset(new OSGInstance);
    osg->root->addChild(osg2->root.get());

    fork.reset(new Fork(env, bullet2, osg2));
    registerFork(fork);

    cout << "forked!" << endl;

    origRobot = pr2m.pr2;
    EnvironmentObject::Ptr p = fork->forkOf(pr2m.pr2);
    if (!p) {
        cout << "failed to get forked version of robot!" << endl;
        return;
    }
    tmpRobot = boost::static_pointer_cast<RaveRobotKinematicObject>(p);
    cout << (tmpRobot->getEnvironment() == env.get()) << endl;
    cout << (tmpRobot->getEnvironment() == fork->env.get()) << endl;
}

void CustomScene::swapFork() {
    // swaps the forked robot with the real one
    cout << "swapping!" << endl;
    int leftidx = pr2m.pr2Left->index;
    int rightidx = pr2m.pr2Right->index;
    origRobot.swap(tmpRobot);
    pr2m.pr2 = origRobot;
    pr2m.pr2Left = pr2m.pr2->getManipByIndex(leftidx);
    pr2m.pr2Right = pr2m.pr2->getManipByIndex(rightidx);

/*    vector<int> indices; vector<dReal> vals;
    for (int i = 0; i < tmpRobot->robot->GetDOF(); ++i) {
        indices.push_back(i);
        vals.push_back(0);
    }
    tmpRobot->setDOFValues(indices, vals);*/
}

void CustomScene::run() {
    viewer.addEventHandler(new CustomKeyHandler(*this));

    const float dt = BulletConfig::dt;
    const float table_height = .5;
    const float table_thickness = .05;
    BoxObject::Ptr table(
        new BoxObject(0, GeneralConfig::scale * btVector3(.75,.75,table_thickness/2),
            btTransform(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(1.2, 0, table_height-table_thickness/2))));
    table->rigidBody->setFriction(10);

	vector<btVector3> ctrlPts;
//    BulletSoftObject::Ptr cloth(
//            createCloth(GeneralConfig::scale * 0.25, GeneralConfig::scale * btVector3(0.9, 0, table_height+0.01)));
	BulletSoftObject::Ptr cloth(createMesh("test",GeneralConfig::scale * btVector3(0.9, 0, table_height+0.01),&ctrlPts));
//cout << "Made it out of createMesh" << endl;
	//btRigidBody cloth(createMesh("test",GeneralConfig::scale * btVector3(0.9, 0, table_height+0.01)));
cout << "Made it out of createMesh" << endl;
    btSoftBody * const psb = cloth->softBody.get();
    pr2m.pr2->ignoreCollisionWith(psb);

	boost::shared_ptr<CapsuleRope> ropePtr(new CapsuleRope(ctrlPts,.03));

//	for(int i = 0; i<200; i++){
//		btVector3 v = ctrlPts[i];
  //  		SphereObject::Ptr sphere(new SphereObject(1, 0.003 * GeneralConfig::scale,
    //    	        btTransform(btQuaternion(0,0,0, 1), v)));
//    		env->add(sphere);
//	}

    env->add(table);
//	env->add(ropePtr);
	
    env->add(cloth);

    leftAction.reset(new PR2SoftBodyGripperAction(pr2m.pr2Left, "l_gripper_l_finger_tip_link", "l_gripper_r_finger_tip_link", 1));
    leftAction->setTarget(psb);
    rightAction.reset(new PR2SoftBodyGripperAction(pr2m.pr2Right, "r_gripper_l_finger_tip_link", "r_gripper_r_finger_tip_link", 1));
   rightAction->setTarget(psb);

    //setSyncTime(true);
    startViewer();
    stepFor(dt, 3);

   
    leftAction->setOpenAction();
    runAction(leftAction, dt);

    rightAction->setOpenAction();
    runAction(rightAction, dt);
    

    startFixedTimestepLoop(dt);
}

int main(int argc, char *argv[]) {
    GeneralConfig::scale = 20.;
    ViewerConfig::cameraHomePosition = btVector3(100, 0, 100);
    BulletConfig::dt = 0.01;
    BulletConfig::internalTimeStep = 0.01;
    BulletConfig::maxSubSteps = 0;

//	SceneConfig::enableIK = false;

    Parser parser;

    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.read(argc, argv);


    CustomScene().run();
    return 0;
}
