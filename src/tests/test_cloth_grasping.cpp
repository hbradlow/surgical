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
#include <vtkMergePoints.h>
#include <vtkCleanPolyData.h>
#include <string>
#include <vector>

#include <vtkCellArray.h>
 
#include <sstream>

#include <Wm5Core.h>
#include <Wm5BSplineCurveFit.h>

#include "simulation/rope.h"

#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/apps/dominant_plane_segmentation.h>

#include "clouds/get_table.h"

#include "clouds/ply_pipeline.h"

#include <cmath>



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
    BulletSoftObject::Ptr createMeshFromTetGen(string filename, const btVector3 &center);
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
BulletSoftObject::Ptr CustomScene::createMeshFromTetGen(string filename,const btVector3 &center) {

	cout << "CENTER: " << center.x() << " " << center.y() << " " << center.z() << endl;
	string ele = filename + ".ele";
	ifstream ele_file(ele.c_str());
	std::string ele_str((std::istreambuf_iterator<char>(ele_file)),
			                 std::istreambuf_iterator<char>());

	string face = filename + ".face";
	ifstream face_file(face.c_str());
	std::string face_str((std::istreambuf_iterator<char>(face_file)),
			                 std::istreambuf_iterator<char>());
	
	string node = filename + ".node";
	ifstream node_file(node.c_str());
	std::string node_str((std::istreambuf_iterator<char>(node_file)),
			                 std::istreambuf_iterator<char>());
	
	btSoftBody *psb = btSoftBodyHelpers::CreateFromTetGenData(env->bullet->softBodyWorldInfo,ele_str.c_str(),face_str.c_str(),node_str.c_str(),true,true,true);

	//set the soft body attributes
    psb->m_cfg.piterations = 2;
    psb->m_cfg.collisions = btSoftBody::fCollision::CL_RS
	| btSoftBody::fCollision::CL_SS;
        //| btSoftBody::fCollision::CL_SELF;
    psb->m_cfg.kDF = .1;
    psb->m_cfg.kSSHR_CL = 1.;
    psb->m_cfg.kSRHR_CL = 1.;
    psb->m_cfg.kSKHR_CL = 1.;
    psb->getCollisionShape()->setMargin(0.04);
    btSoftBody::Material *pm = psb->appendMaterial();
    pm->m_kLST = .5;
	pm->m_flags     +=  btSoftBody::fMaterial::DebugDraw;
    psb->generateBendingConstraints(2, pm);
    psb->randomizeConstraints();


	psb->setVolumeMass(150);
    psb->generateClusters(1000);
	
	cout << "PSB: " << psb << endl;


	return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}

BulletSoftObject::Ptr CustomScene::createMesh(string filename,const btVector3 &center,vector<btVector3> *ctrlPts) {
	string line = "/home/henrybrad/Desktop/kinfu_tests/test11/mesh.vtk";
	process_raw("/home/henrybrad/Desktop/kinfu_tests/test11/mesh.vtk");
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

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb (new pcl::PointCloud<pcl::PointXYZRGB>(1,1));
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

		if(p0.dot(normal)<.017 || !(p0.dot(c0)>0 && p0.dot(c1)>0 && p0.dot(c0)<c0.dot(c0) && p0.dot(c1)<c1.dot(c1))){
			dupVertices[i] = -2;
			dupVerticesCount++;
		}
		/*
	 for(j=0; j < i; j++)
	 {
		btVector3 v2 =  btVector3(ft[j*3],ft[j*3+1],ft[j*3+2]);;

		Eigen::Vector3f p0 = Eigen::Vector3f(ft[i*3],ft[i*3+1],ft[i*3+2])-corners[0];

		if(p0.dot(normal)<.06 || !(p0.dot(c0)>0 && p0.dot(c1)>0 && p0.dot(c0)<c0.dot(c0) && p0.dot(c1)<c1.dot(c1))){
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

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb_seg (new pcl::PointCloud<pcl::PointXYZRGB>(1,1));
	for(int i = 0; i<maxIdx; i++){
		pcl::PointXYZRGB p;
		p.x = vertices[i*3];
		p.y = vertices[i*3+1];
		p.z = vertices[i*3+2];
		p.r = 255; 
		p.g = 0;
		p.b = 0;
		cloudrgb_seg->points.push_back(p);
	}
	pcl::visualization::CloudViewer viewer2 ("Simple Cloud Viewer");
	   viewer2.showCloud (cloudrgb_seg);
	      while (!viewer2.wasStopped ())
			     {
					    }

	Eigen::Vector3f middle = (corners[0] + corners[1] + corners[2] + corners[3])/4.0;
	Eigen::Vector3f sub = Eigen::Vector3f(center.getX()-middle.x(),center.getY()-middle.y(),center.getZ()-middle.z()); 

	for(int i = 0; i<=maxIdx; i++){
		vertices[i*3] -= middle.x();
		vertices[i*3+1] -= middle.y();
		vertices[i*3+2] -= middle.z();
	}
	

	btSoftBody *psb = btSoftBodyHelpers::CreateFromTriMesh(env->bullet->softBodyWorldInfo,vertices, indexes, ntriangles);


	//set the soft body attributes
    psb->m_cfg.piterations = 200;
    psb->m_cfg.collisions = btSoftBody::fCollision::CL_RS
	| btSoftBody::fCollision::CL_SS;
        //| btSoftBody::fCollision::CL_SELF;
    psb->m_cfg.kDF = .1;
    psb->m_cfg.kSSHR_CL = 1.;
    psb->m_cfg.kSRHR_CL = 1.;
    psb->m_cfg.kSKHR_CL = 1.;
    psb->getCollisionShape()->setMargin(0.04);
    btSoftBody::Material *pm = psb->appendMaterial();
    pm->m_kLST = .5;
    psb->generateBendingConstraints(2, pm);
    psb->randomizeConstraints();

	psb->scale(btVector3(35,35,35));

	btVector3 n(normal.x(),normal.y(),normal.z());
	btVector3 tableN(0,0,1);
	psb->rotate(btQuaternion(tableN.cross(n),-acos(((n).dot(tableN))/(n.length()*tableN.length()))));

	btVector3 side(corners[1].x()-corners[0].x(),corners[1].z()-corners[0].z(),corners[1].y()-corners[0].y());
	btVector3 tableSide(0,0,1);
	psb->rotate(btQuaternion(tableN,-acos(((side).dot(tableSide))/(side.length()*tableSide.length()))));

	psb->translate(center);
	psb->translate(btVector3(5,0,0));

    psb->setTotalMass(.001, true);
    psb->generateClusters(0);


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
	BulletSoftObject::Ptr cloth(createMeshFromTetGen("out_tetgen.1",GeneralConfig::scale * btVector3(0.9, 0, table_height+0.01)));
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
