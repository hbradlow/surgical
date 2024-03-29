#include "pr2.h"
#include "simulation/environment.h"
#include "thread_socket_interface.h"

static const char LEFT_GRIPPER_LEFT_FINGER_NAME[] = "l_gripper_l_finger_tip_link";
static const char LEFT_GRIPPER_RIGHT_FINGER_NAME[] = "l_gripper_r_finger_tip_link";

static const char RIGHT_GRIPPER_LEFT_FINGER_NAME[] = "r_gripper_l_finger_tip_link";
static const char RIGHT_GRIPPER_RIGHT_FINGER_NAME[] = "r_gripper_r_finger_tip_link";

// adapted from btSoftBody.cpp (btSoftBody::appendAnchor)
static void btSoftBody_appendAnchor(btSoftBody *psb, btSoftBody::Node *node, btRigidBody *body, btScalar influence=1) {
    btSoftBody::Anchor a = { 0 };
    a.m_node = node;
    a.m_body = body;
    a.m_local = body->getWorldTransform().inverse()*a.m_node->m_x;
    a.m_node->m_battach = 1;
    a.m_influence = influence;
    psb->m_anchors.push_back(a);
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

PR2SoftBodyGripper::PR2SoftBodyGripper(RaveRobotKinematicObject::Ptr robot_, OpenRAVE::RobotBase::ManipulatorPtr manip_, bool leftGripper) :
        robot(robot_), manip(manip_),
        leftFinger(robot->robot->GetLink(leftGripper ? LEFT_GRIPPER_LEFT_FINGER_NAME : RIGHT_GRIPPER_LEFT_FINGER_NAME)),
        rightFinger(robot->robot->GetLink(leftGripper ? LEFT_GRIPPER_RIGHT_FINGER_NAME : RIGHT_GRIPPER_RIGHT_FINGER_NAME)),
        origLeftFingerInvTrans(robot->getLinkTransform(leftFinger).inverse()),
        origRightFingerInvTrans(robot->getLinkTransform(rightFinger).inverse()),
        centerPt(util::toBtTransform(manip->GetTransform(), robot->scale).getOrigin()),
        closingNormal(manip->GetClosingDirection()[0],
                      manip->GetClosingDirection()[1],
                      manip->GetClosingDirection()[2]),
        toolDirection(util::toBtVector(manip->GetLocalToolDirection())), // don't bother scaling
        grabOnlyOnContact(false)
{
}

void PR2SoftBodyGripper::attach(bool left) {
    btRigidBody *rigidBody =
        robot->associatedObj(left ? leftFinger : rightFinger)->rigidBody.get();
    btSoftBody::tRContactArray rcontacts;
    getContactPointsWith(psb, rigidBody, rcontacts);
    cout << "got " << rcontacts.size() << " contacts\n";
    int nAppended = 0;
    for (int i = 0; i < rcontacts.size(); ++i) {
        const btSoftBody::RContact &c = rcontacts[i];
        KinBody::LinkPtr colLink = robot->associatedObj(c.m_cti.m_colObj);
        if (!colLink) continue;
        const btVector3 &contactPt = c.m_node->m_x;
        if (onInnerSide(contactPt, left)) {
            btSoftBody_appendAnchor(psb, c.m_node, rigidBody);
            ++nAppended;
        }
    }
    cout << "appended " << nAppended << " anchors\n";
}

void PR2SoftBodyGripper::grab() {
    if (grabOnlyOnContact) {
        attach(false);
        attach(true);
    } else {
        // the gripper should be closed
        const btVector3 midpt = 0.5 * (getInnerPt(false) + getInnerPt(true));
        // get point on cloth closest to midpt, and attach an anchor there
        // (brute-force iteration through every cloth node)
        btSoftBody::tNodeArray &nodes = psb->m_nodes;
        btSoftBody::Node *closestNode = NULL;
        btScalar closestDist;
        for (int i = 0; i < nodes.size(); ++i) {
            btSoftBody::Node &n = nodes[i];
            btScalar d2 = midpt.distance2(n.m_x);
            if (closestNode == NULL || d2 < closestDist) {
                closestNode = &n;
                closestDist = d2;
            }
        }
        // attach to left finger (arbitrary choice)
        if (closestNode)
            btSoftBody_appendAnchor(psb, closestNode, robot->associatedObj(leftFinger)->rigidBody.get());
    }
}


PR2Manager::PR2Manager(Scene &s) : scene(s), inputState() {
    loadRobot();
    initIK();
    if (SceneConfig::enableHaptics)
        connectionInit(); // socket connection for haptics
    registerSceneCallbacks();
}

void PR2Manager::registerSceneCallbacks() {
    Scene::Callback mousecb = boost::bind(&PR2Manager::processMouseInput, this, _1);
    scene.addCallback(osgGA::GUIEventAdapter::PUSH, mousecb);
    scene.addCallback(osgGA::GUIEventAdapter::DRAG, mousecb);

    Scene::Callback keycb = boost::bind(&PR2Manager::processKeyInput, this, _1);
    scene.addCallback(osgGA::GUIEventAdapter::KEYDOWN, keycb);
    scene.addCallback(osgGA::GUIEventAdapter::KEYUP, keycb);
}

void PR2Manager::loadRobot() {
    if (!SceneConfig::enableRobot) return;
    btTransform trans(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0));
    static const char ROBOT_MODEL_FILE[] = EXPAND(BULLETSIM_DATA_DIR) "/robot_model/pr2_with_kinect.dae";
    pr2.reset(new RaveRobotKinematicObject(scene.rave, ROBOT_MODEL_FILE, trans, GeneralConfig::scale));
    scene.env->add(pr2);
//    pr2->ignoreCollisionWith(ground->rigidBody.get()); // the robot's always touching the ground anyway
}

void PR2Manager::initIK() {
    if (!SceneConfig::enableIK || !SceneConfig::enableRobot) return;
    if (!pr2) {
        //LOG(warning) << "cannot initialize IK since the PR2 model is not yet loaded";
        return;
    }
    pr2Left = pr2->createManipulator("leftarm", SceneConfig::useFakeGrabber);
    pr2Right = pr2->createManipulator("rightarm", SceneConfig::useFakeGrabber);
    if (SceneConfig::useFakeGrabber) {
        scene.env->add(pr2Left->grabber);
        scene.env->add(pr2Right->grabber);
    }
}

void PR2Manager::processHapticInput() {
    if (!SceneConfig::enableRobot || !SceneConfig::enableHaptics)
        return;

    // read the haptic controllers
    btTransform trans0, trans1;
    bool buttons0[2], buttons1[2];
    static bool lastButton[2] = { false, false };
    if (!util::getHapticInput(trans0, buttons0, trans1, buttons1))
        return;

    pr2Left->moveByIK(trans0, SceneConfig::enableRobotCollision, true);
    if (buttons0[0] && !lastButton[0]) {
        if (SceneConfig::useFakeGrabber)
            pr2Left->grabber->grabNearestObjectAhead();
        else
            cout << "not implemented" << endl;
    }
    else if (!buttons0[0] && lastButton[0]) {
        if (SceneConfig::useFakeGrabber)
            pr2Left->grabber->releaseConstraint();
        else
            cout << "not implemented" << endl;
    }
    lastButton[0] = buttons0[0];

    pr2Right->moveByIK(trans0, SceneConfig::enableRobotCollision, true);
    if (buttons1[0] && !lastButton[1]) {
        if (SceneConfig::useFakeGrabber)
            pr2Right->grabber->grabNearestObjectAhead();
        else
            cout << "not implemented" << endl;
    }
    else if (!buttons1[0] && lastButton[1]) {
        if (SceneConfig::useFakeGrabber)
            pr2Right->grabber->releaseConstraint();
        else
            cout << "not implemented" << endl;
    }
    lastButton[1] = buttons1[0];
}

bool PR2Manager::processKeyInput(const osgGA::GUIEventAdapter &ea) {
    switch (ea.getEventType()) {
    case osgGA::GUIEventAdapter::KEYDOWN:
        switch (ea.getKey()) {
        case '1':
            inputState.moveManip0 = true; break;
        case '2':
            inputState.moveManip1 = true; break;
        case 'q':
            inputState.rotateManip0 = true; break;
        case 'w':
            inputState.rotateManip1 = true; break;
        }
        break;
    case osgGA::GUIEventAdapter::KEYUP:
        switch (ea.getKey()) {
        case '1':
            inputState.moveManip0 = false; break;
        case '2':
            inputState.moveManip1 = false; break;
        case 'q':
            inputState.rotateManip0 = false; break;
        case 'w':
            inputState.rotateManip1 = false; break;
        }
        break;
    }
    return false;
}

bool PR2Manager::processMouseInput(const osgGA::GUIEventAdapter &ea) {
    if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH) {
        inputState.startDragging = true;
    } else if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG) {
        // drag the active manipulator in the plane of view
        if (SceneConfig::enableRobot && SceneConfig::enableIK &&
              (ea.getButtonMask() & ea.LEFT_MOUSE_BUTTON) &&
              (inputState.moveManip0 || inputState.moveManip1 ||
               inputState.rotateManip0 || inputState.rotateManip1)) {
            if (inputState.startDragging) {
                inputState.dx = inputState.dy = 0;
            } else {
                inputState.dx = inputState.lastX - ea.getXnormalized();
                inputState.dy = ea.getYnormalized() - inputState.lastY;
            }
            inputState.lastX = ea.getXnormalized(); inputState.lastY = ea.getYnormalized();
            inputState.startDragging = false;

            // get our current view
            osg::Vec3d osgCenter, osgEye, osgUp;
            scene.manip->getTransformation(osgCenter, osgEye, osgUp);
            btVector3 from(util::toBtVector(osgEye));
            btVector3 to(util::toBtVector(osgCenter));
            btVector3 up(util::toBtVector(osgUp)); up.normalize();

            // compute basis vectors for the plane of view
            // (the plane normal to the ray from the camera to the center of the scene)
            btVector3 normal = (to - from).normalized();
            btVector3 yVec = (up - (up.dot(normal))*normal).normalized(); //FIXME: is this necessary with osg?
            btVector3 xVec = normal.cross(yVec);
            btVector3 dragVec = SceneConfig::mouseDragScale * (inputState.dx*xVec + inputState.dy*yVec);

            RaveRobotKinematicObject::Manipulator::Ptr manip;
            if (inputState.moveManip0 || inputState.rotateManip0)
                manip = pr2Left;
            else
                manip = pr2Right;

            btTransform origTrans = manip->getTransform();
            btTransform newTrans(origTrans);

            if (inputState.moveManip0 || inputState.moveManip1)
                // if moving the manip, just set the origin appropriately
                newTrans.setOrigin(dragVec + origTrans.getOrigin());
            else if (inputState.rotateManip0 || inputState.rotateManip1) {
                // if we're rotating, the axis is perpendicular to the
                // direction the mouse is dragging
                btVector3 axis = normal.cross(dragVec);
                btScalar angle = dragVec.length();
                btQuaternion rot(axis, angle);
                // we must ensure that we never get a bad rotation quaternion
                // due to really small (effectively zero) mouse movements
                // this is the easiest way to do this:
                if (rot.length() > 0.99f && rot.length() < 1.01f)
                    newTrans.setRotation(rot * origTrans.getRotation());
            }
            manip->moveByIK(newTrans, SceneConfig::enableRobotCollision, true);
            return true;
        }
    }
    return false;
}
