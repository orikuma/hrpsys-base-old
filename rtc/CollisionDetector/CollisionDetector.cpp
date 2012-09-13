// -*- C++ -*-
/*!
 * @file  CollisionDetector.cpp
 * @brief collisoin detector component
 * $Date$
 *
 * $Id$
 */

#include <iomanip>
#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>
#include <hrpUtil/Eigen3d.h>
#include <hrpUtil/Eigen4d.h>
#include <hrpCollision/ColdetModel.h>
#include "util/GLbody.h"
#include "util/GLutil.h"
#include "util/BVutil.h"

#include "CollisionDetector.h"

#define deg2rad(x)	((x)*M_PI/180)
#define rad2deg(x)      ((x)*180/M_PI)

// Module specification
// <rtc-template block="module_spec">
static const char* component_spec[] =
{
    "implementation_id", "CollisionDetector",
    "type_name",         "CollisionDetector",
    "description",       "collisoin detector component",
    "version",           "1.0",
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.project", "",
    ""
};
// </rtc-template>

CollisionDetector::CollisionDetector(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_qRefIn("qRef", m_qRef),
      m_qCurrentIn("qCurrent", m_qCurrent),
      m_qOut("q", m_q),
      // </rtc-template>
      m_glbody(NULL),
      m_use_viewer(false),
      m_robot(NULL),
      m_scene(&m_log),
      m_window(&m_scene, &m_log),
      dummy(0)
{
    m_log.enableRingBuffer(1);
}

CollisionDetector::~CollisionDetector()
{
}



RTC::ReturnCode_t CollisionDetector::onInitialize()
{
    std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
  
    // </rtc-template>

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("qRef", m_qRefIn);
    addInPort("qCurrent", m_qCurrentIn);

    // Set OutPort buffer
    addOutPort("q", m_qOut);
  
    // Set service provider to Ports
  
    // Set service consumers to Ports
  
    // Set CORBA Service Ports
  
    // </rtc-template>

    //RTC::Properties& prop = getProperties();

    return RTC::RTC_OK;
}



/*
  RTC::ReturnCode_t CollisionDetector::onFinalize()
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t CollisionDetector::onActivated(RTC::UniqueId ec_id)
{
    std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;

    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());

    RTC::Properties& prop = getProperties();

    coil::stringTo(m_dt, prop["dt"].c_str());

    if ( prop["collision_viewer"] == "true" ) {
	m_use_viewer = true;
    }

    m_glbody = new GLbody();
    m_robot = hrp::BodyPtr(m_glbody);
    //
    OpenHRP::BodyInfo_var binfo;
    binfo = hrp::loadBodyInfo(prop["model"].c_str(),
			      CosNaming::NamingContext::_duplicate(naming.getRootContext()));
    if (CORBA::is_nil(binfo)){
	std::cerr << "failed to load model[" << prop["model"] << "]"
		  << std::endl;
	return RTC::RTC_ERROR;
    }
    if (!loadBodyFromBodyInfo(m_robot, binfo, true, GLlinkFactory)) {
	std::cerr << "failed to load model[" << prop["model"] << "]" << std::endl;
	return RTC::RTC_ERROR;
    }
    loadShapeFromBodyInfo(m_glbody, binfo);
    //convertToConvexHull(m_robot);
    //convertToAABB(m_robot);

    if ( prop["collision_pair"] != "" ) {
	std::cerr << "prop[collision_pair] ->" << prop["collision_pair"] << std::endl;
	std::istringstream iss(prop["collision_pair"]);
	std::string tmp;
	while (getline(iss, tmp, ' ')) {
	    size_t pos = tmp.find_first_of(':');
	    std::string name1 = tmp.substr(0, pos), name2 = tmp.substr(pos+1);
	    std::cerr << "check collisions between " << m_robot->link(name1)->name << " and " <<  m_robot->link(name2)->name << std::endl;
	    m_pair.push_back(new hrp::ColdetLinkPair(m_robot->link(name1), m_robot->link(name2))); // tolerance
            //m_pair.push_back(new hrp::ColdetLinkPair(m_robot->link(name1), m_robot->link(name2), 200)); // tolerance
	}
    }

    if ( m_pair.size() == 0 ) {
	std::cerr << "failed to setup collisions" << std::endl;
	return RTC::RTC_ERROR;
    }

    m_scene.addBody(m_glbody);

    // allocate memory for outPorts
    m_q.data.length(m_robot->numJoints());
    m_recover_time = 0;
    m_safe_posture = true;
    i_dt = 1.0;
    default_recover_time = 2.5/m_dt;
    m_recover_jointdata = (double *)malloc(sizeof(double)*m_robot->numJoints());
    m_interpolator = new interpolator(m_robot->numJoints(), i_dt);
    return RTC::RTC_OK;
}

RTC::ReturnCode_t CollisionDetector::onDeactivated(RTC::UniqueId ec_id)
{
    std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
    free(m_recover_jointdata);
    delete m_interpolator;
    return RTC::RTC_OK;
}

RTC::ReturnCode_t CollisionDetector::onExecute(RTC::UniqueId ec_id)
{
    if (m_qRefIn.isNew()) {
	m_qRefIn.read();

	assert(m_qRef.data.length() == m_robot->numJoints());
        if ( m_use_viewer ) {
          for (int i=0; i<m_glbody->numLinks(); i++){
            ((GLlink *)m_glbody->link(i))->highlight(false);
          }
        }
        //set robot model's angle for collision check(two types)
        //  1. current safe angle .. check based on qRef
        //  2. recovery or collision angle .. check based on q'(m_recover_jointdata)
        if (m_safe_posture && m_recover_time == 0) {           // 1. current safe angle
          for ( int i = 0; i < m_robot->numJoints(); i++ ){
	    m_robot->joint(i)->q = m_qRef.data[i];
          }
        }else{   // recovery or collision angle
          for ( int i = 0; i < m_robot->numJoints(); i++ ){
	    m_robot->joint(i)->q = m_recover_jointdata[i];
          }
        }
        //collision check process in case of angle set above
	m_robot->calcForwardKinematics();
	m_robot->updateLinkColdetModelPositions();
        m_safe_posture = true;
	coil::TimeValue tm1 = coil::gettimeofday();
	for (unsigned int i = 0; i < m_pair.size(); i++){
	    hrp::ColdetLinkPairPtr p = m_pair[i];
#if 1
	    if ( p->checkCollision()) {
              m_safe_posture = false;
              hrp::JointPathPtr jointPath = m_robot->getJointPath(p->link(0),p->link(1));
              //std::cerr << i << "/" << m_pair.size() << " pair: " << p->link(0)->name << "/" << p->link(1)->name << "(" << jointPath->numJoints() << ")" << std::endl;
              if ( m_use_viewer ) {
                ((GLlink *)p->link(0))->highlight(true);
                ((GLlink *)p->link(1))->highlight(true);
              }
	    }
#else
	    double point0[3], point1[3];
	    double d = p->computeDistance(point0, point1);
	    if ( d <= 0.05 ) {
		m_safe_posture = false;
		hrp::JointPathPtr jointPath = m_robot->getJointPath(p->link(0),p->link(1));
		std::cerr << i << "/" << m_pair.size() << " pair: " << p->link(0)->name << "/" << p->link(1)->name << "(" << jointPath->numJoints() << "), distance = " << d << std::endl;
	    }
#endif
	}
        //     mode : m_safe_posture : recover_time  : set as q
        // safe     :           true :            0  : qRef
        // collison :          false :         >  0  : q( do nothing)
        // recover  :           true :         >  0  : q'
        //std::cerr << "m_recover_time: " << m_recover_time << std::endl;
        coil::TimeValue tm2 = coil::gettimeofday();
        if (m_safe_posture && m_recover_time == 0){ // safe mode
          //std::cerr << "safe-------------- " << std::endl;
          for ( int i = 0; i < m_q.data.length(); i++ ) {
            m_q.data[i] = m_qRef.data[i];
          }
        } else {
          if(m_safe_posture){  //recover
            //std::cerr << "recover-------------- " << std::endl;
            for ( int i = 0; i < m_q.data.length(); i++ ) {
              m_q.data[i] = m_recover_jointdata[i];
            }
            m_recover_time = m_recover_time - i_dt;
          }else{ //collision
            //std::cerr << "collision-------------- " << std::endl;
            //do nothing (stay previous m_q)
            m_recover_time = default_recover_time;      // m_recover_time should be set based on difference between qRef and q
            m_interpolator->set(m_q.data.get_buffer()); //Set initial angle
          }
          //calc q'
#if 0
          //linear interpolation (dangerous)
          for ( int i = 0; i < m_q.data.length(); i++ ) {
            m_recover_jointdata[i] = m_q.data[i] + (m_qRef.data[i] - m_q.data[i]) / m_recover_time;
          }
#else
          //minjerk interpolation
          m_interpolator->setGoal(m_qRef.data.get_buffer(), m_recover_time);
          m_interpolator->get(m_recover_jointdata);
#endif
        }
#if 0
	std::cerr << "check collisions for for " << m_pair.size() << " pairs in " << (tm2.sec()-tm1.sec())*1000+(tm2.usec()-tm1.usec())/1000.0 
                  << " [msec], safe = " << m_safe_posture << ", time = " << m_recover_time << " " << m_q.data[0] << " " << m_q.data[1] 
                  << " " << m_q.data[2] << " " << m_q.data[3] << " " << m_q.data[4] << " " << m_q.data[5] << " " << m_q.data[6] << std::endl;
#endif
        //
        m_qOut.write();

        TimedPosture tp;
        tp.time = 0;
        tp.posture.resize(m_qRef.data.length());
        for (size_t i=0; i<tp.posture.size(); i++) tp.posture[i] = m_qRef.data[i];
        m_log.add(tp);
    }
    if ( m_use_viewer ) m_window.oneStep();
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t CollisionDetector::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/



extern "C"
{

    void CollisionDetectorInit(RTC::Manager* manager)
    {
        RTC::Properties profile(component_spec);
        manager->registerFactory(profile,
                                 RTC::Create<CollisionDetector>,
                                 RTC::Delete<CollisionDetector>);
    }

};


