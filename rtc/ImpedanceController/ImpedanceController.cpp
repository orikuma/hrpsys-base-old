// -*- C++ -*-
/*!
 * @file  ImpedanceController.cpp
 * @brief impedance controller component
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "ImpedanceController.h"
#include "JointPathEx.h"
#include <hrpModel/JointPath.h>
#include <hrpUtil/MatrixSolvers.h>


typedef coil::Guard<coil::Mutex> Guard;

// Module specification
// <rtc-template block="module_spec">
static const char* impedancecontroller_spec[] =
    {
        "implementation_id", "ImpedanceController",
        "type_name",         "ImpedanceController",
        "description",       "impedance controller component",
        "version",           "1.0",
        "vendor",            "AIST",
        "category",          "example",
        "activity_type",     "DataFlowComponent",
        "max_instance",      "10",
        "language",          "C++",
        "lang_type",         "compile",
        // Configuration variables

        ""
    };
// </rtc-template>

ImpedanceController::ImpedanceController(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_qCurrentIn("qCurrent", m_qCurrent),
      m_qRefIn("qRef", m_qRef),
      m_qOut("q", m_q),
      m_ImpedanceControllerServicePort("ImpedanceControllerService"),
      // </rtc-template>
      m_robot(NULL),
      dummy(0)
{
    m_service0.impedance(this);
}

ImpedanceController::~ImpedanceController()
{
}


RTC::ReturnCode_t ImpedanceController::onInitialize()
{
    std::cout << "ImpedanceController::onInitialize()" << std::endl;
    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("qCurrent", m_qCurrentIn);
    addInPort("qRef", m_qRefIn);

    // Set OutPort buffer
    addOutPort("q", m_qOut);
  
    // Set service provider to Ports
    m_ImpedanceControllerServicePort.registerProvider("service0", "ImpedanceControllerService", m_service0);
  
    // Set service consumers to Ports
  
    // Set CORBA Service Ports
    addPort(m_ImpedanceControllerServicePort);
  
    // </rtc-template>
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
  
    // </rtc-template>

    RTC::Properties& prop = getProperties();
    coil::stringTo(m_dt, prop["dt"].c_str());

    m_robot = new hrp::Body();

    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
    if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(), 
                                 CosNaming::NamingContext::_duplicate(naming.getRootContext())
                                 )){
        std::cerr << "failed to load model[" << prop["model"] << "]" 
                  << std::endl;
    }

    int nforce = m_robot->numSensors(hrp::Sensor::FORCE);
    m_force.resize(nforce);
    m_forceIn.resize(nforce);
    for (unsigned int i=0; i<m_force.size(); i++){
        hrp::Sensor *s = m_robot->sensor(hrp::Sensor::FORCE, i);
        m_forceIn[i] = new InPort<TimedDoubleSeq>(s->name.c_str(), m_force[i]);
        m_force[i].data.length(6);
        registerInPort(s->name.c_str(), *m_forceIn[i]);
    }

    unsigned int dof = m_robot->numJoints();
    for ( int i = 0 ; i < dof; i++ ){
      if ( i != m_robot->joint(i)->jointId ) {
        std::cerr << "jointId is not equal to the index number" << std::endl;
        return RTC::RTC_ERROR;
      }
    }

    // allocate memory for outPorts
    m_q.data.length(dof);

    return RTC::RTC_OK;
}



RTC::ReturnCode_t ImpedanceController::onFinalize()
{
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t ImpedanceController::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ImpedanceController::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t ImpedanceController::onActivated(RTC::UniqueId ec_id)
{
    std::cout << "ImpedanceController::onActivated(" << ec_id << ")" << std::endl;
    
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t ImpedanceController::onDeactivated(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

//#define DEBUG true
#define DEBUG false

RTC::ReturnCode_t ImpedanceController::onExecute(RTC::UniqueId ec_id)
{
    //std::cout << "ImpedanceController::onExecute(" << ec_id << ")" << std::endl;
    for (unsigned int i=0; i<m_forceIn.size(); i++){
        if ( m_forceIn[i]->isNew() ) {
            m_forceIn[i]->read();
        }
    }

    if (m_qCurrentIn.isNew()) {
        m_qCurrentIn.read();
    }
    if (m_qRefIn.isNew()) {
        m_qRefIn.read();
    }
    if ( m_qRef.data.length() ==  m_robot->numJoints() ) {

        if ( DEBUG ) {
            std::cerr << "qRef  : ";
            for ( int i = 0; i <  m_qRef.data.length(); i++ ){
                std::cerr << " " << m_qRef.data[i];
            }
            std::cerr << std::endl;
        }

        if ( m_impedance_param.size() == 0 ) {
          for ( int i = 0; i < m_qRef.data.length(); i++ ){
            m_q.data[i] = m_qRef.data[i];
          }
          m_qOut.write();
          return RTC_OK;
        }

        Guard guard(m_mutex);

	{
	  hrp::dvector qorg(m_robot->numJoints());
	  
	  // reference model
	  for ( int i = 0; i < m_robot->numJoints(); i++ ){
	    qorg[i] = m_robot->joint(i)->q;
            m_robot->joint(i)->q = m_qRef.data[i];
	  }
	  m_robot->calcForwardKinematics();

	  // set sequencer position to target_p0
	  for ( std::map<std::string, ImpedanceParam>::iterator it = m_impedance_param.begin(); it != m_impedance_param.end(); it++ ) {
            ImpedanceParam& param = it->second;
            param.target_p0 = m_robot->link(param.target_name)->p;
            param.target_r0 = hrp::omegaFromRot(m_robot->link(param.target_name)->R);
          }
          // back to impedance robot model (only for controlled joint)
	  for ( std::map<std::string, ImpedanceParam>::iterator it = m_impedance_param.begin(); it != m_impedance_param.end(); it++ ) {
            ImpedanceParam& param = it->second;
            for ( int j = 0; j < param.manip->numJoints(); j++ ){
              int i = param.manip->joint(j)->jointId;
              m_robot->joint(i)->q = qorg[i];
            }
	  }
	  m_robot->calcForwardKinematics();

	}

	// set m_robot to qRef when deleting status
	for ( std::map<std::string, ImpedanceParam>::iterator it = m_impedance_param.begin(); it != m_impedance_param.end(); it++ ) {
	  std::string sensor_name = it->first;
	  ImpedanceParam& param = it->second;

	  if ( param.transition_count > 0 ) {
	    hrp::JointPathExPtr manip = param.manip;
	    for ( int j = 0; j < manip->numJoints(); j++ ) {
              int i = manip->joint(j)->jointId; // index in robot model
	      hrp::Link* joint =  m_robot->joint(i);
              joint->q = ( m_qRef.data[i] - joint->q ) * ( 1.0 / param.transition_count ) + joint->q;
	    }
	    param.transition_count --;

	  } else {
	    // use impedance model

            hrp::Link* base = m_robot->link(param.base_name);
            hrp::Link* target = m_robot->link(param.target_name);
            assert(target);
            assert(base);

            param.current_p0 = target->p;
            param.current_r0 = hrp::omegaFromRot(target->R);

            hrp::JointPathExPtr manip = param.manip;
            assert(manip);
            const int n = manip->numJoints();

            // force
            hrp::Vector3 force_p, force_r;
            for (unsigned int i=0; i<m_forceIn.size(); i++){
                if ( std::string(m_forceIn[i]->name()) == sensor_name ) {
                    assert(m_force[i].data.length()==6);
                    hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(sensor_name);
                    hrp::Vector3 data_p(m_force[i].data[0], m_force[i].data[1], m_force[i].data[2]);
                    hrp::Vector3 data_r(m_force[i].data[3], m_force[i].data[4], m_force[i].data[5]);
                    force_p = sensor->link->R * sensor->localR * (data_p - param.force_offset_p);
                    force_r = sensor->link->R * sensor->localR * (data_r - param.force_offset_r);
                }
            }
            if ( DEBUG ) {
                std::cerr << "force : " << force_p[0] << " " << force_p[1] << " " << force_p[2] << std::endl;
                std::cerr << "force : " << force_r[0] << " " << force_r[1] << " " << force_r[2] << std::endl;
            }

            hrp::Vector3 dif_pos = hrp::Vector3(0,0,0);
            hrp::Vector3 vel_pos0 = hrp::Vector3(0,0,0);
            hrp::Vector3 vel_pos1 = hrp::Vector3(0,0,0);
            hrp::Vector3 dif_target_pos = hrp::Vector3(0,0,0);
            hrp::Vector3 dif_rot = hrp::Vector3(0,0,0);
            hrp::Vector3 vel_rot0 = hrp::Vector3(0,0,0);
            hrp::Vector3 vel_rot1 = hrp::Vector3(0,0,0);
            hrp::Vector3 dif_target_rot = hrp::Vector3(0,0,0);

            // rats/plugins/impedancecontrol.cpp
            //double M = 5, D = 100, K = 200;
            // dif_pos  = target_p0 (target_coords0) - current_p0(move_coords)
            // vel_pos0 = current_p0(move_coors) - current_p1(prev_coords0)
            // vel_pos1 = current_p1(prev_coords0) - current_p2(prev_coords1)
            // dif_target  = target_p0(target_coords0) - target_p1(target_coords1)
            //
            // current_p2(prev_coords1) = current_p1(prev_coords0)
            // currnet_p1(prev_coords0) = current_p0(move_coords) + vel_p
            // target_p1(target_coords1) = target_p0(target_coords0)

            if ( DEBUG ) {
                std::cerr << "cur0  : " << param.current_p0[0] << " " << param.current_p0[1] << " " << param.current_p0[2] << std::endl;
                std::cerr << "cur1  : " << param.current_p1[0] << " " << param.current_p1[1] << " " << param.current_p1[2] << std::endl;
                std::cerr << "cur2  : " << param.current_p2[0] << " " << param.current_p2[1] << " " << param.current_p2[2] << std::endl;
                std::cerr << "tgt0  : " << param.target_p0[0] << " " << param.target_p0[1] << " " << param.target_p0[2] << std::endl;
                std::cerr << "tgt1  : " << param.target_p1[0] << " " << param.target_p1[1] << " " << param.target_p1[2] << std::endl;
            }
            if ( DEBUG ) {
                std::cerr << "cur0  : " << param.current_r0[0] << " " << param.current_r0[1] << " " << param.current_r0[2] << std::endl;
                std::cerr << "cur1  : " << param.current_r1[0] << " " << param.current_r1[1] << " " << param.current_r1[2] << std::endl;
                std::cerr << "cur2  : " << param.current_r2[0] << " " << param.current_r2[1] << " " << param.current_r2[2] << std::endl;
                std::cerr << "tgt0  : " << param.target_r0[0] << " " << param.target_r0[1] << " " << param.target_r0[2] << std::endl;
                std::cerr << "tgt1  : " << param.target_r1[0] << " " << param.target_r1[1] << " " << param.target_r1[2] << std::endl;
            }

            dif_pos  = param.target_p0 - param.current_p0;
            vel_pos0 = param.current_p0 - param.current_p1;
            vel_pos1 = param.current_p1 - param.current_p2;
            dif_target_pos = param.target_p0 - param.target_p1;

            dif_rot  = param.target_r0 - param.current_r0;
            vel_rot0 = param.current_r0 - param.current_r1;
            vel_rot1 = param.current_r1 - param.current_r2;
            dif_target_rot = param.target_r0 - param.target_r1;

            if ( DEBUG ) {
                std::cerr << "dif_p : " << dif_pos[0] << " " << dif_pos[1] << " " << dif_pos[2] << std::endl;
                std::cerr << "vel_p0: " << vel_pos0[0] << " " << vel_pos0[1] << " " << vel_pos0[2] << std::endl;
                std::cerr << "vel_p1: " << vel_pos1[0] << " " << vel_pos1[1] << " " << vel_pos1[2] << std::endl;
                std::cerr << "dif_t : " << dif_target_pos[0] << " " << dif_target_pos[1] << " " << dif_target_pos[2] << std::endl;
            }
            if ( DEBUG ) {
                std::cerr << "dif_r : " << dif_rot[0] << " " << dif_rot[1] << " " << dif_rot[2] << std::endl;
                std::cerr << "vel_r0: " << vel_rot0[0] << " " << vel_rot0[1] << " " << vel_rot0[2] << std::endl;
                std::cerr << "vel_r1: " << vel_rot1[0] << " " << vel_rot1[1] << " " << vel_rot1[2] << std::endl;
                std::cerr << "dif_t : " << dif_target_rot[0] << " " << dif_target_rot[1] << " " << dif_target_rot[2] << std::endl;
            }
            hrp::Vector3 vel_p, vel_r;
            //std::cerr << "MDK = " << param.M_p << " " << param.D_p << " " << param.K_p << std::endl;
            //std::cerr << "MDK = " << param.M_r << " " << param.D_r << " " << param.K_r << std::endl;
	    vel_p = ( force_p * m_dt * m_dt
                      + param.M_p * ( vel_pos1 - vel_pos0 )
                      + param.D_p * ( dif_target_pos - vel_pos0 ) * m_dt
                      + param.K_p * ( dif_pos * m_dt * m_dt  ) ) /
                (param.M_p + (param.D_p * m_dt) + (param.K_p * m_dt * m_dt));

	    vel_r = ( force_r * m_dt * m_dt
                      + param.M_r * ( vel_rot1 - vel_rot0 )
                      + param.D_r * ( dif_target_rot - vel_rot0 ) * m_dt
                      + param.K_r * ( dif_rot * m_dt * m_dt  ) ) /
                (param.M_r + (param.D_r * m_dt) + (param.K_r * m_dt * m_dt));

            if ( DEBUG ) {
                std::cerr << "vel_p : " << vel_p[0] << " " << vel_p[1] << " " << vel_p[2] << std::endl;
                std::cerr << "vel_r : " << vel_r[0] << " " << vel_r[1] << " " << vel_r[2] << std::endl;
            }

#if 0
            manip->setBestEffortIKMode(true);
            manip->setMaxIKError(0.001);
            manip->calcInverseKinematics2(param.current_p0 + vel_p, target->R);
#else
	    hrp::dmatrix J(6, n);
	    hrp::dmatrix Jinv(n, 6);
	    hrp::dmatrix Jnull(n, n);
	    
	    manip->setSRGain(param.sr_gain);
	    manip->setManipulabilityLimit(param.manipulability_limit);
	    manip->calcJacobianInverseNullspace(J, Jinv, Jnull);
	    //manip->calcInverseKinematics2Loop(vel_p, vel_r, dq);

	    hrp::dvector v(6);
	    v << vel_p, vel_r;
	    hrp::dvector dq(n);
	    dq = Jinv * v; // dq = pseudoInverse(J) * v

	    // dq = J#t a dx + ( I - J# J ) Jt b dx
	    // avoid-nspace-joint-limit: avoiding joint angle limit
	    //
	    // dH/dq = (((t_max + t_min)/2 - t) / ((t_max - t_min)/2)) ^2
	    hrp::dvector u(n);
	    for(int j=0; j < n; ++j) { u[j] = 0; }
	    for ( int j = 0; j < n ; j++ ) {
	      double jang = manip->joint(j)->q;
	      double jmax = manip->joint(j)->ulimit;
	      double jmin = manip->joint(j)->llimit;
	      double r = ((( (jmax + jmin) / 2.0) - jang) / ((jmax - jmin) / 2.0));
	      if ( r > 0 ) { r = r*r; } else { r = - r*r; }
	      u[j] += r;
	    }
	    if ( DEBUG ) {
	      std::cerr << "    u : " << u;
	      std::cerr << "  dqb : " << Jnull * u;
	    }
	    dq = dq + Jnull * ( param.avoid_gain *  u );
	    //
	    // qref - qcurr
	    for(int j=0; j < n; ++j) { u[j] = 0; }
	    for ( int j = 0; j < manip->numJoints(); j++ ) {
              u[j] = ( m_qRef.data[manip->joint(j)->jointId] - manip->joint(j)->q );
	    }
	    dq = dq + Jnull * ( param.reference_gain *  u );

            // break if dq(j) is nan/nil
            bool dq_check = true;
            for(int j=0; j < n; ++j){
                if ( isnan(dq(j)) || isinf(dq(j)) ) {
                    dq_check = false;
                    break;
                }
            }
            if ( ! dq_check ) break;

            // check max speed
            double max_speed = 0;
            for(int j=0; j < n; ++j){
                max_speed = std::max(max_speed, fabs(dq(j)));
            }
	    if ( max_speed > 0.2*0.5 ) { // 0.5 safety margin
                if ( DEBUG ) {
                    std::cerr << "spdlmt: ";
                    for(int j=0; j < n; ++j) { std::cerr << dq(j) << " "; } std::cerr << std::endl;
                }
                for(int j=0; j < n; ++j) {
                    dq(j) = dq(j) * 0.2*0.5 / max_speed;
                }
                if ( DEBUG ) {
                    std::cerr << "spdlmt: ";
                    for(int j=0; j < n; ++j) { std::cerr << dq(j) << " "; } std::cerr << std::endl;
                }
            }

            // update robot model
            for(int j=0; j < n; ++j){
                manip->joint(j)->q += dq(j);
            }


            // check limit
            for(int j=0; j < n; ++j){
                if ( manip->joint(j)->q > manip->joint(j)->ulimit) {
                    std::cerr << "Upper joint limit error " << manip->joint(j)->name << std::endl;
                    manip->joint(j)->q = manip->joint(j)->ulimit;
                }
                if ( manip->joint(j)->q < manip->joint(j)->llimit) {
                    std::cerr << "Lower joint limit error " << manip->joint(j)->name << std::endl;
                    manip->joint(j)->q = manip->joint(j)->llimit;
                }
                manip->joint(j)->q = std::max(manip->joint(j)->q, manip->joint(j)->llimit);
            }
#endif

            manip->calcForwardKinematics();

	    param.current_p2 = param.current_p1;
	    param.current_p1 = param.current_p0 + vel_p;
	    param.target_p1 = param.target_p0;

	    param.current_r2 = param.current_r1;
	    param.current_r1 = param.current_r0 + vel_r;
	    param.target_r1 = param.target_r0;

            // generate smooth motion just after mpedance started
            for(int j=0; j < n; ++j){
              int i = manip->joint(j)->jointId;
              if ( param.transition_count < 0 ) {
                manip->joint(j)->q = ( manip->joint(j)->q  - m_qCurrent.data[i] ) * (-1.0 / param.transition_count) + m_qCurrent.data[i];
                param.transition_count++;
              }
            }

	  } // else
        } // for

        if ( m_q.data.length() != 0 ) { // initialized
            for ( int i = 0; i < m_robot->numJoints(); i++ ){
                m_q.data[i] = m_robot->joint(i)->q;
            }
            m_qOut.write();
            if ( DEBUG ) {
                std::cerr << "q     : ";
                for ( int i = 0; i < m_q.data.length(); i++ ){
                    std::cerr << " " << m_q.data[i];
                }
                std::cerr << std::endl;
            }

        }
    }
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t ImpedanceController::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ImpedanceController::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ImpedanceController::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ImpedanceController::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ImpedanceController::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

//

bool ImpedanceController::setImpedanceControllerParam(OpenHRP::ImpedanceControllerService::impedanceParam i_param_)
{
    Guard guard(m_mutex);

    std::string name = std::string(i_param_.name);
    std::string base_name = std::string(i_param_.base_name);
    std::string target_name = std::string(i_param_.target_name);
    
    // error check
    int force_id = -1;
    bool updateparam_flag = false;
    for (unsigned int i=0; i<m_forceIn.size(); i++){
      if ( std::string(m_forceIn[i]->name()) == name ) {
        force_id = i;
        break;
      }
    }
    if ( force_id < 0 ) {
        std::cerr << "Could not found FORCE_SENSOR named " << name << std::endl;
        return false;
    }

    if ( ! ( m_qRef.data.length() ==  m_robot->numJoints() ) ) {
        std::cerr << "Could not found qRef." << std::endl;
        return false;
    }

    //
    if ( m_impedance_param.find(name) == m_impedance_param.end() ) {
        std::cerr << "Set new impedance parameters" << std::endl;

	if ( ! m_robot->link(base_name) ) {
	  std::cerr << "Could not found link " << base_name << std::endl;
	  return false;
	}
	if ( ! m_robot->link(target_name) ) {
	  std::cerr << "Could not found link " << target_name << std::endl;
	  return false;
	}

	// set param
	ImpedanceParam p;
	p.base_name = base_name;
	p.target_name = target_name;
	p.M_p = i_param_.M_p;
	p.D_p = i_param_.D_p;
	p.K_p = i_param_.K_p;
	p.M_r = i_param_.M_r;
	p.D_r = i_param_.D_r;
	p.K_r = i_param_.K_r;

	p.target_p0 = m_robot->link(p.target_name)->p;
	p.target_p1 = m_robot->link(p.target_name)->p;
	p.target_r0 = hrp::omegaFromRot(m_robot->link(p.target_name)->R);
	p.target_r1 = hrp::omegaFromRot(m_robot->link(p.target_name)->R);

	p.current_p0 = m_robot->link(p.target_name)->p;
	p.current_p1 = m_robot->link(p.target_name)->p;
	p.current_p2 = m_robot->link(p.target_name)->p;
	p.current_r0 = hrp::omegaFromRot(m_robot->link(p.target_name)->R);
	p.current_r1 = hrp::omegaFromRot(m_robot->link(p.target_name)->R);
	p.current_r2 = hrp::omegaFromRot(m_robot->link(p.target_name)->R);

	p.force_offset_p = hrp::Vector3(m_force[force_id].data[0], m_force[force_id].data[1], m_force[force_id].data[2]);
	p.force_offset_r = hrp::Vector3(m_force[force_id].data[3], m_force[force_id].data[4], m_force[force_id].data[5]);

	// joint path
	p.manip = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->link(p.base_name), m_robot->link(p.target_name)));

        if ( ! p.manip ) {
          std::cerr << "invalid joint path from " << p.base_name << " to " << p.target_name << std::endl;
          return false;
        }

	// update reference model
        for (int i = 0; i < m_robot->numJoints(); i++ ) {
          // if other controller is already taken the joint, do not update the reference model
          bool update = true;
          for ( std::map<std::string, ImpedanceParam>::iterator it = m_impedance_param.begin(); it != m_impedance_param.end(); it++ ) {
            ImpedanceParam& param = it->second;
            for ( int j = 0; j < param.manip->numJoints(); j++ ){
              if ( i == param.manip->joint(j)->jointId ) update = false;
            }
          }
          if ( update ) m_robot->joint(i)->q = m_qCurrent.data[i];
        }
	m_robot->calcForwardKinematics();

	m_impedance_param[name] = p;

    } else {
        std::cerr << "Update impedance parameters" << std::endl;
    }

    m_impedance_param[name].sr_gain    = i_param_.sr_gain;
    m_impedance_param[name].avoid_gain = i_param_.avoid_gain;
    m_impedance_param[name].reference_gain = i_param_.reference_gain;
    m_impedance_param[name].manipulability_limit = i_param_.manipulability_limit;
    m_impedance_param[name].transition_count = -2/m_dt; // when start impedance, count up to 0

    m_impedance_param[name].M_p = i_param_.M_p;
    m_impedance_param[name].D_p = i_param_.D_p;
    m_impedance_param[name].K_p = i_param_.K_p;
    m_impedance_param[name].M_r = i_param_.M_r;
    m_impedance_param[name].D_r = i_param_.D_r;
    m_impedance_param[name].K_r = i_param_.K_r;

    for ( std::map<std::string, ImpedanceParam>::iterator it = m_impedance_param.begin(); it != m_impedance_param.end(); it++ ) {
      ImpedanceParam& param = it->second;
      std::cerr << "          name : " << it->first << std::endl;
      std::cerr << "     base_name : " << param.base_name << std::endl;
      std::cerr << "   target_name : " << param.target_name << std::endl;
      std::cerr << " K, D, M (pos) : " << param.K_p << " " << param.D_p << " " << param.M_p << std::endl;
      std::cerr << " K, D, M (rot) : " << param.K_r << " " << param.D_r << " " << param.M_r << std::endl;
      std::cerr << "   manip_limit : " << param.manipulability_limit << std::endl;
      std::cerr << "       sr_gain : " << param.sr_gain << std::endl;
      std::cerr << "    avoid_gain : " << param.avoid_gain << std::endl;
      std::cerr << "reference_gain : " << param.reference_gain << std::endl;
    }

    
    return true;
}

bool ImpedanceController::deleteImpedanceController(std::string i_name_)
{
    if ( m_impedance_param.find(i_name_) == m_impedance_param.end() ) {
      std::cerr << "Could not found impedance controller" << i_name_ << std::endl;
      return false;
    }
     
    std::cerr << "Delete impedance parameters " << i_name_ << std::endl;
    m_impedance_param[i_name_].transition_count = 2/m_dt; // when stop impedance, count down to 0

    // wait for transition count
    while ( m_impedance_param[i_name_].transition_count > 0) {
      usleep(10);
    }

    Guard guard(m_mutex);
    m_impedance_param.erase(i_name_);

    return true;
}

extern "C"
{

    void ImpedanceControllerInit(RTC::Manager* manager)
    {
        RTC::Properties profile(impedancecontroller_spec);
        manager->registerFactory(profile,
                                 RTC::Create<ImpedanceController>,
                                 RTC::Delete<ImpedanceController>);
    }

};


