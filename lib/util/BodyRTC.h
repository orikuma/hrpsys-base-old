﻿#ifndef BODY_EXT_H_INCLUDED
#define BODY_EXT_H_INCLUDED

#include <hrpModel/Body.h>
#include <hrpCorba/OpenHRPCommon.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include "Img.hh"

class PortHandler;

class BodyRTC : virtual public hrp::Body, public RTC::DataFlowComponentBase
{
public:
    BodyRTC(RTC::Manager* manager = &RTC::Manager::instance());
    virtual ~BodyRTC(void);

    RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id){
        std::cout << "BodyRTC::onActivated(" << ec_id << ")" << std::endl;
        return RTC::RTC_OK;
    }
    RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id){
        std::cout << "BodyRTC::onDeactivated(" << ec_id << ")" << std::endl;
        return RTC::RTC_OK;
    }

    void createDataPorts();
    void createInPort(const std::string &config);
    void createOutPort(const std::string &config);
    void writeDataPorts();
    void readDataPorts();
    static void moduleInit(RTC::Manager*);

private:
    static const char* bodyrtc_spec[];

    // DataInPort
    RTC::TimedDoubleSeq m_tau;
    RTC::TimedDoubleSeq m_qRef, m_dqRef, m_ddqRef;
    RTC::InPort<RTC::TimedDoubleSeq> m_tauIn;
    RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn, m_dqRefIn, m_ddqRefIn;

    // DataOutPort
    RTC::TimedDoubleSeq m_q, m_dq;
    RTC::TimedPose3D m_basePose;
    std::vector<RTC::TimedAcceleration3D> m_acc;
    std::vector<RTC::TimedAngularVelocity3D> m_rate;
    std::vector<RTC::TimedDoubleSeq> m_force;
    std::vector<RTC::TimedDoubleSeq> m_range;
    std::vector<Img::TimedCameraImage> m_image;

    RTC::OutPort<RTC::TimedDoubleSeq> m_qOut, m_dqOut;
    RTC::OutPort<RTC::TimedPose3D> m_basePoseOut;
    std::vector<RTC::OutPort<RTC::TimedAcceleration3D> *> m_accOut;
    std::vector<RTC::OutPort<RTC::TimedAngularVelocity3D> *> m_rateOut;
    std::vector<RTC::OutPort<RTC::TimedDoubleSeq> *> m_forceOut;
    std::vector<RTC::OutPort<RTC::TimedDoubleSeq> *> m_rangeOut;
    std::vector<RTC::OutPort<Img::TimedCameraImage> *> m_imageOut;

    std::vector<PortHandler *> m_inports;
    std::vector<PortHandler *> m_outports;
    int dummy;
};

typedef boost::intrusive_ptr<BodyRTC> BodyRTCPtr;

#endif
