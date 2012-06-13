#ifndef __PORT_HANDLER_H__
#define __PORT_HANDLER_H__

#include "BodyRTC.h"

namespace hrp{
    class ForceSensor;
    class RateGyroSensor;
    class AccelSensor;
    class RangeSensor;
    class VisionSensor;
};

class PortHandler
{
public:
    virtual void update()=0;
};

template<class T>
class InPortHandler : public PortHandler
{
public:
    InPortHandler(RTC::DataFlowComponentBase *i_rtc, const char *i_portName) : 
        m_port(i_portName, m_data){
        i_rtc->addInPort(i_portName, m_port);
    }
protected:
    T m_data;
    RTC::InPort<T> m_port;
};

template<class T>
class OutPortHandler : public PortHandler
{
public:
    OutPortHandler(RTC::DataFlowComponentBase *i_rtc, const char *i_portName) : 
        m_port(i_portName, m_data){
        i_rtc->addOutPort(i_portName, m_port);
    }
protected:
    T m_data;
    RTC::OutPort<T> m_port;
};

class JointInPortHandler : public InPortHandler<RTC::TimedDoubleSeq>
{
public:
    JointInPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                       const char *i_portName,
                       const std::vector<hrp::Link *> &i_joints);
protected:
    std::vector<hrp::Link *> m_joints;
};

class JointOutPortHandler : public OutPortHandler<RTC::TimedDoubleSeq>
{
public:
    JointOutPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                        const char *i_portName,
                        const std::vector<hrp::Link *> &i_joints);
protected:
    std::vector<hrp::Link *> m_joints;
};

class JointValueInPortHandler : public JointInPortHandler
{
public:
    JointValueInPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                            const char *i_portName,
                            const std::vector<hrp::Link *> &i_joints);
    void update();
};

class JointValueOutPortHandler : public JointOutPortHandler
{
public:
    JointValueOutPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                             const char *i_portName,
                             const std::vector<hrp::Link *> &i_joints);
    void update();
};

class JointVelocityInPortHandler : public JointInPortHandler
{
public:
    JointVelocityInPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                               const char *i_portName,
                               const std::vector<hrp::Link *> &i_joints);
    void update();
};

class JointVelocityOutPortHandler : public JointOutPortHandler
{
public:
    JointVelocityOutPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                                const char *i_portName,
                                const std::vector<hrp::Link *> &i_joints);
    void update();
};

class JointAccelerationInPortHandler : public JointInPortHandler
{
public:
    JointAccelerationInPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                                   const char *i_portName,
                                   const std::vector<hrp::Link *> &i_joints);
    void update();
};

class JointAccelerationOutPortHandler : public JointOutPortHandler
{
public:
    JointAccelerationOutPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                                    const char *i_portName,
                                    const std::vector<hrp::Link *> &i_joints);
    void update();
};

class JointTorqueInPortHandler : public JointInPortHandler
{
public:
    JointTorqueInPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                             const char *i_portName,
                             const std::vector<hrp::Link *> &i_joints);
    void update();
};

class JointTorqueOutPortHandler : public JointOutPortHandler
{
public:
    JointTorqueOutPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                              const char *i_portName,
                              const std::vector<hrp::Link *> &i_joints);
    void update();
};

template<class T, class S>
class SensorPortHandler : public OutPortHandler<S>
{
public:
    SensorPortHandler(RTC::DataFlowComponentBase *i_rtc, const char *i_portName,
                      T *i_sensor) : 
        OutPortHandler<S>(i_rtc, i_portName),
        m_sensor(i_sensor){
    }
protected:
    T *m_sensor;
};

class ForceSensorPortHandler : public SensorPortHandler<hrp::ForceSensor, RTC::TimedDoubleSeq>
{
public:
    ForceSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                           const char *i_portName,
                           hrp::ForceSensor *i_sensor);
    void update();
};

class RateGyroSensorPortHandler : public SensorPortHandler<hrp::RateGyroSensor, RTC::TimedAngularVelocity3D>
{
public:
    RateGyroSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, const char *i_portName,
                           hrp::RateGyroSensor *i_sensor);
    void update();
private:        
};

class AccelSensorPortHandler : public SensorPortHandler<hrp::AccelSensor, RTC::TimedAcceleration3D>
{
public:
    AccelSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, const char *i_portName,
                           hrp::AccelSensor *i_sensor);
    void update();
private:        
};

class RangeSensorPortHandler : public SensorPortHandler<hrp::RangeSensor, RTC::TimedDoubleSeq>
{
public:
    RangeSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, const char *i_portName,
                           hrp::RangeSensor *i_sensor);
    void update();
private:        
};

class VisionSensorPortHandler : public SensorPortHandler<hrp::VisionSensor, Img::TimedCameraImage>
{
public:
    VisionSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, const char *i_portName,
                           hrp::VisionSensor *i_sensor);
    void update();
private:        
};


#endif
