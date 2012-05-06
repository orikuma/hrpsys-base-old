#include <boost/python.hpp>
#include "util/BodyRTC.h"

class PyLink;
class PySimulator;

class PyBody : public BodyRTC
{
public:
    enum{STRUCTURE, KINEMATICS}; 

    PyBody(RTC::Manager* manager = &RTC::Manager::instance());
    virtual ~PyBody();
    //std::vector<double> getPosition();
    PyObject *getPosition();
    void setPosition(PyObject *v);
    PyObject *getRotation();
    void setRotation(PyObject *v);
    PyObject *getPosture();
    void setPosture(PyObject *v);
    std::string getName();
    void setName(std::string name);
    void calcForwardKinematics();
    PyLink *rootLink();
    PyLink *link(std::string name);
    PyObject *links();
    PyLink *joint(int i);
    PyObject *joints();
    PyObject *calcCM();
    void notifyChanged(int change);
    void setListener(PySimulator *i_sim);
    static void moduleInit(RTC::Manager*);
private:
    static const char* pybody_spec[];
    PySimulator *simulator;
};

