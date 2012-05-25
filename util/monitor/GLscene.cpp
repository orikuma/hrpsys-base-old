#include <cstdio>
#include <iostream>
#include <fstream>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <sys/time.h>
#include "util/GLcamera.h"
#include "util/GLlink.h"
#include "util/GLbody.h"
#include "util/LogManager.h"
#include "TimedRobotState.h"
#include "GLscene.h"

using namespace OpenHRP;
using namespace hrp;

void drawString2(const char *str)
{
    for (unsigned int i=0; i<strlen(str); i++){
        glutBitmapCharacter(GLUT_BITMAP_8_BY_13, str[i]);
    }
}

void white(){
    glColor3d(1.0,1.0,1.0);
}

void red(){
    glColor3d(1.0,0.0,0.0);
}

void yellow(){
    glColor3d(1.0,1.0,0.0);
}

void green(){
    glColor3d(0.0,1.0,0.0);
}

void blue(){
    glColor3d(0.0,0.0,1.0);
}

void black(){
    glColor3d(0.0,0.0,0.0);
}

bool isCalibrated(int s) { 
    return s & OpenHRP::RobotHardwareService::CALIB_STATE_MASK; 
}
bool isPowerOn(int s) { 
    return s & OpenHRP::RobotHardwareService::POWER_STATE_MASK; 
}
bool isServoOn(int s) { 
    return s & OpenHRP::RobotHardwareService::SERVO_STATE_MASK; 
}
int servoAlarm(int s){
    return (s & OpenHRP::RobotHardwareService::SERVO_ALARM_MASK)>>OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT; 
}
int temperature(int s){
    return (s & OpenHRP::RobotHardwareService::DRIVER_TEMP_MASK)>>OpenHRP::RobotHardwareService::DRIVER_TEMP_SHIFT; 
}

void GLscene::updateScene()
{
    if (m_log->index()<0) return;

    LogManager<TimedRobotState> *lm 
        = (LogManager<TimedRobotState> *)m_log;
    OpenHRP::StateHolderService::Command &com = lm->state().command;
    if (com.jointRefs.length()){
        GLbody *glbody = dynamic_cast<GLbody *>(body(0).get());
        double *tform = com.baseTransform.get_buffer();
        glbody->setPosition(tform);
        glbody->setRotation(tform+3);
        glbody->setPosture(com.jointRefs.get_buffer());
    }
}

void GLscene::showStatus()
{
    if (m_log->index()<0) return;

    LogManager<TimedRobotState> *lm 
        = (LogManager<TimedRobotState> *)m_log;
    OpenHRP::RobotHardwareService::RobotState &rstate = lm->state().state;

    if (m_showingStatus){
        GLbody *glbody = dynamic_cast<GLbody *>(body(0).get());
#define HEIGHT_STEP 12
        int width = m_width - 410;
        int height = m_height-HEIGHT_STEP;
        char buf[256];
        for (int i=0; i<glbody->numJoints(); i++){
            hrp::Link *l = glbody->joint(i);
            if (l){
                int ss = rstate.servoState[i];
                int x = width;
                // joint ID
                sprintf(buf, "%2d",i);
                if (!isCalibrated(ss)){
                    yellow();
                }else if(isServoOn(ss)){
                    red();
                }
                glRasterPos2f(x, height);
                drawString2(buf);
                white();
                x += 8*3;
                // power status
                if (isPowerOn(ss)) blue();
                glRasterPos2f(x, height);
                drawString2("o");
                if (isPowerOn(ss)) white();
                x += 8*2;
                // joint name, current angle, command angle and torque
                sprintf(buf, "%13s %8.3f %8.3f %6.1f", 
                        l->name.c_str(), 
                        rstate.angle[i]*180/M_PI,
                        rstate.command[i]*180/M_PI,
                        rstate.torque[i]*180/M_PI);
                glRasterPos2f(x, height);
                drawString2(buf);
                x += 8*(14+9+9+7);
                // servo alarms
                sprintf(buf, "%03x", servoAlarm(ss));
                glRasterPos2f(x, height);
                drawString2(buf);
                x += 8*4;
                // driver temperature
                int temp = temperature(ss);
                if (!temp){
                    sprintf(buf, "--", temp);
                }else{
                    sprintf(buf, "%2d", temp);
                }
                if (temp >= 60) red();
                glRasterPos2f(x, height);
                drawString2(buf);
                if (temp >= 60) white();
                x += 8*3;
                
                height -= HEIGHT_STEP;
            }
        }
        if (rstate.accel.length()){
            glRasterPos2f(width, height);
            height -= HEIGHT_STEP;
            drawString2("acc:");
            for (unsigned int i=0; i<rstate.accel.length(); i++){
                sprintf(buf, "  %8.4f %8.4f %8.4f",
                        rstate.accel[i][0], rstate.accel[i][1], rstate.accel[i][2]);
                glRasterPos2f(width, height);
                height -= HEIGHT_STEP;
                drawString2(buf);
            }
        }
        if (rstate.rateGyro.length()){
            glRasterPos2f(width, height);
            height -= HEIGHT_STEP;
            drawString2("rate:");
            for (unsigned int i=0; i<rstate.rateGyro.length(); i++){
                sprintf(buf, "  %8.4f %8.4f %8.4f",
                        rstate.rateGyro[i][0], rstate.rateGyro[i][1], rstate.rateGyro[i][2]);
                glRasterPos2f(width, height);
                height -= HEIGHT_STEP;
                drawString2(buf);
            }
        }
        if (rstate.force.length()){
            glRasterPos2f(width, height);
            height -= HEIGHT_STEP;
            drawString2("force/torque:");
            for (unsigned int i=0; i<rstate.force.length(); i++){
                sprintf(buf, "  %6.1f %6.1f %6.1f %6.2f %6.2f %6.2f",
                        rstate.force[i][0], 
                        rstate.force[i][1], 
                        rstate.force[i][2],
                        rstate.force[i][3], 
                        rstate.force[i][4], 
                        rstate.force[i][5]);
                glRasterPos2f(width, height);
                height -= HEIGHT_STEP;
                drawString2(buf);
            }
        }
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_BLEND);
        glColor4f(0.0,0.0,0.0, 0.5);
        if (m_showSlider){
            glRectf(width,SLIDER_AREA_HEIGHT,m_width,m_height);
        }else{
            glRectf(width,0,m_width,m_height);
        }
        glDisable(GL_BLEND);
    }else{
        // !m_showingRobotState
        bool servo=false, power=false;
        for (unsigned int i=0; i<rstate.servoState.length(); i++){
            if (isServoOn(rstate.servoState[i])) servo = true;
            if (isPowerOn(rstate.servoState[i])) power = true;
        }
        struct timeval tv;
        gettimeofday(&tv, NULL);
        double dt = tv.tv_sec + tv.tv_usec/1e6 - lm->time(m_log->length()-1);
        if (dt < 1.0) green(); else black();
        glRectf(m_width-115,m_height-45,m_width-85,m_height-15);
        if (power) blue(); else black();
        glRectf(m_width- 80,m_height-45,m_width-50,m_height-15);
        if (servo) red(); else black();
        glRectf(m_width- 45,m_height-45,m_width-15,m_height-15);
    }
}

void printMatrix(double mat[16])
{
    for (int i=0; i<4; i++){
        for (int j=0; j<4; j++){
            printf("%6.3f ", mat[j*4+i]);
        }
        printf("\n");
    }
}

void GLscene::drawAdditionalLines()
{
    if (m_log->index()<0) return;

    LogManager<TimedRobotState> *lm 
        = (LogManager<TimedRobotState> *)m_log;
    OpenHRP::StateHolderService::Command &com = lm->state().command;

    if (com.zmp.length() != 3) return;

    Vector3 relZmp(com.zmp[0], com.zmp[1], com.zmp[2]);
    Vector3 rootP(com.baseTransform[0], com.baseTransform[1], com.baseTransform[2]);
    Matrix33 rootR;
    rootR << com.baseTransform[3], com.baseTransform[4], com.baseTransform[5], 
        com.baseTransform[6], com.baseTransform[7], com.baseTransform[8], 
        com.baseTransform[9], com.baseTransform[10], com.baseTransform[11]; 
    Vector3 absZmp = rootR*relZmp + rootP;
    float v[3];
    glColor3f(1,0,1);
    glBegin(GL_LINES);
#define LINE_HLEN 0.5
    v[0] = absZmp[0]+LINE_HLEN; v[1] = absZmp[1]; v[2] = absZmp[2]+0.001;
    glVertex3fv(v);
    v[0] = absZmp[0]-LINE_HLEN;
    glVertex3fv(v);
    v[0] = absZmp[0]; v[1] = absZmp[1]+LINE_HLEN; v[2] = absZmp[2]+0.001;
    glVertex3fv(v);
    v[1] = absZmp[1]-LINE_HLEN;
    glVertex3fv(v);
    v[0] = absZmp[0]; v[1] = absZmp[1]; v[2] = absZmp[2]+0.001;
    glVertex3fv(v);
    v[2] = absZmp[2]+LINE_HLEN;
    glVertex3fv(v);
    glEnd();
}
