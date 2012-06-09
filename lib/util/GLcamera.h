#ifndef __GLCAMERA_H__
#define __GLCAMERA_H__

#include <string>
#include <vector>
#include <hrpCorba/ModelLoader.hh>
#include "GLcoordinates.h"

class GLsceneBase;
class GLlink;
class GLshape;

class GLcamera : public GLcoordinates
{
public:
    GLcamera(const OpenHRP::SensorInfo &i_si, OpenHRP::ShapeSetInfo_ptr i_ssinfo,
             GLlink *i_link);
    GLcamera(int i_width, int i_height, double i_near, double i_far, double i_fovy);
    ~GLcamera();
    const std::string& name() const;
    void setView();
    void computeAbsTransform(double o_trans[16]);
    double *getAbsTransform();
    double near() { return m_near; }
    double far() { return m_far; }
    double fovy() { return m_fovy; }
    int width() { return m_width; }
    int height() { return m_height; }
    void setViewPoint(double x, double y, double z);
    void setViewTarget(double x, double y, double z);
    void draw(int i_mode);
    GLlink *link();
    void highlight(bool flag);
    void render(GLsceneBase *i_scene);
    unsigned char *rgbImage();
private:
    void initFramebuffer( void );
    void initRenderbuffer( void );
    void initTexture( void );
    std::string m_name;
    double m_absTrans[16];
    GLlink *m_link;
    double m_near, m_far, m_fovy;
    int m_width, m_height;
    double m_viewPoint[3], m_viewTarget[3];
    std::vector<GLshape *> m_shapes;
    GLuint m_frameBuffer, m_renderBuffer, m_texture;
    unsigned char *m_rgbImage;
};

#endif
