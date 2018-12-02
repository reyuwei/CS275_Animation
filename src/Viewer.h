//
// Created by stephan-lb on 22/03/2017.
//

#ifndef TINYOBJVIEWER_VIEWER_H
#define TINYOBJVIEWER_VIEWER_H

#include <nanogui/opengl.h>
#include <nanogui/glutil.h>
#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/checkbox.h>
#include <nanogui/button.h>
#include <nanogui/toolbutton.h>
#include <nanogui/popupbutton.h>
#include <nanogui/combobox.h>
#include <nanogui/progressbar.h>
#include <nanogui/entypo.h>
#include <nanogui/messagedialog.h>
#include <nanogui/textbox.h>
#include <nanogui/slider.h>
#include <nanogui/imagepanel.h>
#include <nanogui/imageview.h>
#include <nanogui/vscrollpanel.h>
#include <nanogui/colorwheel.h>
#include <nanogui/graph.h>
#include <nanogui/tabwidget.h>
#include <nanogui/glcanvas.h>
#include <map>
#include "Mesh.h"
#include "KeyFrameAnimator.h"

using namespace nanogui;


class Viewer : public nanogui::Screen {
public:

    Viewer();
    ~Viewer();

    virtual void draw(NVGcontext *ctx);
    virtual void drawContents();


    void animate_hair();
    void refresh_mesh();
    void refresh_trackball_center();

    virtual bool keyboardEvent(int key, int scancode, int action, int modifiers);

    Vector2f getScreenCoord();
    bool scrollEvent(const Vector2i &p, const Vector2f &rel);
    bool mouseMotionEvent(const Vector2i &p, const Vector2i &rel, int button, int modifiers);
    bool mouseButtonEvent(const Vector2i &p, int button, bool down, int modifiers);
private:
    void initShaders();
    void computeCameraMatrices(Eigen::Matrix4f &model,
        Eigen::Matrix4f &view,
        Eigen::Matrix4f &proj);

    TransFactor computeModelTransformation();

    struct CameraParameters {
        nanogui::Arcball arcball;
        float zoom = 1.0f, viewAngle = 45.0f;
        float dnear = 0.05f, dfar = 100.0f;
        Eigen::Vector3f eye = Eigen::Vector3f(0.0f, 0.0f, 5.0f);
        Eigen::Vector3f center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        Eigen::Vector3f up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
        Eigen::Vector3f modelTranslation = Eigen::Vector3f::Zero();
        Eigen::Vector3f modelTranslation_start = Eigen::Vector3f::Zero();
        float modelZoom = 1.0f;
    };

    CameraParameters m_camera;
    bool m_translate = false;
    Vector2i m_translateStart = Vector2i(0, 0);

    // Variables for the viewer
    nanogui::GLShader m_head_shader;
    nanogui::GLShader m_hair_shader;
    nanogui::Window *m_window;
    nanogui::Window *m_window_fps;

    Mesh* m_mesh;

    bool playing = false;

    int currframe = 0;
    TextBox *text_frameid;
    TextBox *FPS;
    Slider *slider;
    TextBox *text_iskeyframe;

    KeyFrameAnimator *m_animator = NULL;

    int frame_num = 60;

    float elapsed = 1;
    bool init_hair_trans = true;
    Eigen::Matrix4f init_model;
};


#endif //TINYOBJVIEWER_VIEWER_H
