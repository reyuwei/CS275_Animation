//
// Created by stephan-lb on 22/03/2017.
//

#include "time.h"
#include "Viewer.h"
#include <algorithm>
#include <Windows.h>

Viewer::Viewer() : nanogui::Screen(Eigen::Vector2i(1024, 900), "KeyFrame") {

    m_window = new Window(this, "Controls");
    m_window->setPosition(Vector2i(15, 15));
    m_window->setLayout(new GroupLayout());

    Button *b = new Button(m_window, "Open mesh ...");
    b->setCallback([this]() {
        std::string filename = nanogui::file_dialog({ {"obj", "Wavefront OBJ"} }, false);

        if (filename != "") {
            mProcessEvents = false;
            m_mesh = new Mesh(filename);
            this->refresh_mesh();
            this->refresh_trackball_center();
            mProcessEvents = true;
        }
    });

    Widget *controller_panel = new Widget(m_window);
    controller_panel->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));

    Button *play = new Button(controller_panel, "Run");
    play->setCallback([this, play]() {
        playing = !playing;
        if (playing)
            play->setCaption("Stop");
        else
            play->setCaption("Run");
    });

    Button *add_keyframe = new Button(controller_panel, "Add");
    add_keyframe->setCallback([this]() {
        TransFactor trans = computeModelTransformation();
        if (m_animator->AddKeyFrame(currframe, trans))
            text_iskeyframe->setVisible(true);
    });

    Button *edit_keyframe = new Button(controller_panel, "Edit");
    edit_keyframe->setCallback([this]() {
        TransFactor trans = computeModelTransformation();
        if (m_animator->EditKeyFrame(currframe, trans))
            text_iskeyframe->setVisible(true);
    });

    Button *delete_keyframe = new Button(controller_panel, "Delete");
    delete_keyframe->setCallback([this]() {
        if (m_animator->DeleteKeyFrame(currframe))
            text_iskeyframe->setVisible(false);
    });

    text_iskeyframe = new TextBox(controller_panel);
    text_iskeyframe->setFixedSize(Vector2i(300, 25));
    text_iskeyframe->setValue("KEYFRAME!");
    text_iskeyframe->setVisible(true);

    Widget *slider_panel = new Widget(m_window);
    slider_panel->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Middle, 0, 5));


    /* Add a text box and set defaults */
    text_frameid = new nanogui::TextBox(slider_panel);
    text_frameid->setFixedSize(Vector2i(300, 25));
    text_frameid->setValue("Current Frame: 0");

    slider = new Slider(slider_panel);
    slider->setValue(0);
    slider->setFixedWidth(500);
    slider->setCallback([this](float value) {
        currframe = value * frame_num;
        text_frameid->setValue("Current Frame: " + std::to_string(currframe));
    });

    m_window_fps = new Window(this, "FPS");
    m_window_fps->setPosition(Vector2i(600, 15));
    m_window_fps->setLayout(new GroupLayout());
    Widget *fps_panel = new Widget(m_window_fps);
    fps_panel->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));

    FPS = new nanogui::TextBox(fps_panel);
    FPS->setFixedSize(Vector2i(300, 25));
    FPS->setValue("FPS: " + std::to_string(1.0f / elapsed));


    performLayout();
    initShaders();

    //m_mesh = new Mesh("teapot.obj");

    m_mesh = new Mesh("..\\..\\model\\Pasha_guard_head.obj");


    this->refresh_mesh();
    this->refresh_trackball_center();
}

bool Viewer::keyboardEvent(int key, int scancode, int action, int modifiers) {
    if (Screen::keyboardEvent(key, scancode, action, modifiers)) {
        return true;
    }
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        setVisible(false);
        return true;
    }
    return false;
}

void Viewer::draw(NVGcontext *ctx) {
    /* Draw the user interface */
    Screen::draw(ctx);
}

Vector2f Viewer::getScreenCoord() {
    Vector2i pos = mousePos();
    return Vector2f(2.0f * (float)pos.x() / width() - 1.0f,
        1.0f - 2.0f * (float)pos.y() / height());
}

void Viewer::drawContents() {
    float lasttime = glfwGetTime();
    using namespace nanogui;

    if (m_mesh == nullptr)
        return;

    /* Draw the window contents using OpenGL */
    m_head_shader.bind();

    Eigen::Matrix4f model, view, proj;
    computeCameraMatrices(model, view, proj);

    if (m_animator == NULL)
        m_animator = new KeyFrameAnimator(computeModelTransformation(), frame_num);

    /* Draw animation */
    if (playing)
    {
        currframe++;
        currframe = currframe > frame_num ? 0 : currframe;
        slider->setValue(currframe * 1.0f / frame_num);
        text_frameid->setValue("Current Frame: " + std::to_string(currframe));
        model = m_animator->GetView(currframe);
    }

    if (m_animator->GetKeyFrameIndex(currframe) != -1)
        text_iskeyframe->setVisible(true);
    else
        text_iskeyframe->setVisible(false);

    Matrix4f mv = view * model;
    Matrix4f p = proj;
    /* MVP uniforms */
    m_head_shader.setUniform("MV", mv);
    m_head_shader.setUniform("P", p);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_mesh->get_texture()->texture());
    m_head_shader.setUniform("tex", 0);

    // Setup OpenGL (making sure the GUI doesn't disable these
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    m_head_shader.drawIndexed(GL_TRIANGLES, 0, m_mesh->get_number_of_face());

    // show hair
    glLineWidth(20.0f);
    m_hair_shader.bind();
    m_hair_shader.setUniform("MV", mv);
    m_hair_shader.setUniform("P", p);
    m_hair_shader.drawIndexed(GL_LINES, 0, m_mesh->get_number_of_hair());
    this->animate_hair();


    Sleep(30);
    float fTime = glfwGetTime();
    elapsed = fTime - lasttime;
    char fps[20];
    sprintf(fps, "%7.2f", 1.0f / elapsed);
    std::string fps_str = fps;
    FPS->setValue("FPS: " + fps_str);
}


bool Viewer::scrollEvent(const Vector2i &p, const Vector2f &rel) {
    if (!Screen::scrollEvent(p, rel)) {
        m_camera.zoom = max(0.1, m_camera.zoom * (rel.y() > 0 ? 1.1 : 0.9));
    }
    return true;
}

bool Viewer::mouseMotionEvent(const Vector2i &p, const Vector2i &rel,
    int button, int modifiers) {
    if (!Screen::mouseMotionEvent(p, rel, button, modifiers)) {
        if (m_camera.arcball.motion(p)) {
            //
        }
        else if (m_translate) {
            Eigen::Matrix4f model, view, proj;
            computeCameraMatrices(model, view, proj);
            Eigen::Vector3f mesh_center = m_mesh->get_mesh_center();
            float zval = nanogui::project(Vector3f(mesh_center.x(),
                mesh_center.y(),
                mesh_center.z()),
                view * model, proj, mSize).z();
            Eigen::Vector3f pos1 = nanogui::unproject(
                Eigen::Vector3f(p.x(), mSize.y() - p.y(), zval),
                view * model, proj, mSize);
            Eigen::Vector3f pos0 = nanogui::unproject(
                Eigen::Vector3f(m_translateStart.x(), mSize.y() -
                    m_translateStart.y(), zval), view * model, proj, mSize);
            m_camera.modelTranslation = m_camera.modelTranslation_start + (pos1 - pos0);
        }
    }
    return true;
}

bool Viewer::mouseButtonEvent(const Vector2i &p, int button, bool down, int modifiers) {
    if (!Screen::mouseButtonEvent(p, button, down, modifiers)) {
        if (button == GLFW_MOUSE_BUTTON_1 && modifiers == 0) {
            m_camera.arcball.button(p, down);
        }
        else if (button == GLFW_MOUSE_BUTTON_2 ||
            (button == GLFW_MOUSE_BUTTON_1 && modifiers == GLFW_MOD_SHIFT)) {
            m_camera.modelTranslation_start = m_camera.modelTranslation;
            m_translate = true;
            m_translateStart = p;
        }
    }
    if (button == GLFW_MOUSE_BUTTON_1 && !down) {
        m_camera.arcball.button(p, false);
    }
    if (!down) {
        m_translate = false;
    }
    return true;
}

void Viewer::initShaders() {
    // Shaders
    m_head_shader.init(
        "a_simple_shader",

        /* Vertex shader */
        "#version 330\n"
        "uniform mat4 MV;\n"
        "uniform mat4 P;\n"

        "in vec3 position;\n"
        "in vec2 vertexUV;\n"
        "out vec2 UV;\n"

        "void main() {\n"
        "    vec4 vpoint_mv = MV * vec4(position, 1.0);\n"
        "    gl_Position = P * vpoint_mv;\n"
        "    UV = vertexUV;\n"
        "}",

        /* Fragment shader */
        "#version 330\n"

        "in vec2 UV;\n"
        "out vec4 color;\n"
        "// Values that stay constant for the whole mesh.\n"
        "uniform sampler2D tex;\n"
        "void main()\n"
        "{"
        "color = texture( tex, UV ).rgba;\n"
        "}"
    );

    m_hair_shader.init(
        /* An identifying name */
        "a_simple_shader_hair",

        /* Vertex shader */
        "#version 330\n"
        "uniform mat4 MV;\n"
        "uniform mat4 P;\n"

        "in vec3 position;\n"
        "in vec3 vec_colors;\n"

        "out vec4 lcolor;\n"

        "void main() {\n"
        "    vec4 vpoint_mv = MV * vec4(position, 1.0);\n"
        "    gl_Position = P * vpoint_mv;\n"
        "    lcolor = vec4(vec_colors,1.0);\n"
        "}",

        /* Fragment shader */
        "#version 330\n"
        "in vec4 lcolor;\n"
        "out vec4 color;\n"
        "void main() {\n"
        "    color = lcolor;\n"
        "}"
    );
}

void Viewer::refresh_trackball_center() {
    // Re-center the mesh
    Eigen::Vector3f mesh_center = m_mesh->get_mesh_center();
    m_camera.arcball = Arcball();
    m_camera.arcball.setSize(mSize);
    m_camera.modelZoom = 2 / m_mesh->get_dist_max();
    m_camera.modelTranslation = -Vector3f(mesh_center.x(), mesh_center.y(), mesh_center.z());
}

void Viewer::refresh_mesh() {
    m_head_shader.bind();
    m_head_shader.uploadIndices(*(m_mesh->get_indices()));
    m_head_shader.uploadAttrib("position", *(m_mesh->get_points()));
    m_head_shader.uploadAttrib("vertexUV", *(m_mesh->get_uvs()));

    m_hair_shader.bind();
    m_hair_shader.uploadIndices(*(m_mesh->get_hairindices()));
    m_hair_shader.uploadAttrib("position", *(m_mesh->get_hairpos()));
    m_hair_shader.uploadAttrib("vec_colors", *(m_mesh->get_haircolors()));
}

void Viewer::animate_hair()
{
    m_hair_shader.bind();
    m_hair_shader.uploadAttrib("position", *(m_mesh->get_hairpos()));
}

void Viewer::computeCameraMatrices(Eigen::Matrix4f &model,
    Eigen::Matrix4f &view,
    Eigen::Matrix4f &proj) {

    view = nanogui::lookAt(m_camera.eye, m_camera.center, m_camera.up);

    float fH = std::tan(m_camera.viewAngle / 360.0f * M_PI) * m_camera.dnear;
    float fW = fH * (float)mSize.x() / (float)mSize.y();

    proj = nanogui::frustum(-fW, fW, -fH, fH, m_camera.dnear, m_camera.dfar);

    model = m_camera.arcball.matrix();
    model = model * nanogui::scale(Eigen::Vector3f::Constant(m_camera.zoom * m_camera.modelZoom));
    model = model * nanogui::translate(m_camera.modelTranslation);
}

TransFactor Viewer::computeModelTransformation()
{
    TransFactor trans;
    trans.SetScale(m_camera.zoom * m_camera.modelZoom);
    nanogui::Quaternionf rot = m_camera.arcball.state();
    trans.SetRotation(rot);
    trans.SetTranslation(m_camera.modelTranslation);
    return trans;
}

Viewer::~Viewer() {
    m_head_shader.free();
    m_hair_shader.free();
    delete m_mesh;
    delete m_animator;
}
