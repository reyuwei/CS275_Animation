#pragma once
#include <map>
#include <vector>
#include <Eigen/Dense>
#include <nanogui/glutil.h>
#include <iostream>
//#include "Mesh.h"


const static float SPRING_REST_LENGTH = 20.0f;
const static float GRAVITY = 5.0f;
const static float MASS = 30.0f;
static float TIME_STEP = 0.2f; // 20ms
const static Eigen::Vector3f GRAVITY_DIRECTION = Eigen::Vector3f(0.0f, 0.0f, -1.0f);


bool isEigenV3Null(Eigen::Vector3f v);
static int factorial(int number);
static int combinator(int n, int m);
static Eigen::Vector3f BazierLinePoint(float t, std::vector<Eigen::Vector3f> control_points);
typedef Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic> MatrixXu;

class Spring
{
private:
    Eigen::Vector3f anchor = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f extra_dir = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

    Eigen::Vector3f endpos = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    float k_stiffness = 30.0f;
    float k_damping = 20.0f;

public:
    Spring();
    Spring(Eigen::Vector3f start_, Eigen::Vector3f dir_);
    Eigen::Vector3f getAnchor();
    Eigen::Vector3f getEndPos();
    void setAnchor(Eigen::Vector3f start_);
    float getCurrLength();
    Eigen::Vector3f getCurrDir();
    Eigen::Vector3f getStiffF();
    Eigen::Vector3f getDampingF();
    Eigen::Vector3f animate(Spring *before, Spring *after);
    void setExtraDir(Eigen::Vector3f extra_force_dir);
};

class SingleHair
{
    std::vector<Spring> segments;
    Eigen::Vector3f root;
    Eigen::Vector3f ori_root;
    Eigen::Vector3f ori_dir;

    //Mesh *head;
public:
    SingleHair(Eigen::Vector3f root_, Eigen::Vector3f dir_, int segment_count_);
    std::vector<Eigen::Vector3f> Animate(bool doanimate);
    Eigen::Vector3f get_root_dir();
    void reset();
    void add_force(int segment_id, Eigen::Vector3f normal);
    void transform_root(Eigen::Matrix4f& model);
    //void setHead(Mesh *model)
    //{
    //    head = model;
    //}
};

class Hair
{
    std::vector<SingleHair> guide_strands;
    std::vector<std::vector<Eigen::Vector3f>> interpolate_strands_roots;
    int interpolate_count = -1;
    int guide_strand_count = -1;
    int segment_count = -1;
    int bline_sample_points = 3;

    //Mesh *head;

public:
    Hair();
    Hair(Eigen::MatrixXf hairpoints_, Eigen::MatrixXf hairnormals_, int guide_strand_count_, int interpolate_count_, int segment_count_);
    //void SetHead(Mesh *model);
    Eigen::Vector3f InterpolateStrandPoint(float t, Eigen::Vector3f guide_p, Eigen::Vector3f dir_);
    Eigen::MatrixXf get_positions(Eigen::MatrixXf &hair_points_normal, bool doanimate = false);
    Eigen::MatrixXf get_colors();
    MatrixXu get_indices();
    int get_number_hair();
    void add_force(int strand_id, int segment_id, Eigen::Vector3f normal);
    void transform(Eigen::Matrix4f& model);
    void reset();
};

