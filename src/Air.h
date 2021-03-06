#pragma once

#include <map>
#include <vector>
#include <Eigen/Dense>
#include <nanogui/glutil.h>
#include <omp.h>
#include "MassSpring.h"
//#include "Mesh.h"

#define NDIM 3
const static int LINEARITER = 10;

static float fluid_step = TIME_STEP;

enum V_AXIS
{
    X = 0, Y = 1, Z = 2,
};

enum FIELD
{
    F_DENSITY,
    F_VELOCITY,

    VX, VY, VZ, PREVX, PREVY, PREVZ
};

class Grid
{
public:
    float size;
    Eigen::Vector3f center;

    float density;
    float prev_density;

    std::vector<float> velocity;
    std::vector<float> prev_velocity;

public:
    Grid(Eigen::Vector3f pos, float length_);
    Eigen::Vector3f traceback();
    void swapvelocity();
    void swapdensity();
    Eigen::Vector3f visvelocity();
};

class Fluid
{
public:

    float k_visc = 2.0f;
    float k_diff = 0.0f;
    float gridlength = 1;
    int gridsidecount = 6;
    Eigen::Vector3f start_pos = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    std::vector<std::vector<std::vector<Grid>>> grids; // 3-dim

    ms::MatrixXu indices;
    Eigen::MatrixXf positions;

public:
    Fluid(Eigen::Vector3f center, float diameter);
    void Animate();
    void swap(FIELD fd);
    void addforce(FIELD fd);
    void addforce_at(Eigen::Vector3f offset, Eigen::Vector3f f);
    void advection(FIELD fd);
    void diffusion(FIELD fd);
    void projection();
    void setbound(FIELD fd, int nd = 0);
    void set_bnd_v(int nd, Eigen::Vector3f* x);
    float interpolatev(Eigen::Vector3f center_pos, int nd);
    float interpolated(Eigen::Vector3f center_pos);
    void lin_solve_diffusion(float a, float c, FIELD fd);
    void lin_solve_projection(float a, float c);
    void vstep();
    void dstep();
    void get_source_force();
    int get_number_of_grid();
    ms::MatrixXu get_indices();
    Eigen::MatrixXf get_positions();

    void update_velocity_field(std::vector<Eigen::Vector3f> p, std::vector<Eigen::Vector3f> v);
};