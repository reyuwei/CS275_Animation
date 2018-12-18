#pragma once

#include <map>
#include <vector>
#include <Eigen/Dense>
#include <nanogui/glutil.h>
#include <omp.h>
#include "MassSpring.h"
#include "nanogui/common.h"

typedef Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic> MatrixXu;

#define NDIM 3
const static int LINEARITER = 10;
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

//v - viscosity
//p - pressure
//rou - density
//f - external force
//u - velocity

class Grid
{
public:
    float size;
    Eigen::Vector3f center;

    float density;
    float prev_density;

    std::vector<Eigen::Vector3f> velocity;
    std::vector<Eigen::Vector3f> prev_velocity;

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
    float k_visc = 1.0f;
    float k_diff = 0.0f;
    float gridlength = 1;
    int gridsidecount = 3;
    Eigen::Vector3f start_pos = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    std::vector<std::vector<std::vector<Grid>>> grids; // 3-dim

    MatrixXu indices;
    Eigen::MatrixXf positions;

public:
    Fluid(Eigen::Vector3f center, float diameter);
    void Animate();
    void swap(FIELD fd);
    void addforce(int nd, FIELD fd);
    void advection(int nd, FIELD fd);
    void diffusion(int nd, FIELD fd);
    void projection();
    void setbound(FIELD fd);
    Eigen::Vector3f interpolatev(Eigen::Vector3f center_pos, int nd);
    float interpolated(Eigen::Vector3f center_pos);
    void lin_solve_diffusion(int nd, float a, float c, FIELD fd);
    void lin_solve_projection(float a, float c);
    void vstep();
    void dstep();

    int get_number_of_grid();
    MatrixXu get_indices();
    Eigen::MatrixXf get_positions();
};