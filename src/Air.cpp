#include "Air.h"
#include <iostream>
#include <algorithm>

float Gaussian(float x)
{
    float sigma = 0.5f;
    return 1 / (sigma * sqrt(2 * 3.14))* exp(-0.5f * pow(x / sigma, 2));
}


Grid::Grid(Eigen::Vector3f pos, float length_)
{
    center = Eigen::Vector3f(pos);
    size = length_;

    density = 0;
    prev_density = 0;

    //velocity.push_back(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    //velocity.push_back(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    //velocity.push_back(Eigen::Vector3f(0.0f, 0.0f, 0.0f));

    //prev_velocity.push_back(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    //prev_velocity.push_back(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    //prev_velocity.push_back(Eigen::Vector3f(0.0f, 0.0f, 0.0f));

    velocity.push_back(0);
    velocity.push_back(0);
    velocity.push_back(0);

    prev_velocity.push_back(0);
    prev_velocity.push_back(0);
    prev_velocity.push_back(0);
}

Eigen::Vector3f Grid::traceback()
{
    Eigen::Vector3f velocity_all(velocity[X], velocity[Y], velocity[Z]);
    //Eigen::Vector3f traceback_center = center + -velocity[X] * TIME_STEP + -velocity[Y] * TIME_STEP + -velocity[Z] * TIME_STEP;
    Eigen::Vector3f traceback_center = center + -velocity_all * fluid_step;
    return traceback_center;
}

void Grid::swapvelocity()
{
    for (int i = 0; i < velocity.size(); i++)
    {
        float temp = prev_velocity[i];
        prev_velocity[i] = velocity[i];
        velocity[i] = temp;

        //Eigen::Vector3f temp = Eigen::Vector3f(prev_velocity[i]);
        //prev_velocity[i] = Eigen::Vector3f(velocity[i]);
        //velocity[i] = Eigen::Vector3f(temp);
    }
}

void Grid::swapdensity()
{
    float temp = prev_density;
    prev_density = density;
    density = prev_density;
}

Eigen::Vector3f Grid::visvelocity()
{
    velocity[X] = std::max(0.01f, velocity[X]);
    velocity[Y] = std::max(0.01f, velocity[Y]);
    velocity[Z] = std::max(0.01f, velocity[Z]);
    Eigen::Vector3f dir_v(velocity[X], velocity[Y], velocity[Z]);

    //std::cout << dir_v[0] << " " << dir_v[1] << " " << dir_v[2] << std::endl;
    //std::cout << center[0] << " " << center[1] << " " << center[2] << std::endl;

    //if (dir_v.x() <= 1e-9 && dir_v.y() <= 1e-9 && dir_v.z() <= 1e-9)
    //    dir_v << 0.01f, 0.01f, 0.01f;
    //else
    //    dir_v = dir_v.normalized();
    Eigen::Vector3f end_v = center + dir_v * size / 2.0f;
    return end_v;
}

Fluid::Fluid(Eigen::Vector3f center, float diameter)
{
    indices = ms::MatrixXu(2, get_number_of_grid());
    for (int i = 0; i < get_number_of_grid(); i++)
    {
        indices.col(i) << 2 * i, 2 * i + 1;
    }
    positions = Eigen::MatrixXf(3, get_number_of_grid() * 2);
    int counter = 0;
    diameter *= 2.0f;
    gridlength = diameter * 1.0f / gridsidecount;
    float half_diameter = diameter / 2.0f;
    float half_gridlength = gridlength / 2.0f;
    start_pos = center - Eigen::Vector3f(half_diameter, half_diameter, half_diameter);
    Eigen::Vector3f half_gridlengthv = Eigen::Vector3f(half_gridlength, half_gridlength, half_gridlength);
    Eigen::Vector3f full_gridlengthv = Eigen::Vector3f(gridlength, gridlength, gridlength);
    for (int x = 0; x < gridsidecount; x++)
    {
        std::vector<std::vector<Grid>> oneface;
        for (int y = 0; y < gridsidecount; y++)
        {
            std::vector<Grid> oneline;
            for (int z = 0; z < gridsidecount; z++)
            {
                Eigen::Vector3f pos = start_pos + full_gridlengthv.cwiseProduct(Eigen::Vector3f(x, y, z))
                    + half_gridlengthv;
                //std::cout << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
                oneline.push_back(Grid(pos, gridlength));
                positions.col(counter++) << pos.x(), pos.y(), pos.z();
                positions.col(counter++) << pos.x(), pos.y(), pos.z();
            }
            oneface.push_back(oneline);
        }
        grids.push_back(oneface);
    }
}

void Fluid::Animate()
{
    //fluid_step = fluid_step * 0.1f;

    fluid_step = TIME_STEP;

    vstep();

    dstep();
}

void Fluid::get_source_force()
{

    //grids[gridsidecount / 2][gridsidecount / 2][gridsidecount / 2].velocity[X] = 10.0f;

    // add velocity
    for (int j = 0; j < gridsidecount; j++)
        for (int k = 0; k < gridsidecount; k++)
        {
            //grids[0][j][k].prev_velocity[X] = Eigen::Vector3f(0.50f, 0.0f, 0.0f);
            //grids[0][j][k].prev_velocity[Y] = Eigen::Vector3f(0.50f, 0.0f, 0.0f);
            //grids[0][j][k].prev_velocity[Z] = Eigen::Vector3f(0.50f, 0.0f, 0.0f);

            grids[0][j][k].prev_velocity[X] = 10.0f;
            grids[0][j][k].prev_velocity[Y] = 0.0f;
            grids[0][j][k].prev_velocity[Z] = 0.0f;
        }
    //grids[1][1][1].prev_density = 200.0f;

    // add density at center;
    //grids[gridsidecount / 2][gridsidecount / 2][gridsidecount / 2].prev_density = 200.0f;

}

void Fluid::vstep()
{
    // add force
    addforce(F_VELOCITY);

    swap(F_VELOCITY);

    // diffusion
    diffusion(F_VELOCITY);

    // projection
    projection();

    swap(F_VELOCITY);

    // advection
    advection(F_VELOCITY);

    // projection
    projection();

}

void Fluid::dstep()
{
    addforce(F_DENSITY);
    swap(F_DENSITY);
    diffusion(F_DENSITY);
    swap(F_DENSITY);
    advection(F_DENSITY);
}

void Fluid::swap(FIELD fd)
{
    if (fd == F_DENSITY)
        for (int x = 0; x < gridsidecount; x++)
            for (int y = 0; y < gridsidecount; y++)
                for (int z = 0; z < gridsidecount; z++)
                    grids[x][y][z].swapdensity();
    else if (fd == F_VELOCITY)
        for (int x = 0; x < gridsidecount; x++)
            for (int y = 0; y < gridsidecount; y++)
                for (int z = 0; z < gridsidecount; z++)
                    grids[x][y][z].swapvelocity();
}

void Fluid::addforce(FIELD fd)
{
    //auto f = GRAVITY_DIRECTION * GRAVITY;
    if (fd == F_VELOCITY)
        for (int x = 0; x < gridsidecount; x++)
            for (int y = 0; y < gridsidecount; y++)
                for (int z = 0; z < gridsidecount; z++)
                {
                    grids[x][y][z].velocity[X] += fluid_step *grids[x][y][z].prev_velocity[X];
                    grids[x][y][z].velocity[Y] += fluid_step *grids[x][y][z].prev_velocity[Y];
                    grids[x][y][z].velocity[Z] += fluid_step *grids[x][y][z].prev_velocity[Z];
                }
    else if (fd == F_DENSITY)
        for (int x = 0; x < gridsidecount; x++)
            for (int y = 0; y < gridsidecount; y++)
                for (int z = 0; z < gridsidecount; z++)
                    grids[x][y][z].density += fluid_step * grids[x][y][z].prev_density;

}

void Fluid::addforce_at(Eigen::Vector3f offset, Eigen::Vector3f f)
{
    int x0 = floor(offset.x());
    int y0 = floor(offset.y());
    int z0 = floor(offset.z());


    float xd = abs(offset.x() - x0);
    float yd = abs(offset.y() - y0);
    float zd = abs(offset.z() - z0);

    x0 = std::max(0, x0);
    y0 = std::max(0, y0);
    z0 = std::max(0, z0);
    x0 = std::min(x0, gridsidecount - 1);
    y0 = std::min(y0, gridsidecount - 1);
    z0 = std::min(z0, gridsidecount - 1);

    grids[x0][y0][z0].velocity[X] = f[0];
    grids[x0][y0][z0].velocity[Y] = f[1];
    grids[x0][y0][z0].velocity[Z] = f[2];


    //int x1 = std::min(x0 + 1, gridsidecount - 1);
    //int y1 = std::min(y0 + 1, gridsidecount - 1);
    //int z1 = std::min(z0 + 1, gridsidecount - 1);
}

void Fluid::advection(FIELD fd)
{
    if (fd == F_VELOCITY)
    {
        for (int x = 0; x < gridsidecount; x++)
            for (int y = 0; y < gridsidecount; y++)
                for (int z = 0; z < gridsidecount; z++)
                {
                    // trace back grid center
                    Eigen::Vector3f center_pos = grids[x][y][z].traceback();
                    // find its grid
                    Eigen::Vector3f offset = center_pos - start_pos;
                    // interpolation
                    grids[x][y][z].velocity[X] = interpolatev(offset / gridlength, X);
                    grids[x][y][z].velocity[Y] = interpolatev(offset / gridlength, Y);
                    grids[x][y][z].velocity[Z] = interpolatev(offset / gridlength, Z);
                }
        setbound(fd);
    }
    else if (fd == F_DENSITY)
    {
        for (int x = 0; x < gridsidecount; x++)
            for (int y = 0; y < gridsidecount; y++)
                for (int z = 0; z < gridsidecount; z++)
                {
                    // trace back grid center
                    Eigen::Vector3f center_pos = grids[x][y][z].traceback();
                    // find its grid
                    Eigen::Vector3f offset = center_pos - start_pos;
                    // interpolation
                    grids[x][y][z].density = interpolated(offset / gridlength);
                }
        setbound(fd);
    }
}

void Fluid::diffusion(FIELD fd)
{
    float scalar = 0;
    if (fd == F_VELOCITY)
    {
        scalar = k_visc;
    }
    else if (fd == F_DENSITY)
    {
        scalar = k_diff;
    }
    float a = scalar * fluid_step * gridsidecount * gridsidecount * gridsidecount;
    float c = 1 + 6 * a;
    lin_solve_diffusion(a, c, fd);
}

void Fluid::set_bnd_v(int nd, Eigen::Vector3f *x)
{
    //int M = gridsidecount - 2;
    //int N = M;
    //int O = M;
    //int b = nd + 1;

    //// bounds are cells at faces of the cube

    //int i, j;

    ////setting faces
    //for (i = 1; i <= M; i++)
    //{
    //    for (j = 1; j <= N; j++)
    //    {
    //        x[IX(i, j, 0)] = b == 3 ? -x[IX(i, j, 1)] : x[IX(i, j, 1)];
    //        x[IX(i, j, O + 1)] = b == 3 ? -x[IX(i, j, O)] : x[IX(i, j, O)];
    //    }
    //}

    //for (i = 1; i <= N; i++)
    //{
    //    for (j = 1; j <= O; j++)
    //    {
    //        x[IX(0, i, j)] = b == 1 ? -x[IX(1, i, j)] : x[IX(1, i, j)];
    //        x[IX(M + 1, i, j)] = b == 1 ? -x[IX(M, i, j)] : x[IX(M, i, j)];
    //    }
    //}

    //for (i = 1; i <= M; i++)
    //{
    //    for (j = 1; j <= O; j++)
    //    {
    //        x[IX(i, 0, j)] = b == 2 ? -x[IX(i, 1, j)] : x[IX(i, 1, j)];
    //        x[IX(i, N + 1, j)] = b == 2 ? -x[IX(i, N, j)] : x[IX(i, N, j)];
    //    }
    //}

    ////Setting edges
    //for (i = 1; i <= M; i++)
    //{
    //    x[IX(i, 0, 0)] = 1.0 / 2.0 * (x[IX(i, 1, 0)] + x[IX(i, 0, 1)]);
    //    x[IX(i, N + 1, 0)] = 1.0 / 2.0 * (x[IX(i, N, 0)] + x[IX(i, N + 1, 1)]);
    //    x[IX(i, 0, O + 1)] = 1.0 / 2.0 * (x[IX(i, 0, O)] + x[IX(i, 1, O + 1)]);
    //    x[IX(i, N + 1, O + 1)] = 1.0 / 2.0 * (x[IX(i, N, O + 1)] + x[IX(i, N + 1, O)]);
    //}

    //for (i = 1; i <= N; i++)
    //{
    //    x[IX(0, i, 0)] = 1.0 / 2.0 * (x[IX(1, i, 0)] + x[IX(0, i, 1)]);
    //    x[IX(M + 1, i, 0)] = 1.0 / 2.0 * (x[IX(M, i, 0)] + x[IX(M + 1, i, 1)]);
    //    x[IX(0, i, O + 1)] = 1.0 / 2.0 * (x[IX(0, i, O)] + x[IX(1, i, O + 1)]);
    //    x[IX(M + 1, i, O + 1)] = 1.0 / 2.0 * (x[IX(M, i, O + 1)] + x[IX(M + 1, i, O)]);
    //}

    //for (i = 1; i <= O; i++)
    //{
    //    x[IX(0, 0, i)] = 1.0 / 2.0 * (x[IX(0, 1, i)] + x[IX(1, 0, i)]);
    //    x[IX(0, N + 1, i)] = 1.0 / 2.0 * (x[IX(0, N, i)] + x[IX(1, N + 1, i)]);
    //    x[IX(M + 1, 0, i)] = 1.0 / 2.0 * (x[IX(M, 0, i)] + x[IX(M + 1, 1, i)]);
    //    x[IX(M + 1, N + 1, i)] = 1.0 / 2.0 * (x[IX(M + 1, N, i)] + x[IX(M, N + 1, i)]);
    //}

    ////setting corners
    //x[IX(0, 0, 0)] = 1.0 / 3.0 * (x[IX(1, 0, 0)] + x[IX(0, 1, 0)] + x[IX(0, 0, 1)]);
    //x[IX(0, N + 1, 0)] = 1.0 / 3.0 * (x[IX(1, N + 1, 0)] + x[IX(0, N, 0)] + x[IX(0, N + 1, 1)]);

    //x[IX(M + 1, 0, 0)] = 1.0 / 3.0 * (x[IX(M, 0, 0)] + x[IX(M + 1, 1, 0)] + x[IX(M + 1, 0, 1)]);
    //x[IX(M + 1, N + 1, 0)] = 1.0 / 3.0 * (x[IX(M, N + 1, 0)] + x[IX(M + 1, N, 0)] + x[IX(M + 1, N + 1, 1)]);

    //x[IX(0, 0, O + 1)] = 1.0 / 3.0 * (x[IX(1, 0, O + 1)] + x[IX(0, 1, O + 1)] + x[IX(0, 0, O)]);
    //x[IX(0, N + 1, O + 1)] = 1.0 / 3.0 * (x[IX(1, N + 1, O + 1)] + x[IX(0, N, O + 1)] + x[IX(0, N + 1, O)]);

    //x[IX(M + 1, 0, O + 1)] = 1.0 / 3.0 * (x[IX(M, 0, O + 1)] + x[IX(M + 1, 1, O + 1)] + x[IX(M + 1, 0, O)]);
    //x[IX(M + 1, N + 1, O + 1)] = 1.0 / 3.0 * (x[IX(M, N + 1, O + 1)] + x[IX(M + 1, N, O + 1)] + x[IX(M + 1, N + 1, O)]);
}

void Fluid::setbound(FIELD fd, int nd)
{
    if (fd == F_VELOCITY)
    {
        for (int x = 0; x < gridsidecount; x++)
            for (int y = 0; y < gridsidecount; y++)
                for (int z = 0; z < gridsidecount; z++)
                    if (x == 0 || y == 0 || z == 0 ||
                        x == gridsidecount - 1 || y == gridsidecount - 1 || z == gridsidecount - 1) // is boundary grid
                    {
                        // set velocity to zero
                        //grids[x][y][z].velocity[X] = Eigen::Vector3f(0, 0, 0);
                        //grids[x][y][z].velocity[Y] = Eigen::Vector3f(0, 0, 0);
                        //grids[x][y][z].velocity[Z] = Eigen::Vector3f(0, 0, 0);

                        grids[x][y][z].velocity[nd] = 0;
                        //grids[x][y][z].velocity[Y] = 0;
                        //grids[x][y][z].velocity[Z] = 0;

                        //grids[x][y][z].velocity[X] = -0.5f*grids[x][y][z].velocity[X];
                        //grids[x][y][z].velocity[Y] = -0.5f*grids[x][y][z].velocity[Y];
                        //grids[x][y][z].velocity[Z] = -0.5f*grids[x][y][z].velocity[Z];
                    }
    }
    else if (fd == F_DENSITY)
    {
        for (int x = 0; x < gridsidecount; x++)
            for (int y = 0; y < gridsidecount; y++)
                for (int z = 0; z < gridsidecount; z++)
                    if (x == 0 || y == 0 || z == 0 ||
                        x == gridsidecount - 1 || y == gridsidecount - 1 || z == gridsidecount - 1) // is boundary grid
                    {
                        // set velocity to zero
                        grids[x][y][z].density = 0;
                    }
    }
    else if (fd == PREVX)
    {
        for (int x = 0; x < gridsidecount; x++)
            for (int y = 0; y < gridsidecount; y++)
                for (int z = 0; z < gridsidecount; z++)
                    if (x == 0 || y == 0 || z == 0 ||
                        x == gridsidecount - 1 || y == gridsidecount - 1 || z == gridsidecount - 1) // is boundary grid
                    {
                        // set velocity to zero
                        //grids[x][y][z].prev_velocity[X] = Eigen::Vector3f(0, 0, 0);
                        grids[x][y][z].prev_velocity[X] = 0;
                        //grids[x][y][z].prev_velocity[X] = -0.5f * grids[x][y][z].prev_velocity[X];
                    }
    }
    else if (fd == PREVY)
    {
        for (int x = 0; x < gridsidecount; x++)
            for (int y = 0; y < gridsidecount; y++)
                for (int z = 0; z < gridsidecount; z++)
                    if (x == 0 || y == 0 || z == 0 ||
                        x == gridsidecount - 1 || y == gridsidecount - 1 || z == gridsidecount - 1) // is boundary grid
                    {
                        // set velocity to zero
                        //grids[x][y][z].prev_velocity[Y] = Eigen::Vector3f(0, 0, 0);
                        grids[x][y][z].prev_velocity[Y] = 0;
                        //grids[x][y][z].prev_velocity[Y] = -0.5f* grids[x][y][z].prev_velocity[Y];
                    }
    }
}

void Fluid::projection()
{
    // p - u0 = last_v_x
    // div - v0 = last_v_y


    //for (int nd = 0; nd < NDIM; nd++)
    for (int x = 1; x < gridsidecount - 1; x++)
    {
        for (int y = 1; y < gridsidecount - 1; y++)
        {
            for (int z = 1; z < gridsidecount - 1; z++)
            {
                grids[x][y][z].prev_velocity[Y] = -1.0 / 3.0* gridsidecount*(
                    (grids[x + 1][y][z].velocity[X] - grids[x - 1][y][z].velocity[X]) +
                    (grids[x][y + 1][z].velocity[Y] - grids[x][y - 1][z].velocity[Y]) +
                    (grids[x][y][z + 1].velocity[Z] - grids[x][y][z - 1].velocity[Z]));
                grids[x][y][z].prev_velocity[X] = 0;
            }
        }
    }

    setbound(PREVX);
    setbound(PREVY);

    lin_solve_projection(1, 6);
    //for (int nd = 0; nd < NDIM; nd++)

    for (int x = 1; x < gridsidecount - 1; x++)
    {
        for (int y = 1; y < gridsidecount - 1; y++)
        {
            for (int z = 1; z < gridsidecount - 1; z++)
            {
                grids[x][y][z].velocity[X] -= 0.5f* (grids[x + 1][y][z].prev_velocity[X] - grids[x - 1][y][z].prev_velocity[X]) / gridsidecount;
                grids[x][y][z].velocity[Y] -= 0.5f* (grids[x][y + 1][z].prev_velocity[X] - grids[x][y - 1][z].prev_velocity[X]) / gridsidecount;
                grids[x][y][z].velocity[Z] -= 0.5f* (grids[x][y][z + 1].prev_velocity[X] - grids[x][y][z - 1].prev_velocity[X]) / gridsidecount;
            }
        }
    }
    setbound(F_VELOCITY);
}

float Fluid::interpolatev(Eigen::Vector3f difference, int nd)
{
    // Trilinear interpolation velocity
    int x0 = floor(difference.x());
    int y0 = floor(difference.y());
    int z0 = floor(difference.z());

    float xd = abs(difference.x() - x0);
    float yd = abs(difference.y() - y0);
    float zd = abs(difference.z() - z0);

    x0 = std::max(0, x0);
    y0 = std::max(0, y0);
    z0 = std::max(0, z0);
    x0 = std::min(x0, gridsidecount - 1);
    y0 = std::min(y0, gridsidecount - 1);
    z0 = std::min(z0, gridsidecount - 1);

    int x1 = std::min(x0 + 1, gridsidecount - 1);
    int y1 = std::min(y0 + 1, gridsidecount - 1);
    int z1 = std::min(z0 + 1, gridsidecount - 1);

    float c00 = grids[x0][y0][z0].prev_velocity[nd] * (1 - xd) + grids[x1][y0][z0].prev_velocity[nd] * xd;
    float c01 = grids[x0][y0][z1].prev_velocity[nd] * (1 - xd) + grids[x1][y0][z1].prev_velocity[nd] * xd;
    float c10 = grids[x0][y1][z0].prev_velocity[nd] * (1 - xd) + grids[x1][y1][z0].prev_velocity[nd] * xd;
    float c11 = grids[x0][y1][z1].prev_velocity[nd] * (1 - xd) + grids[x1][y1][z1].prev_velocity[nd] * xd;

    float c0 = c00 * (1 - yd) + c10 * yd;
    float c1 = c01*(1 - yd) + c11 * yd;
    float c = c0*(1 - zd) + c1*zd;

    return c;
}

float Fluid::interpolated(Eigen::Vector3f difference)
{
    // Trilinear interpolation velocity
    int x0 = floor(difference.x());
    int y0 = floor(difference.y());
    int z0 = floor(difference.z());

    float xd = abs(difference.x() - x0);
    float yd = abs(difference.y() - y0);
    float zd = abs(difference.z() - z0);

    x0 = std::max(0, x0);
    y0 = std::max(0, y0);
    z0 = std::max(0, z0);
    x0 = std::min(x0, gridsidecount - 1);
    y0 = std::min(y0, gridsidecount - 1);
    z0 = std::min(z0, gridsidecount - 1);

    int x1 = std::min(x0 + 1, gridsidecount - 1);
    int y1 = std::min(y0 + 1, gridsidecount - 1);
    int z1 = std::min(z0 + 1, gridsidecount - 1);

    float c00 = grids[x0][y0][z0].prev_density * (1 - xd) + grids[x1][y0][z0].prev_density * xd;
    float c01 = grids[x0][y0][z1].prev_density * (1 - xd) + grids[x1][y0][z1].prev_density * xd;
    float c10 = grids[x0][y1][z0].prev_density * (1 - xd) + grids[x1][y1][z0].prev_density * xd;
    float c11 = grids[x0][y1][z1].prev_density * (1 - xd) + grids[x1][y1][z1].prev_density * xd;

    float c0 = c00 * (1 - yd) + c10 * yd;
    float c1 = c01*(1 - yd) + c11 * yd;
    float c = c0*(1 - zd) + c1*zd;

    return c;
}

void Fluid::lin_solve_diffusion(float a, float c, FIELD fd)
{
    if (fd == F_VELOCITY)
    {
        for (int nd = 0; nd < NDIM; nd++)
        {
            for (int l = 0; l < LINEARITER; l++)
                for (int x = 1; x < gridsidecount - 1; x++)
                    for (int y = 1; y < gridsidecount - 1; y++)
                        for (int z = 1; z < gridsidecount - 1; z++)
                            // iterate the solver
                            // update for each cell
                            grids[x][y][z].velocity[nd] = (grids[x][y][z].prev_velocity[nd] +
                                a*(grids[x - 1][y][z].velocity[nd] +
                                    grids[x + 1][y][z].velocity[nd] +
                                    grids[x][y - 1][z].velocity[nd] +
                                    grids[x][y + 1][z].velocity[nd] +
                                    grids[x][y][z - 1].velocity[nd] +
                                    grids[x][y][z + 1].velocity[nd])) / c;
            setbound(fd, nd);
        }
    }
    else if (fd == F_DENSITY)
    {
        for (int l = 0; l < LINEARITER; l++)
            for (int x = 1; x < gridsidecount - 1; x++)
                for (int y = 1; y < gridsidecount - 1; y++)
                    for (int z = 1; z < gridsidecount - 1; z++)
                        // iterate the solver
                        // update for each cell
                        grids[x][y][z].density = (grids[x][y][z].prev_density +
                            a*(grids[x - 1][y][z].density +
                                grids[x + 1][y][z].density +
                                grids[x][y - 1][z].density +
                                grids[x][y + 1][z].density +
                                grids[x][y][z - 1].density +
                                grids[x][y][z + 1].density)) / c;
        setbound(fd);
    }
}

void Fluid::lin_solve_projection(float a, float c)
{
    for (int l = 0; l < LINEARITER; l++)
    {
        for (int x = 1; x < gridsidecount - 1; x++)
        {
            for (int y = 1; y < gridsidecount - 1; y++)
            {
                for (int z = 1; z < gridsidecount - 1; z++)
                {
                    // iterate the solver
                    // update for each cell
                    grids[x][y][z].prev_velocity[X] = (grids[x][y][z].prev_velocity[Y] +
                        a*(grids[x - 1][y][z].prev_velocity[X] +
                            grids[x + 1][y][z].prev_velocity[X] +
                            grids[x][y - 1][z].prev_velocity[X] +
                            grids[x][y + 1][z].prev_velocity[X] +
                            grids[x][y][z - 1].prev_velocity[X] +
                            grids[x][y][z + 1].prev_velocity[X])) / c;
                }
            }
        }
        setbound(PREVX);
    }
}

int Fluid::get_number_of_grid()
{
    return gridsidecount * gridsidecount*gridsidecount;
}

ms::MatrixXu Fluid::get_indices()
{
    return indices;
}

Eigen::MatrixXf Fluid::get_positions()
{
    int counter = 0;
    for (int i = 0; i < gridsidecount; i++)
        for (int j = 0; j < gridsidecount; j++)
            for (int k = 0; k < gridsidecount; k++)
            {
                counter++;
                //positions.col(counter++);// keep
                Eigen::Vector3f endp = grids[i][j][k].visvelocity();
                positions.col(counter++) << endp.x(), endp.y(), endp.z();
            }

    return positions;
}

void Fluid::update_velocity_field(std::vector<Eigen::Vector3f> p, std::vector<Eigen::Vector3f> v)
{
    for (int i = 0; i < gridsidecount; i++)
    {
        for (int j = 0; j < gridsidecount; j++)
        {
            for (int k = 0; k < gridsidecount; k++)
            {
                for (int n = 0; n < p.size(); n++)
                {
                    auto center = grids[i][j][k].center;
                    float dis = (center - p[n]).norm();
                    float weight = Gaussian(dis);
                    grids[i][j][k].velocity[X] += v[n][0] * weight;
                    grids[i][j][k].velocity[Y] += v[n][1] * weight;
                    grids[i][j][k].velocity[Z] += v[n][2] * weight;
                }
            }
        }
    }
}

