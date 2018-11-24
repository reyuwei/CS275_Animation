//
// Created by stephan-lb on 23/03/2017.
//

#include "Mesh.h"

#include <iostream>
#include <algorithm>
#include <Eigen/Dense>
#include "windows.h" 
#include <fstream>

#define MIN -1e10
#define MAX 1e10

bool ExistsFile(const std::string &path) {
    std::fstream _file;
    _file.open(path, std::ios::in);
    if (!_file) {
        return false;
    }
    else {
        return true;
    }
}

Mesh::Mesh(const std::string &filename) : m_num_vertices(0), m_num_faces(0)
{
    m_bmin = Eigen::Vector3f(MAX, MAX, MAX);
    m_bmax = Eigen::Vector3f(MIN, MIN, MIN);
    m_mesh_center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    m_dist_max = 0.0f;

    load_mesh(filename);
    std::string directory = filename.substr(0, filename.find_last_of('\\'));
    load_label(directory + "\\ishair_fix.txt");
    generateHair();
}

Mesh::~Mesh() {
}

bool Mesh::load_mesh(const std::string &filename) {

    std::string err;
    tinyobj::attrib_t attrib;
    std::string base_dir = filename.substr(0, filename.find_last_of('\\') + 1);
    bool load = tinyobj::LoadObj(&attrib, &m_shapes, &m_materials, &err, filename.c_str(), base_dir.c_str(), true);

    if (!load) {
        std::cerr << err << std::endl;
        return false;
    }

    if (!m_materials.empty())
    {
        std::cerr << "We are not supporting materials for the moments" << std::endl;
    }

    std::cout << "[LoadOBJ] # of shapes in .obj : " << m_shapes.size() << std::endl;
    std::cout << "[LoadOBJ] # of materials in .obj : " << m_materials.size() << std::endl;

    m_num_vertices = (int)(attrib.vertices.size()) / 3;
    m_num_uvs = (int)(attrib.texcoords.size()) / 2;

    for (size_t i = 0; i < m_shapes.size(); i++) {
        m_num_faces += m_shapes[i].mesh.indices.size() / 3;
    }

    std::cout << "[LoadOBJ] # of faces: " << m_num_faces << std::endl;
    std::cout << "[LoadOBJ] # of vertices: " << m_num_vertices << std::endl;
    std::cout << "[LoadOBJ] # of uvs: " << m_num_uvs << std::endl;

    m_points = Eigen::MatrixXf(3, m_num_faces * 3);
    m_indices = MatrixXu(3, m_num_faces);
    m_normals = Eigen::MatrixXf(3, m_num_faces * 3);
    m_uvs = Eigen::MatrixXf(2, m_num_faces * 3);

    for (size_t s = 0; s < m_shapes.size(); s++)
    {
        for (size_t f = 0; f < m_shapes[s].mesh.indices.size() / 3; f++)
        {
            tinyobj::index_t idx0 = m_shapes[s].mesh.indices[3 * f + 0];
            tinyobj::index_t idx1 = m_shapes[s].mesh.indices[3 * f + 1];
            tinyobj::index_t idx2 = m_shapes[s].mesh.indices[3 * f + 2];

            int f0 = f * 3 + 0;
            int f1 = f * 3 + 1;
            int f2 = f * 3 + 2;

            m_indices.col(f) << f0, f1, f2;

            assert(f0 >= 0);
            assert(f1 >= 0);
            assert(f2 >= 0);

            float v[3][3];
            for (int k = 0; k < 3; k++) { // x,y,z
                v[0][k] = attrib.vertices[3 * idx0.vertex_index + k];
                v[1][k] = attrib.vertices[3 * idx1.vertex_index + k];
                v[2][k] = attrib.vertices[3 * idx2.vertex_index + k];

                m_bmin[k] = min(v[0][k], m_bmin[k]);
                m_bmin[k] = min(v[1][k], m_bmin[k]);
                m_bmin[k] = min(v[2][k], m_bmin[k]);

                m_bmax[k] = max(v[0][k], m_bmax[k]);
                m_bmax[k] = max(v[1][k], m_bmax[k]);
                m_bmax[k] = max(v[2][k], m_bmax[k]);
            }

            m_mesh_center = (m_bmax + m_bmin) / 2;
            m_dist_max = max((m_bmax - m_mesh_center).norm(), (m_bmax - m_mesh_center).norm());

            float n[3][3];
            if (attrib.normals.size() > 0) {
                for (int k = 0; k < 3; k++) {
                    n[0][k] = attrib.normals[3 * idx0.normal_index + k];
                    n[1][k] = attrib.normals[3 * idx1.normal_index + k];
                    n[2][k] = attrib.normals[3 * idx2.normal_index + k];
                }
            }
            else {
                // compute geometric normal
                Eigen::Vector3f v0, v1, v2;
                v0 << v[0][0], v[0][1], v[0][2];
                v1 << v[1][0], v[1][1], v[1][2];
                v2 << v[2][0], v[2][1], v[2][2];
                Eigen::Vector3f v10 = v1 - v0;
                Eigen::Vector3f v20 = v2 - v0;
                Eigen::Vector3f N_face = v10.cross(v20);
                N_face.normalize();
                n[0][0] = N_face[0];
                n[0][1] = N_face[1];
                n[0][2] = N_face[2];
                n[1][0] = N_face[0];
                n[1][1] = N_face[1];
                n[1][2] = N_face[2];
                n[2][0] = N_face[0];
                n[2][1] = N_face[1];
                n[2][2] = N_face[2];
            }

            float uv[3][2];
            uv[0][0] = attrib.texcoords[2 * idx0.texcoord_index];
            uv[1][0] = attrib.texcoords[2 * idx1.texcoord_index];
            uv[2][0] = attrib.texcoords[2 * idx2.texcoord_index];
            uv[0][1] = 1 - attrib.texcoords[2 * idx0.texcoord_index + 1];
            uv[1][1] = 1 - attrib.texcoords[2 * idx1.texcoord_index + 1];
            uv[2][1] = 1 - attrib.texcoords[2 * idx2.texcoord_index + 1];

            //////////////////////////////////////////////////////////////////////////
            m_uvs.col(f0) << uv[0][0], uv[0][1];
            m_uvs.col(f1) << uv[1][0], uv[1][1];
            m_uvs.col(f2) << uv[2][0], uv[2][1];

            m_points.col(f0) << v[0][0], v[0][1], v[0][2];
            m_points.col(f1) << v[1][0], v[1][1], v[1][2];
            m_points.col(f2) << v[2][0], v[2][1], v[2][2];

            m_normals.col(f0) << n[0][0], n[0][1], n[0][2];
            m_normals.col(f1) << n[1][0], n[1][1], n[1][2];
            m_normals.col(f2) << n[2][0], n[2][1], n[2][2];
        }
    }

    // Load diffuse textures
    for (size_t m = 0; m < m_materials.size(); m++) {
        tinyobj::material_t* mp = &m_materials[m];
        if (mp->diffuse_texname.length() > 0) {
            GLuint texture_id = 0;
            int w, h;
            int comp;
            std::string texture_filename = mp->diffuse_texname;
            m_texture = GLTexture(texture_filename, texture_id);
            auto data = m_texture.load((base_dir + texture_filename).c_str());
        }
    }

    return true;
}

bool Mesh::load_label(const std::string &filename)
{
    std::ifstream infile;
    infile.open(filename);
    if (!infile.is_open())
        return false;

    ishair.clear();
    for (int i = 0; i < m_num_faces * 3; i++)
        ishair.push_back(999);

    std::string s;
    int counter = 0;
    hair_num = 0;
    while (getline(infile, s))
    {
        float label = -1;
        std::stringstream stream(s);
        stream >> label;
        ishair[counter] = label;
        counter++;
        if (label == 1)
            hair_num++;
    }
    infile.close();
    return true;
#ifdef FIX_LABEL
    std::vector<std::vector<float>> ishair_entry;

    std::string s;
    int counter = 0;
    while (getline(infile, s))
    {
        std::vector<float> vertex_entry;
        do
        {
            std::string number_s = s.substr(0, s.find_first_of(" "));
            float number = -1;
            std::stringstream stream(number_s);
            stream >> number;
            vertex_entry.push_back(number);
            s.erase(0, s.find_first_of(" ") + 1);
        } while (vertex_entry.size() < 4);

        ishair_entry.push_back(vertex_entry);

        counter++;
        if (counter % 1000 == 0)
            std::cout << counter << std::endl;
    }
    infile.close();

    for (int i = 0; i < m_num_faces * 3; i++)
    {
        float mindis = 1e10;
        int target_label = -1;
        for (int j = 0; j < ishair_entry.size(); j++)
        {
            float point_dis = pow(m_points(0, i) - ishair_entry[j][0], 2) +
                pow(m_points(1, i) - ishair_entry[j][1], 2) +
                pow(m_points(2, i) - ishair_entry[j][2], 2);
            if (point_dis < mindis)
            {
                mindis = point_dis;
                target_label = j;
            }
        }
        //std::cout << mindis << std::endl;
        ishair[i] = ishair_entry[target_label][3];
    }


    std::ofstream outfile;
    outfile.open(filename + "x");
    if (!outfile.is_open())
        return false;

    for (int i = 0; i < ishair.size(); i++)
        outfile << ishair[i] << std::endl;
    outfile.close();
    return true;

#endif

}

unsigned int Mesh::get_number_of_face() {
    return m_num_faces;
}

const Eigen::Vector3f Mesh::get_mesh_center() {
    return m_mesh_center;
}

const Eigen::MatrixXf* Mesh::get_points() {
    return &m_points;
}

const MatrixXu *Mesh::get_indices() {
    return &m_indices;
}

const Eigen::MatrixXf *Mesh::get_normals() {
    return &m_normals;
}

const Eigen::MatrixXf *Mesh::get_uvs()
{
    return &m_uvs;
}

const Eigen::MatrixXf *Mesh::get_hairpos()
{
    m_hair = Eigen::MatrixXf(3, hair_num * 2);
    int counter = 0;
    for (int i = 0; i < ishair.size(); i++)
    {
        if (ishair[i] == 1)
        {
            m_hair.col(2 * counter + 0) << m_points.col(i).x(), m_points.col(i).y(), m_points.col(i).z();
            auto offset = m_points.col(i) + m_normals.col(i) * 50.0;
            m_hair.col(2 * counter + 1) << offset.x(), offset.y(), offset.z();
            counter++;
        }
    }
    return &m_hair;
}

const MatrixXu *Mesh::get_hairindices()
{
    m_hairindices = MatrixXu(2, hair_num);
    for (int i = 0; i < hair_num; i++)
    {
        m_hairindices.col(i) << 2 * i + 0, 2 * i + 1;
    }
    return &m_hairindices;
}


float Mesh::get_dist_max()
{
    return m_dist_max;
}

GLTexture* Mesh::get_texture()
{
    return &m_texture;
}

void Mesh::generateHair()
{
    Eigen::MatrixXf hairpoints = Eigen::MatrixXf(3, hair_num);
    int counter = 0;
    for (int i = 0; i < ishair.size(); i++)
    {
        if (ishair[i] == 1)
            hairpoints.col(counter++) << m_points.col(i).x(), m_points.col(i).y(), m_points.col(i).z();
    }
    hairs = Hair(hairpoints, 20, 10, 10);
}

int Mesh::get_number_of_hair()
{
    return hair_num;
}