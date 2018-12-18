
#version 330
uniform mat4 MV;
uniform mat4 P;

in vec3 position;
in vec3 normal;
in vec3 vec_colors;

out vec3 fcolor;
out vec3 fnormal;
out vec3 view_dir;
out vec3 light_dir;

void main() {
    vec4 vpoint_mv = MV * vec4(position, 1.0);
    gl_Position = P * vpoint_mv;
    fcolor = vec_colors;
    fnormal = mat3(transpose(inverse(MV))) * normal;
    light_dir = vec3(0.0, 30.0, 0.0) - vpoint_mv.xyz;
    view_dir = -vpoint_mv.xyz;
}