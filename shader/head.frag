#version 330

in vec2 UV;
in vec3 fnormal;
in vec3 view_dir;
in vec3 light_dir;

out vec4 color;

// Values that stay constant for the whole mesh.

uniform sampler2D tex;

void main() {
vec4 color_uv = texture( tex, UV ).rgba;
vec3 c = vec3(0.0);
c += vec3(1.0)*vec3(0.18, 0.1, 0.1);
vec3 n = normalize(fnormal);
vec3 v = normalize(view_dir);
vec3 l = normalize(light_dir);
float lambert = dot(n,l);
if(lambert > 0.0) {
    c += vec3(lambert);
    vec3 v = normalize(view_dir);
    vec3 r = reflect(-l,n);
    c += vec3(pow(max(dot(r,v), 0.0), 90.0));
}
color = vec4(c, 1.0);
color *= color_uv;
}