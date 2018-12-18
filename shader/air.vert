#version 330 core

// In data
// Index to pick the current particle
in vec3 position;

// Uniform data
// Transform matrices
uniform mat4 MV;
uniform mat4 P;

void main(){
	// Set camera position
	vec4 vpoint_mv = MV * vec4(position, 1.0);
    gl_Position = P * vpoint_mv;

	// Position and size of point
	// gl_PointSize = 10;
}