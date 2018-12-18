#version 330

in vec3 fcolor;
in vec3 fnormal;
in vec3 view_dir;
in vec3 light_dir;

out vec4 color;

vec3 kajiyaKay(vec3 V, vec3 L, vec3 T) {                                  
float TdotL = dot(T, L);                                                  
float TdotV = dot(T, V);                                                  
vec3 specolor = vec3(1.0)*vec3(0.18, 0.1, 0.1);                           
// The diffuse component                                                  
float kajiyaDiff = sin(acos(TdotL));                                      
kajiyaDiff = pow(max(kajiyaDiff,0.0), 10);                                
                                                                            
// The specular component                                                 
float kajiyaSpec = cos(abs(acos(TdotL) - acos(-TdotV)));                  
kajiyaSpec = pow(max(0.0,kajiyaSpec), 100);                               
                                                                            
// The fragment color                                                     
vec3 outcolor = fcolor * 0.6 + fcolor * 0.5 * kajiyaDiff + specolor * kajiyaSpec;
return outcolor;                                                          
}                                                                         


void main() {
    vec3 n = normalize(fnormal);
    vec3 v = normalize(view_dir);
    vec3 l = normalize(light_dir);
    vec3 kjk = kajiyaKay(v,l,n);
    color = vec4(kjk, 0.725);
}