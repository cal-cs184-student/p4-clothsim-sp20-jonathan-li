#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
    float ka = 0.2;
    float ia = 1;
    float kd = 0.1;
    float ks = 0.1;
    float p = 4.0;
    
    vec3 l = u_light_pos - v_position.xyz;
    vec3 IoverR2 = u_light_intensity / dot(l, l);
    vec3 h = (l + (u_cam_pos - v_position.xyz)) / 2;
    // (Placeholder code. You will want to replace it.)
    out_color.xyz = (ia * ka) + (kd * IoverR2 * max(0.0, dot(v_normal.xyz, l))) + (ks * IoverR2 * pow(max(0.0, dot(v_normal.xyz, h)), p));
    out_color.a = 1;
}

