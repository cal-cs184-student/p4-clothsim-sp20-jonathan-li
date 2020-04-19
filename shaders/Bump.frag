#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  return texture(u_texture_2,uv).r;
}

void main() {
    float ka = 0.2;
    float ia = 1;
    float kd = 0.1;
    float ks = 0.1;
    float p = 4.0;
    
    float u = v_uv.x;
    float v = v_uv.y;
    float w = u_texture_2_size.x;
    mat3 tbn = mat3(v_tangent.xyz, cross(v_normal.xyz, v_tangent.xyz), v_normal.xyz);
    float du = (h(vec2(u + 1 / w, v)) - h(vec2(u, v))) * u_normal_scaling * u_height_scaling;
    float dv = (h(vec2(u, v + 1 / w)) - h(vec2(u, v))) * u_normal_scaling * u_height_scaling;
    vec3 nd = tbn * vec3(-du, -dv, 1);
    nd = normalize(nd);
    
    vec3 l = u_light_pos - v_position.xyz;
    vec3 IoverR2 = u_light_intensity / dot(l, l);
    vec3 h = (l + (u_cam_pos - v_position.xyz)) / 2;
    // (Placeholder code. You will want to replace it.)
    out_color.xyz = (ia * ka) + (kd * IoverR2 * max(0.0, dot(nd, l))) + (ks * IoverR2 * pow(max(0.0, dot(nd, h)), p));
    out_color.a = 1;
}

