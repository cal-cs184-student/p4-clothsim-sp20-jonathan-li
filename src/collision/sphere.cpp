#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
    Vector3D direction = pm.position - origin;
    double len = direction.norm();
    direction.normalize();
    
    double scaling = (1.0 - friction);
    if (len <= radius) {
        Vector3D tangent = origin + direction * radius;
        Vector3D correction = tangent - pm.last_position;
        pm.position = pm.last_position + scaling * correction;
    }

}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
