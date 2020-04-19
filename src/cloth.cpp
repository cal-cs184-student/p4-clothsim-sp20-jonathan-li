#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
    for (int j=0; j<num_height_points; j++) {
        for (int i=0; i<num_width_points; i++) {
            double w = width/num_width_points;
            double h = height/num_height_points;
            Vector3D pos = Vector3D();
            
            double xpos = w*i;
            double ypos = h*j;
            if (orientation == HORIZONTAL) { //horizontal
                pos = Vector3D(xpos, 1.0, ypos);
                PointMass point = PointMass(pos, false);
                point_masses.push_back(point);
            } else if (orientation == VERTICAL) { //vertical
                double r = ((double) rand() / (double) RAND_MAX);
                double b = 1.0/1000.0;
                double a = -1.0/1000.0;
                double zpos = a + (r * (b-a));
                pos = Vector3D(xpos, ypos, zpos);
                PointMass point = PointMass(pos, false);
                point_masses.push_back(point);
            }
        }
    }
    for (int k=0; k<pinned.size(); k++) {
        point_masses[pinned[k][1] * num_width_points + pinned[k][0]].pinned = true;
    }
    for (int j=0; j<num_height_points; j++) { //create springs
        for (int i=0; i<num_width_points; i++) {
            int pmass = j * num_width_points + i;
            int connect;
            if (i - 1 >= 0) {
                connect = j * num_width_points + (i - 1);
                springs.push_back(Spring(&point_masses[pmass], &point_masses[connect], STRUCTURAL));
            }
            if (j - 1 >= 0) {
                connect = (j - 1) * num_width_points + i;
                springs.push_back(Spring(&point_masses[pmass], &point_masses[connect], STRUCTURAL));
            }
            if (j - 1 >= 0 && i - 1 >= 0) {
                connect = (j - 1) * num_width_points + (i - 1);
                springs.push_back(Spring(&point_masses[pmass], &point_masses[connect], SHEARING));
            }
            if (j - 1 >= 0 && i + 1 < num_width_points) {
                connect = (j - 1) * num_width_points + (i + 1);
                springs.push_back(Spring(&point_masses[pmass], &point_masses[connect], SHEARING));
            }
            if (i - 2 >= 0) {
                connect = j * num_width_points + (i - 2);
                springs.push_back(Spring(&point_masses[pmass], &point_masses[connect], BENDING));
            }
            if (j - 2 >= 0) {
                connect = (j - 2) * num_width_points + i;
                springs.push_back(Spring(&point_masses[pmass], &point_masses[connect], BENDING));
            }
        }
    }

}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

    

  // TODO (Part 2): Compute total force acting on each point mass.
    Vector3D tef = Vector3D();
    for (Vector3D &a : external_accelerations)
      tef += a * mass;
    for (int i=0; i<point_masses.size(); i++) {
        point_masses[i].forces = tef;
    }
    for (Spring &spring : springs) {
        if (spring.spring_type == STRUCTURAL or spring.spring_type == BENDING or spring.spring_type == SHEARING) {
            if (cp->enable_structural_constraints) {
                Vector3D direction = spring.pm_b->position - spring.pm_a->position;
                double diff = direction.norm();
                direction.normalize();
                Vector3D F = 0.2 * cp->ks * (diff - spring.rest_length) * direction;
    
                spring.pm_a->forces += F;
                spring.pm_b->forces += -F;
            }
        }
    }

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
    for (PointMass &p : point_masses) {
        if (not p.pinned) {
            Vector3D a = p.forces / mass;
            double d = cp->damping / 100.0;
            Vector3D newp = p.position + (1.0 - d) * (p.position - p.last_position) + a * pow(delta_t, 2);
            p.last_position = p.position;
            p.position = newp;
        }
    }

  // TODO (Part 4): Handle self-collisions.
    build_spatial_map();
    for (int i=0; i<point_masses.size(); i++) {
        self_collide(point_masses[i], simulation_steps);
     }

  // TODO (Part 3): Handle collisions with other primitives.
    for (int i=0; i<point_masses.size(); i++) {
        for (CollisionObject *object : *collision_objects) {
            object->collide(point_masses[i]);
        }
    }

  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
    for (Spring &spring : springs) {
        Vector3D direction = spring.pm_b->position - spring.pm_a->position;
        double diff = direction.norm();
        direction.normalize();
        if (diff - 0.1*spring.rest_length>spring.rest_length) {
            double over = diff - 1.1 * spring.rest_length;
            if (!spring.pm_a->pinned && !spring.pm_b->pinned) {
                spring.pm_a->position += direction * (over / 2);
                spring.pm_b->position -= direction * (over / 2);
            } else if (spring.pm_a->pinned && !spring.pm_b->pinned) {
                spring.pm_b->position -= direction * over;
            } else if (!spring.pm_a->pinned && spring.pm_b->pinned) {
                spring.pm_a->position += direction * over;
            }
        }
    }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
    for (int i=0; i<point_masses.size(); i++) {
        double pos = hash_position(point_masses[i].position);
        if (map.find(pos) == map.end()) {
            
            map[pos] = new vector<PointMass *>;
        }
        
        map[pos]->push_back(&point_masses[i]);
    }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
    if (pm.pinned) return;
    Vector3D correction = Vector3D();
    double pos = hash_position(pm.position);
    int count = 0;
    vector<PointMass *> *points = map[pos];
    vector<PointMass *>::iterator p, end;
    for (p=(*points).begin(), end=(*points).end(); p!=end; ++p) {
        if (!((*p)->position == pm.position)) {
            Vector3D direction = pm.position - (*p)->position;
            double dist = direction.norm();
            direction.normalize();
            
            if (dist < 2*thickness) {
                correction += direction * (2*thickness - dist);
                count++;
            }
        }
    }
    if (count) {
        pm.position += correction / count / simulation_steps;
    }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
    double w = 3.0 * width / num_width_points;
    double h = 3.0 * height / num_height_points;
    
    double t = max(w, h);
    
    
    double x = (pos.x - fmod(pos.x, w));
    double y = (pos.y - fmod(pos.y, h));
    double z = (pos.z - fmod(pos.z, t));
    
    double cantor = 0.5*(x + y)*(x + y + 1) + y;
    return 0.5*(cantor + z)*(cantor + z + 1) + z;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B,
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D,
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
