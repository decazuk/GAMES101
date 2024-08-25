#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        if (num_nodes == 1) {
            masses.push_back(new Mass(start, node_mass, false));
        } else {
            double stepX = (end.x - start.x) / (num_nodes - 1);
            double stepY = (end.y - start.y) / (num_nodes - 1);
            for (int i = 0; i < num_nodes; i++) {
                Mass * newMass = new Mass(Vector2D(start.x + i * stepX, start.y + i * stepY), node_mass, false);
                masses.push_back(newMass);
            }
            for (int i = 1; i < num_nodes; i++) {
                Spring * spring = new Spring(masses[i-1], masses[i], k);
                springs.push_back(spring);
            }
        }
        
//        Comment-in this part when you implement the constructor
    //    for (auto &i : pinned_nodes) {
    //        masses[i]->pinned = true;
    //    }
        masses[0]->pinned = true;
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Mass * m1 = s->m1;
            Mass * m2 = s->m2;
            Vector2D p21 = m2->position - m1->position;
            Vector2D f1 = s->k * p21 / p21.norm() * (p21.norm() - s->rest_length);
            Vector2D f2 = -f1;
            m1->forces += f1;
            m2->forces += f2;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                Vector2D accelerateSpeed = m->forces / m->mass + gravity;
                Vector2D vt1 = m->velocity + accelerateSpeed * delta_t;

                // explicit eula method will diverge
                // Vector2D xt1 = m->position + m->velocity * delta_t;

                // semi-implicit eula method is ok 
                Vector2D xt1 = m->position + vt1 * delta_t;
                // // TODO (Part 2): Add global damping
                float damping_factor = 0.00005;
                m->velocity = vt1 * (1 - damping_factor);
                m->position = xt1;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Mass * m1 = s->m1;
            Mass * m2 = s->m2;
            double length = (m1->position - m2->position).norm();
            if (length != s->rest_length) {
                if (m1->pinned) {
                    Vector2D p12 = (m1->position - m2->position);
                    Vector2D p12normalize = p12 / p12.norm();
                    m2->position += ((length - s->rest_length) * p12normalize);
                } else if (m2->pinned) {
                    Vector2D p21 = (m2->position - m1->position);
                    Vector2D p21normalize = p21 / p21.norm();
                    m1->position += ((length - s->rest_length) * p21normalize);
                } else {
                    Vector2D p21 = (m2->position - m1->position);
                    Vector2D p21normalize = p21 / p21.norm();
                    m1->position += 0.5 * ((length - s->rest_length) * p21normalize);
                    m2->position -= 0.5 * ((length - s->rest_length) * p21normalize);
                }
            }
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                Vector2D last_position = m->last_position;
                float damping_factor = 0.00005;
                Vector2D x1 = temp_position + (1 - damping_factor) * (temp_position - last_position) + gravity * delta_t * delta_t;
                // TODO (Part 4): Add global Verlet damping
                m->last_position = temp_position;
                m->position = x1;
            }
        }
    }
}
