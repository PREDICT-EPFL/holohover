// Offline run of the simulator's puck physics: the same Box2D puck, walls and collision models
// (simulator_collisions.hpp) as the simulator node, without ROS and as fast as possible.
// Used by scripts/impact_comparison.py to replay recorded wall impacts.
//
// Input (YAML, stdin):
//   model: box2d | elastic | friction_restitution
//   restitution: 0.88                    # elastic
//   e_n: 0.88  mu: 0.07  e_t: 0.46       # friction_restitution
//   radius: 0.035
//   linear_damping: 0.4  angular_damping: 0.5  gravity: [gx, gy]   # free flight (Box2D damping ~ exp(-c t))
//   period: 0.01  substeps: 10  iterations: [6, 2]  duration: 0.6
//   state: [x, y, yaw, vx, vy, w]
//   walls: [{normal: [nx, ny], offset: o}]   # identification walls: n into the table, o = n . (puck centre) at contact
// Output (CSV, stdout): t,x,y,yaw,vx,vy,w every period, from t = 0

#include "simulator_collisions.hpp"

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <vector>

int main()
{
    const YAML::Node c = YAML::Load(std::cin);
    CollisionParams p;
    p.model = parse_collision_model(c["model"].as<std::string>());
    p.restitution = c["restitution"].as<double>(p.restitution);
    p.e_n = c["e_n"].as<double>(p.e_n);
    p.mu = c["mu"].as<double>(p.mu);
    p.e_t = c["e_t"].as<double>(p.e_t);

    const auto g = c["gravity"].as<std::vector<float>>(std::vector<float>{0, 0});
    b2World world(b2Vec2(g[0], g[1]));
    const float r = c["radius"].as<float>();
    for (const auto &w : c["walls"]) {
        const auto nv = w["normal"].as<std::vector<float>>();
        b2Vec2 n(nv[0], nv[1]);
        n.Normalize();
        // the puck centre touches at n . x = offset: the edge is r and its skin (b2_polygonRadius) further out
        const b2Vec2 t(-n.y, n.x), p0 = (w["offset"].as<float>() - r - b2_polygonRadius) * n;
        b2EdgeShape edge;
        edge.SetTwoSided(p0 - 10.0f * t, p0 + 10.0f * t);
        create_wall(world, edge);
    }

    const auto s = c["state"].as<std::vector<double>>();
    b2Body *puck = create_puck(world, b2Vec2(s[0], s[1]), r, c["linear_damping"].as<float>(0.4f),
                               c["angular_damping"].as<float>(0.5f));
    puck->SetTransform(b2Vec2(s[0], s[1]), s[2]);
    puck->SetLinearVelocity(b2Vec2(s[3], s[4]));
    puck->SetAngularVelocity(s[5]);
    ContactListener listener(nullptr, p, p);
    world.SetContactListener(&listener);

    const double period = c["period"].as<double>(0.01);
    const int substeps = c["substeps"].as<int>(10);
    const auto it = c["iterations"].as<std::vector<int>>(std::vector<int>{6, 2});
    const int steps = int(c["duration"].as<double>() / period + 0.5);
    std::cout << "t,x,y,yaw,vx,vy,w\n";
    for (int k = 0; k <= steps; k++) {
        const b2Vec2 x = puck->GetPosition(), v = puck->GetLinearVelocity();
        std::cout << k * period << ',' << x.x << ',' << x.y << ',' << puck->GetAngle() << ',' << v.x << ',' << v.y
                  << ',' << puck->GetAngularVelocity() << '\n';
        for (int j = 0; j < substeps; j++)
            world.Step(period / substeps, it[0], it[1]);
    }
    world.SetContactListener(nullptr);
}
