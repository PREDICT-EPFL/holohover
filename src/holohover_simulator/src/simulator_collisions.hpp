#ifndef HOLOHOVER_SIMULATOR_COLLISIONS_HPP
#define HOLOHOVER_SIMULATOR_COLLISIONS_HPP

// Contacts of the puck with the walls and the hovercraft.
//
// Model of each contact, node parameters wall_collision_model and hovercraft_collision_model (config/*.yaml):
//   box2d:                Box2D's own contact solver with the fixture values
//   elastic:              v_n' = -e v_n, tangential velocity and spin unchanged
//   friction_restitution: independent friction-restitution impact (normal restitution e_n, Coulomb friction mu,
//                         tangential restitution e_t), the model fitted by identification/ (see model.py there)
//
// The same impact law serves both contacts: it is solved in the frame of the surface that is hit, moving with the
// velocity of its contact point (zero for a wall). There the hovercraft is a stationary wall and the puck carries
// all the energy; the hovercraft (~100x heavier than the 3.85 g puck) gets the opposite impulse back.

#include <box2d/box2d.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

enum class CollisionModel { box2d, elastic, friction_restitution };

struct CollisionParams {
    CollisionModel model = CollisionModel::elastic;
    double restitution = 0.9;                // elastic
    double e_n = 0.85, mu = 0.2, e_t = 0.3;  // friction_restitution
};

inline CollisionModel parse_collision_model(const std::string &model)
{
    if (model == "box2d") return CollisionModel::box2d;
    if (model == "elastic") return CollisionModel::elastic;
    if (model == "friction_restitution") return CollisionModel::friction_restitution;
    throw std::invalid_argument("collision model must be box2d, elastic or friction_restitution, not '" + model + "'");
}

// Parameters <contact>_collision_model, elastic.<contact>.restitution, friction_restitution.<contact>.{e_n,mu,e_t}
inline CollisionParams load_collision_params(rclcpp::Node &node, const std::string &contact,
                                             const std::string &default_model, double default_restitution)
{
    CollisionParams c;
    const auto model = node.declare_parameter<std::string>(contact + "_collision_model", default_model);
    c.model = parse_collision_model(model);
    c.restitution = node.declare_parameter<double>("elastic." + contact + ".restitution", default_restitution);
    c.e_n = node.declare_parameter<double>("friction_restitution." + contact + ".e_n", c.e_n);
    c.mu = node.declare_parameter<double>("friction_restitution." + contact + ".mu", c.mu);
    c.e_t = node.declare_parameter<double>("friction_restitution." + contact + ".e_t", c.e_t);
    RCLCPP_INFO(node.get_logger(), "Puck-%s collision model: %s", contact.c_str(), model.c_str());
    return c;
}

// Velocity v and spin w of the puck after an impact, v relative to the surface that is hit. n: surface normal
// towards the puck, r: puck radius, kappa = I / (m r^2). Same law as impact() in identification/model.py
inline void friction_restitution_impact(b2Vec2 &v, float &w, b2Vec2 n, float r, float kappa, const CollisionParams &c)
{
    const b2Vec2 t(-n.y, n.x);  // z x n
    const float vn = b2Dot(v, n), vt = b2Dot(v, t);
    if (vn >= 0) return;
    const float ut = vt - r * w;  // slip of the contact point
    const float friction = std::min(std::abs(ut), float(c.mu * (1 + c.e_n) * -vn * (1 + kappa) / kappa));
    const float du = -std::copysign(1.0f, ut) * float(1 + c.e_t) * friction;
    v = float(-c.e_n * vn) * n + (vt + kappa / (1 + kappa) * du) * t;
    w -= du / (r * (1 + kappa));
}

// Bounce of the puck on a surface with normal n (towards the puck) whose contact point moves with surface_v.
// Returns the impulse given to the puck (zero when it is not moving into the surface)
inline b2Vec2 bounce_puck(b2Body *puck, b2Vec2 n, b2Vec2 surface_v, const CollisionParams &c)
{
    const b2Vec2 v0 = puck->GetLinearVelocity();
    b2Vec2 v = v0 - surface_v;  // in the frame of the surface
    const float vn = b2Dot(v, n);
    if (vn >= 0) return b2Vec2_zero;
    float w = puck->GetAngularVelocity();
    if (c.model == CollisionModel::elastic) {
        v -= float(1 + c.restitution) * vn * n;
    } else {
        const float r = puck->GetFixtureList()->GetShape()->m_radius;
        friction_restitution_impact(v, w, n, r, puck->GetInertia() / (puck->GetMass() * r * r), c);
    }
    puck->SetLinearVelocity(v + surface_v);
    puck->SetAngularVelocity(w);
    return puck->GetMass() * (v + surface_v - v0);
}

// Puck body of the simulator (fixture values of the box2d model), at rest at `position`
inline b2Body *create_puck(b2World &world, b2Vec2 position, float radius,
                           float linear_damping = 0.4f, float angular_damping = 0.5f)
{
    b2BodyDef def;
    def.type = b2_dynamicBody;
    def.linearDamping = linear_damping;
    def.angularDamping = angular_damping;
    def.position = position;
    b2Body *puck = world.CreateBody(&def);
    b2CircleShape shape;
    shape.m_radius = radius;
    b2FixtureDef fixture;
    fixture.shape = &shape;
    fixture.density = 1.0f;
    fixture.friction = 0.02f;
    fixture.restitution = 0.8f;
    // Restitution at every impact speed. Box2D's default restitutionThreshold (1 m/s, its guard against jitter in
    // resting contacts) made the hovercraft-puck contacts under 1 m/s fully inelastic (e = 0). A contact takes the
    // smaller threshold of its two fixtures, so this one covers them all.
    fixture.restitutionThreshold = 0.0f;
    puck->CreateFixture(&fixture);
    puck->GetUserData().pointer = reinterpret_cast<uintptr_t>("puck");
    return puck;
}

// Static wall of the given shape (fixture values of the box2d model)
inline b2Body *create_wall(b2World &world, const b2Shape &shape, b2Vec2 position = b2Vec2_zero)
{
    b2BodyDef def;
    def.type = b2_staticBody;
    def.position = position;
    b2Body *wall = world.CreateBody(&def);
    b2FixtureDef fixture;
    fixture.shape = &shape;
    fixture.restitution = 0.0f;
    fixture.friction = 0.05f;
    wall->CreateFixture(&fixture);
    wall->GetUserData().pointer = reinterpret_cast<uintptr_t>("wall");
    return wall;
}

// Update Sep 30: collision detector
// node: for the wall_collided / hovercraft_collided topics and the logs, nullptr for none (collision_replay)
class ContactListener : public b2ContactListener {
public:
    ContactListener(rclcpp::Node *node, CollisionParams wall, CollisionParams hovercraft)
        : node_(node), wall_(wall), hovercraft_(hovercraft)
    {
        if (!node_) return;
        wall_collided_pub_ = node_->create_publisher<std_msgs::msg::Bool>("wall_collided", 10);
        hovercraft_collided_pub_ = node_->create_publisher<std_msgs::msg::Bool>("hovercraft_collided", 10);
    }

    // elastic / friction_restitution: the bounce is set in BeginContact, Box2D only keeps the bodies apart
    void PreSolve(b2Contact *contact, const b2Manifold *) override {
        const Puck p(contact);
        if (p.params(*this) && p.params(*this)->model != CollisionModel::box2d) {
            contact->SetRestitution(0.0f);
            contact->SetFriction(0.0f);
        }
    }

    void BeginContact(b2Contact *contact) override {
        const Puck p(contact);
        if (!p.puck) return;
        const CollisionParams *c = p.params(*this);
        if (!c) {
            if (node_) RCLCPP_WARN(node_->get_logger(), "Puck collided with an unknown object.");
            return;
        }
        if (node_) RCLCPP_INFO(node_->get_logger(), "Puck collided with %s.", p.wall ? "wall" : "hovercraft");
        publish(p, true);
        if (c->model == CollisionModel::box2d) return;

        b2WorldManifold m;
        contact->GetWorldManifold(&m);
        const b2Vec2 n = p.puck_is_a ? -m.normal : m.normal;  // the manifold normal points from A to B
        const b2Vec2 point = m.points[0];
        b2Body *other = p.other->GetBody();
        const b2Vec2 impulse = bounce_puck(p.puck->GetBody(), n, other->GetLinearVelocityFromWorldPoint(point), *c);
        if (p.hovercraft)
            other->ApplyLinearImpulse(-impulse, point, true);
    }

    void EndContact(b2Contact *contact) override {
        const Puck p(contact);
        if (!p.puck) return;
        if (!p.params(*this)) {
            if (node_) RCLCPP_WARN(node_->get_logger(), "Puck stopped colliding with an unknown object.");
            return;
        }
        if (node_) RCLCPP_INFO(node_->get_logger(), "Puck stopped colliding with %s.", p.wall ? "wall" : "hovercraft");
        publish(p, false);
    }

private:
    static bool is(b2Fixture *f, const char *name) {
        const char *data = reinterpret_cast<const char *>(f->GetBody()->GetUserData().pointer);
        return data && std::string(data) == name;
    }

    // The puck and the other fixture of a contact (puck null when the puck is not in it)
    struct Puck {
        b2Fixture *puck = nullptr, *other = nullptr;
        bool puck_is_a = false, wall = false, hovercraft = false;
        explicit Puck(b2Contact *contact) {
            b2Fixture *a = contact->GetFixtureA(), *b = contact->GetFixtureB();
            if (is(a, "puck")) { puck = a; other = b; puck_is_a = true; }
            else if (is(b, "puck")) { puck = b; other = a; }
            else return;
            wall = is(other, "wall");
            hovercraft = is(other, "hovercraft");
        }
        const CollisionParams *params(const ContactListener &l) const {
            return wall ? &l.wall_ : hovercraft ? &l.hovercraft_ : nullptr;
        }
    };

    void publish(const Puck &p, bool collided) {
        if (!node_) return;
        std_msgs::msg::Bool msg;
        msg.data = collided;
        (p.wall ? wall_collided_pub_ : hovercraft_collided_pub_)->publish(msg);
    }

    rclcpp::Node *node_;
    CollisionParams wall_, hovercraft_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wall_collided_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hovercraft_collided_pub_;
};

#endif // HOLOHOVER_SIMULATOR_COLLISIONS_HPP
