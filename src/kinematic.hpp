#pragma once

/**
 * Kinematic Tree - Server-side calculations
 * C++ port of kinematic.py
 */

#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <nos/trent/trent.h>

namespace webkin
{

struct Vec3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3 &other) const
    {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }

    Vec3 operator*(double scalar) const
    {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }

    nos::trent to_trent() const
    {
        nos::trent t;
        t.init(nos::trent::type::list);
        t.push_back(x);
        t.push_back(y);
        t.push_back(z);
        return t;
    }

    static Vec3 from_trent(const nos::trent &t)
    {
        if (!t.is_list() || t.as_list().size() < 3)
            return Vec3();
        return Vec3(
            t[0].as_numer_default(0),
            t[1].as_numer_default(0),
            t[2].as_numer_default(0));
    }
};

struct Quat
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;

    Quat() = default;
    Quat(double x_, double y_, double z_, double w_) : x(x_), y(y_), z(z_), w(w_) {}

    Quat operator*(const Quat &other) const
    {
        return Quat(
            w * other.x + x * other.w + y * other.z - z * other.y,
            w * other.y - x * other.z + y * other.w + z * other.x,
            w * other.z + x * other.y - y * other.x + z * other.w,
            w * other.w - x * other.x - y * other.y - z * other.z);
    }

    Vec3 rotate_vec(const Vec3 &v) const
    {
        Quat qv(v.x, v.y, v.z, 0);
        Quat conj(-x, -y, -z, w);
        Quat result = (*this) * qv * conj;
        return Vec3(result.x, result.y, result.z);
    }

    nos::trent to_trent() const
    {
        nos::trent t;
        t.init(nos::trent::type::list);
        t.push_back(x);
        t.push_back(y);
        t.push_back(z);
        t.push_back(w);
        return t;
    }

    static Quat from_trent(const nos::trent &t)
    {
        if (!t.is_list() || t.as_list().size() < 4)
            return Quat();
        return Quat(
            t[0].as_numer_default(0),
            t[1].as_numer_default(0),
            t[2].as_numer_default(0),
            t[3].as_numer_default(1));
    }

    static Quat from_axis_angle(const Vec3 &axis, double angle)
    {
        double half = angle / 2;
        double s = std::sin(half);
        return Quat(axis.x * s, axis.y * s, axis.z * s, std::cos(half));
    }
};

struct Pose
{
    Vec3 position;
    Quat orientation;

    Pose() = default;
    Pose(const Vec3 &pos, const Quat &ori) : position(pos), orientation(ori) {}

    Pose operator*(const Pose &other) const
    {
        return Pose(
            position + orientation.rotate_vec(other.position),
            orientation * other.orientation);
    }

    nos::trent to_trent() const
    {
        nos::trent t;
        t.init(nos::trent::type::dict);
        t["position"] = position.to_trent();
        t["orientation"] = orientation.to_trent();
        return t;
    }
};

class KinematicNode
{
public:
    std::string name;
    std::string type; // "transform", "rotator", "actuator"
    KinematicNode *parent = nullptr;
    std::vector<std::unique_ptr<KinematicNode>> children;

    Pose local_pose;
    Vec3 axis{0, 0, 1};
    double axis_offset = 0.0;  // Offset added to coord before scaling
    double axis_scale = 1.0;   // Scale multiplier: effective_coord = (coord + offset) * scale
    double coord = 0.0;

    nos::trent model; // Pass through to client

    Pose global_pose;

    KinematicNode() = default;

    void load(const nos::trent &data, KinematicNode *parent_node = nullptr)
    {
        parent = parent_node;
        name = data["name"].as_string_default("unnamed");
        type = data["type"].as_string_default("transform");

        // Local pose
        const auto &pose_data = data["pose"];
        if (!pose_data.is_nil())
        {
            local_pose.position = Vec3::from_trent(pose_data["position"]);
            local_pose.orientation = Quat::from_trent(pose_data["orientation"]);
        }

        // Axis for rotator/actuator
        const auto &axis_data = data["axis"];
        if (!axis_data.is_nil())
        {
            axis = Vec3::from_trent(axis_data);
        }

        // Axis offset and scale
        const auto &offset_data = data["axis_offset"];
        if (!offset_data.is_nil())
        {
            axis_offset = offset_data.as_numer_default(0.0);
        }

        const auto &scale_data = data["axis_scale"];
        if (!scale_data.is_nil())
        {
            axis_scale = scale_data.as_numer_default(1.0);
        }

        // Model data (pass through to client)
        model = data["model"];

        // Parse children
        const auto &children_data = data["children"];
        if (children_data.is_list())
        {
            for (const auto &child_data : children_data.as_list())
            {
                auto child = std::make_unique<KinematicNode>();
                child->load(child_data, this);
                children.push_back(std::move(child));
            }
        }
    }

    void set_coord(double value)
    {
        coord = value;
    }

    Pose get_joint_transform() const
    {
        // Apply offset and scale: effective_coord = (coord + offset) * axis_scale
        double effective_coord = (coord + axis_offset) * axis_scale;
        if (type == "rotator")
        {
            Quat q = Quat::from_axis_angle(axis, effective_coord);
            return Pose(Vec3(), q);
        }
        else if (type == "actuator")
        {
            Vec3 offset = axis * effective_coord;
            return Pose(offset, Quat());
        }
        return Pose();
    }

    void compute_global_poses(const Pose &parent_pose = Pose())
    {
        // Global = parent * local * joint
        global_pose = parent_pose * local_pose * get_joint_transform();

        for (auto &child : children)
        {
            child->compute_global_poses(global_pose);
        }
    }

    nos::trent get_scene_data() const
    {
        nos::trent result;
        result.init(nos::trent::type::dict);

        nos::trent node_data;
        node_data.init(nos::trent::type::dict);
        node_data["pose"] = global_pose.to_trent();
        node_data["model"] = model;

        result[name] = std::move(node_data);

        for (const auto &child : children)
        {
            nos::trent child_data = child->get_scene_data();
            for (const auto &p : child_data.as_dict())
            {
                result[p.first] = p.second;
            }
        }

        return result;
    }

    KinematicNode *find_by_name(const std::string &search_name)
    {
        if (name == search_name)
            return this;
        for (auto &child : children)
        {
            KinematicNode *found = child->find_by_name(search_name);
            if (found)
                return found;
        }
        return nullptr;
    }

    void get_all_joints(std::vector<KinematicNode *> &joints)
    {
        if (type == "rotator" || type == "actuator")
        {
            joints.push_back(this);
        }
        for (auto &child : children)
        {
            child->get_all_joints(joints);
        }
    }
};

class KinematicTree
{
public:
    std::unique_ptr<KinematicNode> root;
    std::map<std::string, KinematicNode *> joints;

    void load(const nos::trent &data)
    {
        root = std::make_unique<KinematicNode>();
        root->load(data);

        // Build joints map
        joints.clear();
        std::vector<KinematicNode *> joint_list;
        root->get_all_joints(joint_list);
        for (auto *j : joint_list)
        {
            joints[j->name] = j;
        }

        update();
    }

    void set_joint_coords(const std::map<std::string, double> &coords)
    {
        for (const auto &[name, value] : coords)
        {
            auto it = joints.find(name);
            if (it != joints.end())
            {
                it->second->set_coord(value);
            }
        }
    }

    void update()
    {
        if (root)
        {
            root->compute_global_poses();
        }
    }

    nos::trent get_scene_data() const
    {
        if (root)
        {
            return root->get_scene_data();
        }
        nos::trent empty;
        empty.init(nos::trent::type::dict);
        return empty;
    }

    std::vector<std::string> get_joint_names() const
    {
        std::vector<std::string> names;
        names.reserve(joints.size());
        for (const auto &[name, _] : joints)
        {
            names.push_back(name);
        }
        return names;
    }

    nos::trent get_joint_names_trent() const
    {
        nos::trent t;
        t.init(nos::trent::type::list);
        for (const auto &name : get_joint_names())
        {
            t.push_back(name);
        }
        return t;
    }
};

} // namespace webkin
