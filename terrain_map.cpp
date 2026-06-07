/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 *******************************************************************************/

#include <mutex>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

#include <fins/node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace finenav {

using Position3D = Eigen::Vector3d;

class SimpleMapNode : public fins::Node {
public:
    SimpleMapNode() = default;
    virtual ~SimpleMapNode() = default;

    void define() override {
		set_name("SimpleMapNode");
		set_description("Self-contained lightweight 2D rolling grid map node");
		set_category("FineNav>Map");

        register_input<sensor_msgs::msg::PointCloud2>("point_cloud", &SimpleMapNode::onPointCloud);
        register_input<geometry_msgs::msg::PoseStamped>("robot_pose", &SimpleMapNode::onRobotPose);

        register_server<bool(void)>("is_tracking_unknown", &SimpleMapNode::handleIsTrackingUnknown);
        register_server<bool(void)>("consider_footprint", &SimpleMapNode::handleConsiderFootprint);
        register_server<bool(float, float, float)>("is_collision", &SimpleMapNode::handleIsCollision);
        register_server<float(void)>("get_radius", &SimpleMapNode::handleGetRadius);
        register_server<int(const Position3D&)>("get_cost", &SimpleMapNode::handleGetCost);
        register_server<bool(float, float, float)>("cost_at_pose", &SimpleMapNode::handleCostAtPose);
        register_server<std::string(void)>("get_base_frame_id", &SimpleMapNode::handleGetBaseFrameID);
    }

    void initialize() override {
        logger->info("Initializing SimpleMapNode...");

        map_size_ = 200;
        resolution_ = 0.1f;
        grid_.resize(map_size_ * map_size_, 0);

        origin_x_ = 0.0f;
        origin_y_ = 0.0f;

        logger->info("SimpleMapNode initialized.");
    }

    void run() override {
        logger->info("SimpleMapNode is active.");
    }

    void pause() override {}

    void reset() override {
        std::lock_guard<std::mutex> lock(map_mutex_);
        std::fill(grid_.begin(), grid_.end(), 0);
        logger->info("SimpleMapNode reset.");
    }

private:
    void onRobotPose(const geometry_msgs::msg::PoseStamped& msg) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        latest_pose_ = msg;

        origin_x_ = static_cast<float>(latest_pose_.pose.position.x);
        origin_y_ = static_cast<float>(latest_pose_.pose.position.y);
    }

    void onPointCloud(const sensor_msgs::msg::PointCloud2& msg) {
        std::lock_guard<std::mutex> lock(map_mutex_);

        std::fill(grid_.begin(), grid_.end(), 0);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

        double yaw = getYaw(latest_pose_.pose.orientation);
        float cos_y = static_cast<float>(std::cos(yaw));
        float sin_y = static_cast<float>(std::sin(yaw));

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float lx = *iter_x;
            float ly = *iter_y;
            float lz = *iter_z;

            if (lz < 0.1f || lz > 1.5f) {
                continue;
            }

            float gx = lx * cos_y - ly * sin_y + origin_x_;
            float gy = lx * sin_y + ly * cos_y + origin_y_;

            if (isInside(gx, gy)) {
                int ix = getIndexX(gx);
                int iy = getIndexY(gy);

                grid_[iy * map_size_ + ix] = 254;

                for (int dx = -2; dx <= 2; ++dx) {
                    for (int dy = -2; dy <= 2; ++dy) {
                        int nx = ix + dx;
                        int ny = iy + dy;
                        if (nx >= 0 && nx < map_size_ && ny >= 0 && ny < map_size_) {
                            int dist_sq = dx * dx + dy * dy;
                            uint8_t cost = 0;
                            if (dist_sq == 0) cost = 254;
                            else if (dist_sq <= 1) cost = 180;
                            else if (dist_sq <= 4) cost = 80;
                            
                            int cell_idx = ny * map_size_ + nx;
                            grid_[cell_idx] = std::max(grid_[cell_idx], cost);
                        }
                    }
                }
            }
        }
    }

    bool handleIsTrackingUnknown() {
        return is_tracking_unknown_;
    }

    bool handleConsiderFootprint() {
        return consider_footprint_;
    }

    float handleGetRadius() {
        return 0.85f;
    }

    std::string handleGetBaseFrameID() {
        return "map";
    }

    int handleGetCost(const Position3D& pos) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        float x = static_cast<float>(pos.x());
        float y = static_cast<float>(pos.y());

        if (!isInside(x, y)) {
            return is_tracking_unknown_ ? 0 : 255;
        }
        return grid_[getIndexY(y) * map_size_ + getIndexX(x)];
    }

    bool handleIsCollision(float x, float y, float theta) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        return checkCollisionFootprint(x, y, theta);
    }

    float handleCostAtPose(float x, float y, float theta) {
        std::lock_guard<std::mutex> lock(map_mutex_);

        int max_cost = 0;
        auto pts = getFootprintPoints(x, y, theta);
        for (const auto& pt : pts) {
            if (isInside(pt.first, pt.second)) {
                int ix = getIndexX(pt.first);
                int iy = getIndexY(pt.second);
                max_cost = std::max(max_cost, static_cast<int>(grid_[iy * map_size_ + ix]));
            }
        }
        return static_cast<float>(max_cost);
    }

    inline bool isInside(float x, float y) const {
        float dx = x - origin_x_;
        float dy = y - origin_y_;
        float max_range = (map_size_ / 2) * resolution_;
        return (std::abs(dx) < max_range && std::abs(dy) < max_range);
    }

    inline int getIndexX(float x) const {
        return std::clamp(static_cast<int>((x - origin_x_) / resolution_) + map_size_ / 2, 0, map_size_ - 1);
    }

    inline int getIndexY(float y) const {
        return std::clamp(static_cast<int>((y - origin_y_) / resolution_) + map_size_ / 2, 0, map_size_ - 1);
    }

    bool checkCollisionFootprint(float x, float y, float theta) const {
        auto pts = getFootprintPoints(x, y, theta);
        for (const auto& pt : pts) {
            if (!isInside(pt.first, pt.second)) {
                if (!is_tracking_unknown_) return true;
                continue;
            }
            int ix = getIndexX(pt.first);
            int iy = getIndexY(pt.second);
            if (grid_[iy * map_size_ + ix] >= 253) {
                return true;
            }
        }
        return false;
    }

    std::vector<std::pair<float, float>> getFootprintPoints(float x, float y, float theta) const {
        const float l_half = 0.4f;
        const float w_half = 0.35f;
        const std::array<std::pair<float, float>, 5> offsets = {{
            {0.0f, 0.0f},
            {l_half, w_half}, {l_half, -w_half},
            {-l_half, w_half}, {-l_half, -w_half}
        }};

        float cos_t = std::cos(theta);
        float sin_t = std::sin(theta);

        std::vector<std::pair<float, float>> pts;
        pts.reserve(5);
        for (const auto& opt : offsets) {
            float gx = opt.first * cos_t - opt.second * sin_t + x;
            float gy = opt.first * sin_t + opt.second * cos_t + y;
            pts.push_back({gx, gy});
        }
        return pts;
    }

    double getYaw(const geometry_msgs::msg::Quaternion& q) const {
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

private:
    std::mutex map_mutex_;

    int map_size_;
    float resolution_;
    float origin_x_;
    float origin_y_;
    std::vector<uint8_t> grid_;

    geometry_msgs::msg::PoseStamped latest_pose_;

    bool is_tracking_unknown_{false};
    bool consider_footprint_{true};
};

EXPORT_NODE(SimpleMapNode)

} // namespace finenav

DEFINE_PLUGIN_ENTRY(fins::STATEFUL)