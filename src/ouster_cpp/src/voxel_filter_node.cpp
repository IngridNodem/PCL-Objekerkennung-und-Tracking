// src/voxel_filter_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <unordered_map>
#include <vector>
#include <tuple>
#include <cmath>
#include <memory>
#include <string>
#include <optional>
#include <limits>

using std::placeholders::_1;

// ------------------------------------------------------------
// Voxel-Koordinate für Hash-Map (diskretisierte 3D-Indizes)
// ------------------------------------------------------------
struct VoxelCoord {
  int x, y, z;
  bool operator==(const VoxelCoord& o) const { return x == o.x && y == o.y && z == o.z; }
};

namespace std {
  // ----------------------------------------------------------
  // Hash-Funktion für VoxelCoord (einfache Bitmischung)
  // ----------------------------------------------------------
  template<> struct hash<VoxelCoord> {
    size_t operator()(const VoxelCoord& v) const noexcept {
      size_t h1 = hash<int>{}(v.x);
      size_t h2 = hash<int>{}(v.y);
      size_t h3 = hash<int>{}(v.z);
      return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
  };
}

class VoxelFilterNode : public rclcpp::Node {
public:
  VoxelFilterNode() : rclcpp::Node("voxel_filter_node") {
    // --------------------------------------------------------
    // Parameter (standardmäßig relative Topics -> /<ns>/...)
    // --------------------------------------------------------
    input_topic_  = declare_parameter<std::string>("input_topic",  "points_cropped");
    output_topic_ = declare_parameter<std::string>("output_topic", "points_voxel");
    voxel_size_   = declare_parameter<double>("voxel_size", 0.20);

    // --------------------------------------------------------
    // Dynamische Anpassung: nur voxel_size wird zur Laufzeit aktualisiert
    // --------------------------------------------------------
    param_cb_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& ps) {
        for (const auto& p : ps) {
          if (p.get_name() == "voxel_size" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            double v = std::max(1e-3, p.as_double());  // Mindestvoxelgröße
            voxel_size_ = v;
          }
        }
        rcl_interfaces::msg::SetParametersResult r; r.successful = true; return r;
      }
    );

    // QoS: Sensor-Ein eingehend best_effort, ausgehend zuverlässig
    auto sub_qos = rclcpp::SensorDataQoS();              // best_effort
    auto pub_qos = rclcpp::SensorDataQoS().reliable();   // zuverlässiger Downstream

    // ROS I/O
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, sub_qos, std::bind(&VoxelFilterNode::cb, this, _1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, pub_qos);

    RCLCPP_INFO(get_logger(), "VoxelFilterNode aktiv. in='%s' out='%s' voxel=%.3f",
                input_topic_.c_str(), output_topic_.c_str(), voxel_size_);
  }

private:
  // ----------------------------------------------------------
  // Hilfsfunktion: ermittelt Datentyp eines Feldes (falls vorhanden)
  // ----------------------------------------------------------
  static std::optional<uint8_t> field_type(const sensor_msgs::msg::PointCloud2& msg,
                                           const std::string& name) {
    for (const auto& f : msg.fields) if (f.name == name) return f.datatype;
    return std::nullopt;
  }

  // ----------------------------------------------------------
  // Callback: führt die eigentliche Voxel-Filterung aus
  // ----------------------------------------------------------
  void cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    const double voxel = std::max(1e-3, voxel_size_);

    // Leere Wolke -> leere Ausgabe mit Header
    if (msg->width == 0 || msg->height == 0) {
      sensor_msgs::msg::PointCloud2 empty; empty.header = msg->header;
      // Zeitstempel auf Sendezeit setzen (Latenzmessung)
      empty.header.stamp = this->now();
      pub_->publish(empty); return;
    }

    // Optionale Felder erkennen
    const auto t_i      = field_type(*msg, "intensity");
    const bool has_if   = (t_i && *t_i == sensor_msgs::msg::PointField::FLOAT32);
    const bool has_iu   = (t_i && *t_i == sensor_msgs::msg::PointField::UINT16);
    const bool has_i    = has_if || has_iu;

    const auto t_ring   = field_type(*msg, "ring");
    const bool has_ring = (t_ring && *t_ring == sensor_msgs::msg::PointField::UINT16);

    // Eingabe-Iteratoren (Pflichtfelder)
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");

    // Eingabe-Iteratoren (optional)
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>>    i_float;
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint16_t>> i_uint16;
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint16_t>> i_ring;
    if (has_if)   i_float  = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*msg, "intensity");
    if (has_iu)   i_uint16 = std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint16_t>>(*msg, "intensity");
    if (has_ring) i_ring   = std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint16_t>>(*msg, "ring");

    // --------------------------------------------------------
    // Voxelisierung: erster Punkt pro Voxel wird beibehalten
    // Speicherlayout: Map von VoxelCoord -> (x,y,z,intensity,ring)
    // --------------------------------------------------------
    std::unordered_map<VoxelCoord, std::tuple<float,float,float,float,uint16_t>> voxels;
    voxels.reserve(static_cast<size_t>(msg->width) * msg->height / 8 + 1);

    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const float x = *ix, y = *iy, z = *iz;

      // Unzulässige Werte überspringen (NaN/Inf)
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        if (i_float) ++(*i_float); else if (i_uint16) ++(*i_uint16);
        if (i_ring) ++(*i_ring);
        continue;
      }

      float    intensity = 0.0f;
      uint16_t ring      = 0;
      if (i_float)   { intensity = **i_float;   ++(*i_float); }
      else if (i_uint16) { intensity = static_cast<float>(**i_uint16); ++(*i_uint16); }
      if (i_ring)    { ring = **i_ring; ++(*i_ring); }

      // Diskretisierung der Koordinaten in Voxel-Indizes
      const int vx = static_cast<int>(std::floor(x / voxel));
      const int vy = static_cast<int>(std::floor(y / voxel));
      const int vz = static_cast<int>(std::floor(z / voxel));
      VoxelCoord key{vx, vy, vz};

      // Ersten Punkt je Voxel übernehmen (Downsampling)
      if (voxels.find(key) == voxels.end()) {
        voxels.emplace(key, std::make_tuple(x, y, z, intensity, ring));
      }
    }

    // --------------------------------------------------------
    // Ausgabewolke erstellen (Layout: 20 Byte/Punkt)
    // Felder: x,y,z,intensity (float32) + ring (uint16) + _pad (uint16)
    // --------------------------------------------------------
    const size_t n = voxels.size();

    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    // Zeitstempel auf Sendezeit setzen (Latenzmessung)
    out.header.stamp = this->now();
    out.height = 1;
    out.width  = static_cast<uint32_t>(n);
    out.is_bigendian = false;
    out.is_dense = false;

    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2Fields(
      6,
      "x",         1, sensor_msgs::msg::PointField::FLOAT32,
      "y",         1, sensor_msgs::msg::PointField::FLOAT32,
      "z",         1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "ring",      1, sensor_msgs::msg::PointField::UINT16,
      "_pad",      1, sensor_msgs::msg::PointField::UINT16
    );
    mod.resize(n);

    sensor_msgs::PointCloud2Iterator<float>    ox(out, "x");
    sensor_msgs::PointCloud2Iterator<float>    oy(out, "y");
    sensor_msgs::PointCloud2Iterator<float>    oz(out, "z");
    sensor_msgs::PointCloud2Iterator<float>    oi(out, "intensity");
    sensor_msgs::PointCloud2Iterator<uint16_t> oring(out, "ring");
    sensor_msgs::PointCloud2Iterator<uint16_t> opad(out, "_pad");

    // Punkte aus der Map seriell in die Ausgabewolke schreiben
    for (const auto& kv : voxels) {
      const auto& [x, y, z, intensity, ring] = kv.second;
      *ox = x; *oy = y; *oz = z; *oi = intensity; *oring = ring; *opad = 0;
      ++ox; ++oy; ++oz; ++oi; ++oring; ++opad;
    }

    // Veröffentlichung der gefilterten Punktwolke
    pub_->publish(out);
  }

  // ------------------------------
  // Member/Parameter/ROS-I/O
  // ------------------------------
  std::string input_topic_, output_topic_;
  double voxel_size_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelFilterNode>());
  rclcpp::shutdown();
  return 0;
}
