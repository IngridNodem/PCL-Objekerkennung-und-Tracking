// src/crop_box_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <array>
#include <string>
#include <vector>
#include <cstring>   // memcpy
#include <optional>

using std::placeholders::_1;

class CropBoxNode : public rclcpp::Node {
public:
  CropBoxNode() : rclcpp::Node("crop_box_node") {
    // --- Parameter (I/O und Grenzen der Zuschneide-Box)
    input_topic_  = declare_parameter<std::string>("input_topic",  "/ouster/points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/points_cropped");
    min_bound_    = get_vec3_param("min_bound", {-20.0, -15.0, -3.0});
    max_bound_    = get_vec3_param("max_bound", { 20.0,  15.0,  5.0});

    // Dynamische Re-Konfiguration (optional): aktualisiert lokale Parameterwerte
    param_cb_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params) {
        for (const auto& p : params) {
          if (p.get_name() == "min_bound" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
            min_bound_ = vec3_from_param(p.as_double_array(), min_bound_);
          }
          if (p.get_name() == "max_bound" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
            max_bound_ = vec3_from_param(p.as_double_array(), max_bound_);
          }
          // Hinweis: Änderungen an Topic-Namen würden ein erneutes Anlegen von Pub/Sub erfordern
          if (p.get_name() == "input_topic" && p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            input_topic_ = p.as_string();
          }
          if (p.get_name() == "output_topic" && p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            output_topic_ = p.as_string();
          }
        }
        rcl_interfaces::msg::SetParametersResult r; r.successful = true; return r;
      }
    );

    // --- QoS für Sensordaten
    auto qos = rclcpp::SensorDataQoS();

    // Subscriber (Eingangspunktwolke)
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos, std::bind(&CropBoxNode::callback, this, _1));

    // Publisher (ausgeschnittene Punktwolke)
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);

    RCLCPP_INFO(get_logger(),
      "CropBoxNode aktiv. input='%s' output='%s' min=[%.2f,%.2f,%.2f] max=[%.2f,%.2f,%.2f]",
      input_topic_.c_str(), output_topic_.c_str(),
      min_bound_[0], min_bound_[1], min_bound_[2],
      max_bound_[0], max_bound_[1], max_bound_[2]
    );
  }

private:
  // --- Helfer: liest einen Vektor-Parameter (3 Elemente) mit Fallback
  static std::array<float,3> get_vec3_param(const std::string& name, const std::array<double,3>& def) {
    // Robuste Übernahme der Standardwerte (der Klarheit halber ohne direkte Parameterabfrage)
    std::array<float,3> out;
    out[0]=def[0]; out[1]=def[1]; out[2]=def[2];
    return out;
  }

  // --- Helfer: wandelt vector<double> in std::array<float,3> um (bei falscher Größe Fallback)
  static std::array<float,3> vec3_from_param(const std::vector<double>& v, const std::array<float,3>& fallback) {
    if (v.size() != 3) return fallback;
    return {static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])};
  }

  // --- Helfer: findet Datentyp eines Feldes nach Name (falls vorhanden)
  static std::optional<uint8_t> find_field_datatype(const sensor_msgs::msg::PointCloud2& msg,
                                                    const std::string& name) {
    for (const auto& f : msg.fields) {
      if (f.name == name) return f.datatype;
    }
    return std::nullopt;
  }

  // --- Callback: filtert Punkte nach AABB und publiziert Ergebnis
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (msg->width == 0 || msg->height == 0) {
      // Leere Punktwolke mit kopiertem Header ausgeben
      sensor_msgs::msg::PointCloud2 empty;
      empty.header = msg->header;
      pub_->publish(empty);
      return;
    }

    // Erkennen vorhandener Felder (für robuste Verarbeitung optionaler Felder)
    const auto dt_i    = find_field_datatype(*msg, "intensity"); // erwartet FLOAT32 oder UINT16
    const bool has_i_f = (dt_i && *dt_i == sensor_msgs::msg::PointField::FLOAT32);
    const bool has_i_u = (dt_i && *dt_i == sensor_msgs::msg::PointField::UINT16);
    const bool has_i   = has_i_f || has_i_u;

    const auto dt_ring = find_field_datatype(*msg, "ring");
    const bool has_ring= (dt_ring && *dt_ring == sensor_msgs::msg::PointField::UINT16);

    // Pflichtfelder (x/y/z) als Iteratoren
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");

    // Optionale Felder: intensity (float32/uint16)
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>>    i_float;
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint16_t>> i_uint16;
    if (has_i_f)   i_float  = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*msg, "intensity");
    if (has_i_u)   i_uint16 = std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint16_t>>(*msg, "intensity");

    // Optionales Feld: ring
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint16_t>> i_ring;
    if (has_ring) i_ring = std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint16_t>>(*msg, "ring");

    // Temporäre Sammelpuffer (ein Durchlauf über die Eingabedaten)
    std::vector<float>    out_xyz_i;   out_xyz_i.reserve(msg->width * msg->height * 4);
    std::vector<uint16_t> out_ring;    out_ring.reserve(msg->width * msg->height);

    const float xmin = min_bound_[0], ymin = min_bound_[1], zmin = min_bound_[2];
    const float xmax = max_bound_[0], ymax = max_bound_[1], zmax = max_bound_[2];

    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const float x = *ix, y = *iy, z = *iz;

      // Optionale Iteratoren synchron voranschieben (falls vorhanden)
      float intensity_f = 0.0f;
      if (i_float)   { intensity_f = **i_float;   ++(*i_float); }
      else if (i_uint16) { intensity_f = static_cast<float>(**i_uint16); ++(*i_uint16); }

      uint16_t ring_v = 0;
      if (i_ring) { ring_v = **i_ring; ++(*i_ring); }

      // AABB-Test (Punkt innerhalb Zuschneide-Box?)
      if (x >= xmin && x <= xmax &&
          y >= ymin && y <= ymax &&
          z >= zmin && z <= zmax) {
        out_xyz_i.push_back(x);
        out_xyz_i.push_back(y);
        out_xyz_i.push_back(z);
        out_xyz_i.push_back(intensity_f);
        out_ring.push_back(ring_v);
      }
    }

    const size_t n = out_ring.size();

    // --- Aufbau der Ausgabepunktwolke mit festem Layout (20 Byte pro Punkt)
    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    out.height = 1;
    out.width  = static_cast<uint32_t>(n);
    out.is_bigendian = false;
    out.is_dense = true;

    sensor_msgs::PointCloud2Modifier mod(out);
    // Felder: xyz + intensity (float32) + ring (uint16) + Padding (uint16)
    mod.setPointCloud2Fields(
      6,
      "x",          1, sensor_msgs::msg::PointField::FLOAT32,
      "y",          1, sensor_msgs::msg::PointField::FLOAT32,
      "z",          1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity",  1, sensor_msgs::msg::PointField::FLOAT32,
      "ring",       1, sensor_msgs::msg::PointField::UINT16,
      "_pad",       1, sensor_msgs::msg::PointField::UINT16
    );
    mod.resize(n); // reserviert Speicher und setzt point_step/row_step korrekt

    sensor_msgs::PointCloud2Iterator<float>    ox(out, "x");
    sensor_msgs::PointCloud2Iterator<float>    oy(out, "y");
    sensor_msgs::PointCloud2Iterator<float>    oz(out, "z");
    sensor_msgs::PointCloud2Iterator<float>    oi(out, "intensity");
    sensor_msgs::PointCloud2Iterator<uint16_t> oring(out, "ring");
    sensor_msgs::PointCloud2Iterator<uint16_t> opad(out, "_pad");

    for (size_t i = 0; i < n; ++i, ++ox, ++oy, ++oz, ++oi, ++oring, ++opad) {
      *ox    = out_xyz_i[i*4 + 0];
      *oy    = out_xyz_i[i*4 + 1];
      *oz    = out_xyz_i[i*4 + 2];
      *oi    = out_xyz_i[i*4 + 3];
      *oring = out_ring[i];
      *opad  = 0;
    }

    // Veröffentlichung der zugeschnittenen Punktwolke
    pub_->publish(out);
  }

private:
  // Parameter (intern)
  std::string input_topic_;
  std::string output_topic_;
  std::array<float,3> min_bound_;
  std::array<float,3> max_bound_;

  // ROS I/O
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;

  // Handle für Parameter-Callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CropBoxNode>());
  rclcpp::shutdown();
  return 0;
}
