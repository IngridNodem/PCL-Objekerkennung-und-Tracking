#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Dense>

#include <ouster_cpp/hungarian.hpp>

#include <unordered_map>
#include <vector>
#include <limits>
#include <cmath>
#include <string>

// -----------------------------
// Kalman-Filter im 2D-Plan (XY)
// Zustand: [x, y, vx, vy]^T
// -----------------------------
struct Kalman2D {
  Eigen::Vector4d x;
  Eigen::Matrix4d P;
  Eigen::Matrix4d F;
  Eigen::Matrix<double,2,4> H;
  Eigen::Matrix4d Q;
  Eigen::Matrix2d R;

  Kalman2D() {
    F.setIdentity();
    H.setZero(); H(0,0)=1; H(1,1)=1;
    Q.setZero();
    R.setIdentity();
    P.setIdentity(); P *= 10.0;
    x.setZero();
  }

  void init(double x0, double y0) {
    x << x0, y0, 0.0, 0.0;
    P.setIdentity(); P *= 10.0;
  }

  void set_dt(double dt) {
    F.setIdentity();
    F(0,2) = dt; // x += vx*dt
    F(1,3) = dt; // y += vy*dt

    // Einfache Prozessrauschannahme (konstante Beschleunigung ~1 m/s^2)
    const double a = 1.0;
    const double dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt2*dt2;
    const double q = a*a;
    Q.setZero();
    Q(0,0)=dt4/4*q; Q(0,2)=dt3/2*q;
    Q(1,1)=dt4/4*q; Q(1,3)=dt3/2*q;
    Q(2,0)=dt3/2*q; Q(2,2)=dt2*q;
    Q(3,1)=dt3/2*q; Q(3,3)=dt2*q;

    // Messrauschen: Positionsmessung ~0.2 m
    R.setIdentity(); R *= 0.04;
  }

  void predict() {
    x = F * x;
    P = F * P * F.transpose() + Q;
  }

  void update(double zx, double zy) {
    Eigen::Vector2d z(zx, zy);
    Eigen::Vector2d y = z - H * x;
    Eigen::Matrix2d S = H * P * H.transpose() + R;
    Eigen::Matrix<double,4,2> K = P * H.transpose() * S.inverse();
    x = x + K * y;
    P = (Eigen::Matrix4d::Identity() - K * H) * P;
  }

  double mahalanobis2(double zx, double zy) const {
    Eigen::Vector2d z(zx, zy);
    Eigen::Vector2d y = z - H * x;
    Eigen::Matrix2d S = H * P * H.transpose() + R;
    return (y.transpose() * S.inverse() * y)(0,0);
  }
};

// -----------------------------
// Track-Struktur (eine verfolgte Spur)
// -----------------------------
struct Track {
  int id{0};
  Kalman2D kf;
  int hits{0};
  int missed{0};
  bool announced{false}; // wurde „<Klasse> erkannt“ bereits ausgegeben?
  rclcpp::Time last_stamp;
  // Für Ausgabe: letzte BBox-Maße + z-Zentrum (aus letzter Assoziation)
  double w{0.8}, l{0.8}, h{1.6}, z{0.0};
  double yaw{0.0}; // hier nicht gefiltert

  // NEU: Orientierung (Quaternion) aus der zugeordneten Detection
  double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
};

// -----------------------------
// Klassenkennung (nur noch 4 Klassen + Unbekannt)
// -----------------------------
enum class ClassId { Unbekannt, Person, Fahrradfahrer, Pkw, Lkw };

struct ClassResult {
  ClassId id;
  std::string class_str; // Ausgabestring, z. B. "Mensch", "Pkw"
  float score;           // grober Vertrauenswert [0..1]
};

// Heuristische Klassifikation anhand BBox (L,W,H), Zentrumshöhe z_center, Geschwindigkeit speed_xy.
// Gibt NUR {Mensch, Fahrradfahrer, Pkw, Lkw} oder Unbekannt zurück.
static inline ClassResult classify_bbox(double L, double W, double H, double z_center, double speed_xy)
{
  const double z_min = z_center - 0.5 * H;

  // Lkw: typische Abmessungen (nur wenn in allen Dimensionen plausibel)
  if (L > 4.5 && L <= 8.0 &&
      W >= 2.0 && W <= 5.0 &&
      H >= 2.0 && H <= 4.2) {
    return {ClassId::Lkw, "Lkw", 0.85f};
  }


  // Pkw
  if (L >= 2.0 && L <= 4.5 &&
      W >= 1.4 && W <= 3.0 &&
      H >= 0.6 && H <= 2.1 ) {
    return {ClassId::Pkw, "Pkw", 0.85f};
  }

  // Fahrradfahrer (bewegter, länglicher Footprint + bodennah)
  /* if (H >= 1.2 && H <= 2.2 &&
      L >= 1.0 && L <= 2.5 &&
      W >= 0.3 && W <= 1.2 &&
      z_min < 0.7 &&
      (speed_xy > 0.25 || L > 1.1*W)) {
    float sc = (speed_xy > 3.0) ? 0.9f : 0.75f;
    return {ClassId::Fahrradfahrer, "Fahrradfahrer", sc};
  } */

  //Fahrradfahrer nur Abessungen basiert
  if (
    L >= 0.6 && L <= 2.0 &&
    W >= 0.6 && W <= 1.5 && 
    H >= 0.4 && H <= 1.6) 
  {
    return {ClassId::Fahrradfahrer, "Fahrradfahrer", 0.70f};
  }

  // Person (Mensch)
  if (L >= 0.3 && L <= 0.6 &&
      W >= 0.4 && W <= 1.0 &&
      H >= 1.0 && H <= 2.0)
  {
    return {ClassId::Person, "Mensch", 0.8f};
  }

  return {ClassId::Unbekannt, "Unbekannt", 0.2f};
}

// Nur diese 4 Klassen sind erlaubt
static inline bool is_whitelisted(ClassId id) {
  return (id == ClassId::Person ||
          id == ClassId::Fahrradfahrer ||
          id == ClassId::Pkw ||
          id == ClassId::Lkw);
}

// Maximal zulässige BBox (alles darüber wird verworfen und nicht getrackt)
static inline bool within_max_size(double L, double W, double H,
                                   double Lmax, double Wmax, double Hmax) {
  return (L <= Lmax && W <= Wmax && H <= Hmax);
}

// -----------------------------
// SORT-ähnlicher Tracker-Node
// -----------------------------
class SortTrackerNode : public rclcpp::Node {
public:
  SortTrackerNode() : Node("sort_tracker_node") {
    // Parameter
    det_topic_     = declare_parameter<std::string>("detections_topic", "/detections_raw");
    tracks_topic_  = declare_parameter<std::string>("tracks_topic", "/tracks_raw");
    markers_topic_ = declare_parameter<std::string>("tracks_markers", "/tracks_markers");

    gate_dist_max_ = declare_parameter<double>("gate_dist_max", 2.0);   // [m] maximale Assoziationsdistanz
    max_missed_    = declare_parameter<int>("max_missed", 5);           // max. verpasste Frames vor Löschung
    min_hits_      = declare_parameter<int>("min_hits", 3);             // min. Updates bis Track "bestätigt"

    // Größtes erlaubtes Objekt ≈ Lkw (alles drüber wird sofort verworfen)
    max_L_allowed_ = declare_parameter<double>("max_L_allowed", 12.0);
    max_W_allowed_ = declare_parameter<double>("max_W_allowed", 5.0);
    max_H_allowed_ = declare_parameter<double>("max_H_allowed", 5.0);

    sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
      det_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SortTrackerNode::cbDetections, this, std::placeholders::_1));

    pub_tracks_  = create_publisher<vision_msgs::msg::Detection3DArray>(tracks_topic_, 10);
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(markers_topic_, 10);
  }

private:
  void cbDetections(const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
    const auto& dets_raw = msg->detections;
    const rclcpp::Time stamp = msg->header.stamp;

    // 0) Vor-Filter: zu große Objekte (größer als Lkw-Limits) komplett verwerfen
    std::vector<int> det_idx; det_idx.reserve(dets_raw.size());
    for (int j = 0; j < (int)dets_raw.size(); ++j) {
      const auto& b = dets_raw[j].bbox;
      if (within_max_size(b.size.x, b.size.y, b.size.z,
                          max_L_allowed_, max_W_allowed_, max_H_allowed_)) {
        det_idx.push_back(j);
      }
    }

    // 1) Prädiktion aller vorhandenen Tracks
    for (auto& kv : tracks_) {
      auto& t = kv.second;
      const double dt = (t.last_stamp.nanoseconds() == 0) ? 0.1
                        : std::max(0.01, (stamp - t.last_stamp).seconds());
      t.kf.set_dt(dt);
      t.kf.predict();
      t.last_stamp = stamp;
    }

    // 2) Kostenmatrix (nur für vorgefilterte Detections)
    const int T = (int)tracks_.size();
    const int D = (int)det_idx.size();
    std::vector<int> track_ids; track_ids.reserve(T);
    for (auto& kv : tracks_) track_ids.push_back(kv.first);

    std::vector<std::vector<double>> cost(T, std::vector<double>(D, 1e6));
    for (int i=0;i<T;i++) {
      const auto& t = tracks_.at(track_ids[i]);
      for (int jj=0;jj<D;jj++) {
        const auto& b = dets_raw[det_idx[jj]].bbox;
        const double dx = b.center.position.x - t.kf.x(0);
        const double dy = b.center.position.y - t.kf.x(1);
        const double dist = std::hypot(dx, dy);
        if (dist <= gate_dist_max_) cost[i][jj] = dist;
      }
    }

    // 3) Zuordnung via Hungarian
    std::vector<int> assign = (T>0 && D>0) ? Hungarian::solve(cost) : std::vector<int>(T, -1);
    std::vector<char> det_used(D, false);

    // 4) Update zugeordneter Tracks
    for (int i=0;i<T;i++) {
      const int trk_id = track_ids[i];
      const int jj = assign[i];
      if (jj>=0 && jj<D && cost[i][jj] < 1e5) {
        const int j = det_idx[jj];
        auto& t = tracks_.at(trk_id);
        const auto& b = dets_raw[j].bbox;

        // Mess-Update (nur Position XY wird gefiltert)
        t.kf.update(b.center.position.x, b.center.position.y);

        // Zähler/Zeit
        t.hits++; t.missed = 0;
        t.last_stamp = stamp;

        // Geometrie + Höhe übernehmen
        t.l = b.size.x; t.w = b.size.y; t.h = b.size.z; t.z = b.center.position.z;

        // NEU: Orientierung (Quaternion) aus Detection übernehmen
        t.qx = b.center.orientation.x;
        t.qy = b.center.orientation.y;
        t.qz = b.center.orientation.z;
        t.qw = b.center.orientation.w;

        // Robustheit: falls Größe plötzlich zu groß -> löschen
        if (!within_max_size(t.l, t.w, t.h, max_L_allowed_, max_W_allowed_, max_H_allowed_)) {
          to_remove_.push_back(trk_id);
        }
        det_used[jj] = true;
      } else {
        tracks_.at(trk_id).missed++;
      }
    }

    // 5) Neue Tracks nur für Whitelist-Klassen anlegen
    for (int jj=0;jj<D;jj++) if (!det_used[jj]) {
      const int j = det_idx[jj];
      const auto& b = dets_raw[j].bbox;
      const double L=b.size.x, W=b.size.y, H=b.size.z, Zc=b.center.position.z;
      auto cls0 = classify_bbox(L, W, H, Zc, 0.0);

      if (!is_whitelisted(cls0.id)) continue; // Nicht erlaubt -> kein Track

      Track t;
      t.id = next_id_++;
      t.kf.init(b.center.position.x, b.center.position.y);
      t.kf.set_dt(0.1);
      t.hits = 1;
      t.missed = 0;
      t.announced = false;
      t.last_stamp = stamp;

      // Geometrie/Höhe
      t.l = L; t.w = W; t.h = H; t.z = Zc;

      // NEU: Orientierung vom Cluster übernehmen
      t.qx = b.center.orientation.x;
      t.qy = b.center.orientation.y;
      t.qz = b.center.orientation.z;
      t.qw = b.center.orientation.w;

      tracks_.emplace(t.id, std::move(t));
    }

    // 6) Tracks löschen (zu viele verpasste Updates oder markiert)
    std::vector<int> to_remove_local;
    for (auto& kv : tracks_) if (kv.second.missed > max_missed_) to_remove_local.push_back(kv.first);
    for (int id : to_remove_) to_remove_local.push_back(id);
    to_remove_.clear();
    for (int id : to_remove_local) tracks_.erase(id);

    // 7) Publizieren
    publishTracks(msg->header);
  }

  void publishTracks(const std_msgs::msg::Header& header) {
    vision_msgs::msg::Detection3DArray out;
    out.header = header;
    // Zeitstempel: aktuelle Zeit für Latenzmessung
    out.header.stamp = this->now();

    visualization_msgs::msg::MarkerArray markers;
    {
      visualization_msgs::msg::Marker del;
      del.header = header;
      del.header.stamp = this->now();
      del.action = visualization_msgs::msg::Marker::DELETEALL;
      markers.markers.push_back(del);
    }

    // Sammelliste für sofortiges Löschen nicht-Whitelist
    std::vector<int> erase_ids;

    for (auto& kv : tracks_) {
      auto& t = kv.second;

      // Nur bestätigte Tracks publizieren (min_hits); frisch geborene dürfen laufen, wenn gerade geupdatet
      if (t.hits < min_hits_ && t.missed > 0) continue;

      const double x=t.kf.x(0), y=t.kf.x(1);
      const double speed_xy = std::hypot(t.kf.x(2), t.kf.x(3));
      auto cls = classify_bbox(t.l, t.w, t.h, t.z, speed_xy);

      // Harte Whitelist: nur Mensch, Fahrradfahrer, Pkw, Lkw
      if (!is_whitelisted(cls.id)) {
        erase_ids.push_back(t.id);
        continue;
      }

      // Einmalige Konsolen-Ausgabe bei erstmaliger Bestätigung
      if (!t.announced && t.hits >= min_hits_ && t.missed == 0) {
        RCLCPP_INFO(this->get_logger(), "%s erkannt (Track %d)", cls.class_str.c_str(), t.id);
        t.announced = true;
      }

      // Detection3D befüllen (inkl. Orientierung aus Track)
      vision_msgs::msg::Detection3D det;
      // Gleicher Frame, aktuelle Zeit wie im Array-Header
      det.header = out.header;
      det.bbox.center.position.x = x;
      det.bbox.center.position.y = y;
      det.bbox.center.position.z = t.z;
      det.bbox.size.x = t.l;
      det.bbox.size.y = t.w;
      det.bbox.size.z = t.h;
      det.bbox.center.orientation.x = t.qx;
      det.bbox.center.orientation.y = t.qy;
      det.bbox.center.orientation.z = t.qz;
      det.bbox.center.orientation.w = t.qw;

      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = cls.class_str; // "Mensch" | "Fahrradfahrer" | "Pkw" | "Lkw"
      hyp.hypothesis.score    = cls.score;
      det.results.push_back(hyp);

      out.detections.push_back(det);

      // RViz Marker (mit derselben Orientierung)
      visualization_msgs::msg::Marker box;
      box.header = header;
      box.header.stamp = this->now();
      box.ns = "tracks";
      box.id = t.id;
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.pose.position.x = det.bbox.center.position.x;
      box.pose.position.y = det.bbox.center.position.y;
      box.pose.position.z = det.bbox.center.position.z;
      box.pose.orientation.x = t.qx;
      box.pose.orientation.y = t.qy;
      box.pose.orientation.z = t.qz;
      box.pose.orientation.w = t.qw;
      box.scale.x = det.bbox.size.x;
      box.scale.y = det.bbox.size.y;
      box.scale.z = det.bbox.size.z;
      box.color.r = 0.2; box.color.g = 0.8; box.color.b = 1.0; box.color.a = 0.7;
      box.lifetime = rclcpp::Duration::from_seconds(0.4);

      visualization_msgs::msg::Marker label;
      label.header = header;
      label.header.stamp = this->now();
      label.ns = "tracks_labels";
      label.id = 100000 + t.id;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position = box.pose.position;
      label.pose.position.z += 0.6 * box.scale.z;
      label.scale.z = 0.5;
      label.color.r = 1.0; label.color.g = 1.0; label.color.b = 1.0; label.color.a = 1.0;
      label.text = "T" + std::to_string(t.id) + " " + hyp.hypothesis.class_id;
      label.lifetime = rclcpp::Duration::from_seconds(0.4);

      markers.markers.push_back(box);
      markers.markers.push_back(label);
    }

    // Nicht-whitelistete sofort entfernen
    for (int id : erase_ids) tracks_.erase(id);

    pub_tracks_->publish(out);
    pub_markers_->publish(markers);
  } 

  // -----------------------------
  // Parameter / ROS-IO
  // -----------------------------
  std::string det_topic_, tracks_topic_, markers_topic_;
  double gate_dist_max_;
  int max_missed_, min_hits_;

  double max_L_allowed_, max_W_allowed_, max_H_allowed_;

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_tracks_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  std::unordered_map<int, Track> tracks_;
  std::vector<int> to_remove_;
  int next_id_{1};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SortTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
