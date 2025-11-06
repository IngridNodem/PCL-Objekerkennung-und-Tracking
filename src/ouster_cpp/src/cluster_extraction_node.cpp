#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <cmath>

// ------------------------------------------------------------
// ClusterExtractionNode
//  - Führt euklidisches Clustering auf Hindernispunkten durch
//  - Ermittelt AABB (Zentrum, Dimensionen, Orientierung)
//  - Publiziert RViz-Marker + Detection3D für den Tracker
//  - NEU: Größenfilter -> zu kleine Objekte und Objekte größer als Lkw werden verworfen
// ------------------------------------------------------------
class ClusterExtractionNode : public rclcpp::Node {
public:
  ClusterExtractionNode() : Node("cluster_extraction_node") {
    // -------------------------------
    // Parameter (I/O)
    // -------------------------------
    input_topic_      = declare_parameter<std::string>("input_topic", "/obstacle_points");
    marker_topic_     = declare_parameter<std::string>("marker_topic", "/detections_markers");
    detections_topic_ = declare_parameter<std::string>("detections_topic", "/detections_raw");

    // -------------------------------
    // Clustering-Parameter
    // -------------------------------
    tol_          = declare_parameter<double>("cluster_tolerance", 0.5);     // max. Punktabstand innerhalb eines Clusters [m]
    min_size_pts_ = declare_parameter<int>("min_cluster_size", 30);          // min. Anzahl Punkte je Cluster
    max_size_pts_ = declare_parameter<int>("max_cluster_size", 8000);       // max. Anzahl Punkte je Cluster
    max_clusters_ = declare_parameter<int>("max_clusters", 200);             // maximale Anzahl Cluster pro Frame

    // -------------------------------
    // NEU: Größen-Filter (Meter)
    //   - zu kleine Objekte raus
    //   - Objekte größer als Lkw raus (Orientierungs-unabhängig, rein auf OBB/AABB-Dimensionen)
    // -------------------------------
    min_L_allowed_ = declare_parameter<double>("min_L_allowed", 0.20);  // kleinste Länge
    min_W_allowed_ = declare_parameter<double>("min_W_allowed", 0.20);  // kleinste Breite
    min_H_allowed_ = declare_parameter<double>("min_H_allowed", 0.50);  // kleinste Höhe

    // Obergrenzen ≈ Lkw
    max_L_allowed_ = declare_parameter<double>("max_L_allowed", 8.0);   // L <= 8.0
    max_W_allowed_ = declare_parameter<double>("max_W_allowed", 5.0);   // W <= 5.0
    max_H_allowed_ = declare_parameter<double>("max_H_allowed", 4.2);   // H <= 4.2

    // -------------------------------
    // ROS I/O
    // -------------------------------
    rclcpp::SensorDataQoS qos;
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos, std::bind(&ClusterExtractionNode::callback, this, std::placeholders::_1));

    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    pub_dets_    = create_publisher<vision_msgs::msg::Detection3DArray>(detections_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(),
      "ClusterExtraction aktiv. input: %s, markers: %s, detections: %s",
      input_topic_.c_str(), marker_topic_.c_str(), detections_topic_.c_str());
    RCLCPP_INFO(get_logger(),
      "Parameter: tol=%.2f  minPts=%d  maxPts=%d  maxClusters=%d",
      tol_, min_size_pts_, max_size_pts_, max_clusters_);
    RCLCPP_INFO(get_logger(),
      "Groessenfilter: min[L,W,H]=[%.2f, %.2f, %.2f] m | max[L,W,H]=[%.2f, %.2f, %.2f] m",
      min_L_allowed_, min_W_allowed_, min_H_allowed_, max_L_allowed_, max_W_allowed_, max_H_allowed_);
  }

private:
  // ------------------------------------------------------------
  // Hilfsfunktion: prüft, ob Dimensionen innerhalb [min,max] liegen
  // ------------------------------------------------------------
  inline bool dims_okay(float L, float W, float H) const {
    if (!std::isfinite(L) || !std::isfinite(W) || !std::isfinite(H)) return false;
    if (L < min_L_allowed_ || W < min_W_allowed_ || H < min_H_allowed_) return false;      // zu klein
    if (L > max_L_allowed_ || W > max_W_allowed_ || H > max_H_allowed_) return false;      // größer als Lkw
    return true;
  }

  // ------------------------------------------------------------
  // Haupt-Callback: Clustering -> AABB -> Größenfilter -> Publizieren
  // ------------------------------------------------------------
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    using PointT = pcl::PointXYZ;

    // Eingabewolke konvertieren
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud);

    // RViz: vorherige Marker löschen
    visualization_msgs::msg::MarkerArray viz_arr;
    {
      visualization_msgs::msg::Marker m;
      m.header = msg->header;
      m.action = visualization_msgs::msg::Marker::DELETEALL;
      viz_arr.markers.push_back(m);
    }

    // Ausgabecontainer für den Tracker
    vision_msgs::msg::Detection3DArray det_arr;
    det_arr.header = msg->header;

    if (cloud->empty()) {
      pub_markers_->publish(viz_arr);
      pub_dets_->publish(det_arr);
      return;
    }

    // ----------------------------------------
    // Euclidisches Clustering (PCL)
    // ----------------------------------------
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(tol_);
    ec.setMinClusterSize(min_size_pts_);
    ec.setMaxClusterSize(max_size_pts_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int id = 0;
    for (const auto& indices : cluster_indices) {
      if (id >= max_clusters_) break;

      // Clusterpunkte extrahieren
      pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
      cluster->reserve(indices.indices.size());
      for (int idx : indices.indices) cluster->push_back((*cloud)[idx]);
      if (cluster->empty()) continue;

      // ----------------------------
      // AABB berechnen
      // ----------------------------
      Eigen::Vector3f center;
      Eigen::Vector3f dims;
      Eigen::Quaternionf q(1,0,0,0);

      // AABB (achsenparallel)
      pcl::PointXYZ min_pt, max_pt;
      pcl::getMinMax3D(*cluster, min_pt, max_pt);
      center = 0.5f * (min_pt.getVector3fMap() + max_pt.getVector3fMap());
      dims   = (max_pt.getVector3fMap() - min_pt.getVector3fMap());
      q      = Eigen::Quaternionf::Identity();
      
      // Numerische Stabilität: minimale Ausdehnung erzwingen (>= 5 cm)
      dims = dims.cwiseMax(Eigen::Vector3f(0.05f, 0.05f, 0.05f));

      const float L = dims.x(), W = dims.y(), H = dims.z();

      // ----------------------------
      // NEU: Größenfilter anwenden
      //   - zu kleine Cluster -> verwerfen
      //   - größer als Lkw -> verwerfen
      // ----------------------------
      if (!dims_okay(L, W, H)) {
        // Optionales Debug:
        // RCLCPP_DEBUG(get_logger(), "Cluster verworfen: L=%.2f W=%.2f H=%.2f", L, W, H);
        continue;
      }

      // ----------------------------
      // (A) RViz: Markerbox + Label
      // ----------------------------
      visualization_msgs::msg::Marker box;
      box.header = msg->header;
      box.ns = "clusters";
      box.id = id;
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.pose.position.x = center.x();
      box.pose.position.y = center.y();
      box.pose.position.z = center.z();
      box.pose.orientation.x = q.x();
      box.pose.orientation.y = q.y();
      box.pose.orientation.z = q.z();
      box.pose.orientation.w = q.w();
      box.scale.x = L;
      box.scale.y = W;
      box.scale.z = H;
      box.color.r = 1.0; box.color.g = 0.7; box.color.b = 0.2; box.color.a = 0.6;
      box.lifetime = rclcpp::Duration::from_seconds(0.6);

      visualization_msgs::msg::Marker label;
      label.header = msg->header;
      label.ns = "clusters_labels";
      label.id = 100000 + id;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = center.x();
      label.pose.position.y = center.y();
      label.pose.position.z = center.z() + 0.6f * H;
      label.scale.z = 0.4;
      label.color.r = 1.0; label.color.g = 1.0; label.color.b = 1.0; label.color.a = 0.9;
      char buf[128];
      std::snprintf(buf, sizeof(buf), "#%d %.2fx%.2fx%.2f m", id, L, W, H);
      label.text = std::string(buf);
      label.lifetime = rclcpp::Duration::from_seconds(0.6);

      viz_arr.markers.push_back(box);
      viz_arr.markers.push_back(label);

      // ----------------------------
      // (B) Detection3D für den Tracker
      // ----------------------------
      vision_msgs::msg::Detection3D det;
      det.header = msg->header;
      det.bbox.center.position.x = center.x();
      det.bbox.center.position.y = center.y();
      det.bbox.center.position.z = center.z();
      det.bbox.center.orientation.x = q.x();
      det.bbox.center.orientation.y = q.y();
      det.bbox.center.orientation.z = q.z();
      det.bbox.center.orientation.w = q.w();
      det.bbox.size.x = L;
      det.bbox.size.y = W;
      det.bbox.size.z = H;

      det_arr.detections.push_back(det);

      ++id;
    }

    // Ergebnisse veröffentlichen
    pub_markers_->publish(viz_arr);
    pub_dets_->publish(det_arr);
  }

  // -------------------------------
  // Membervariablen / Parameter
  // -------------------------------
  std::string input_topic_, marker_topic_, detections_topic_;
  double tol_;
  int min_size_pts_, max_size_pts_, max_clusters_;

  // Größenfilter (Meter)
  double min_L_allowed_, min_W_allowed_, min_H_allowed_;
  double max_L_allowed_, max_W_allowed_, max_H_allowed_;

  // ROS I/O
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_dets_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClusterExtractionNode>());
  rclcpp::shutdown();
  return 0;
}
