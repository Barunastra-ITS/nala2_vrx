#include <fstream>
#include <OgreSceneNode.h>
#include <chrono>

#include "rviz_plugin/waypoint_nav_frame.hpp"
#include "rviz_plugin/waypoint_nav_tool.hpp"

#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

#include <QFileDialog>
#include <QKeyEvent>

#include <boost/foreach.hpp>

#include <nala2_interfaces/srv/string_service.hpp>
#include <nala2_interfaces/srv/set_path.hpp>

using json = nlohmann::json;
using namespace std::chrono_literals;

namespace waypoint_nav_plugin 
{

WaypointNavPanel::WaypointNavPanel(QWidget *parent
                                  , rviz_common::DisplayContext* context
                                  , std::map<int, Ogre::SceneNode* >* waypointNodeMap
                                  , std::shared_ptr<interactive_markers::InteractiveMarkerServer> server
                                  , std::shared_ptr<int> uniqueWaypointIndex
                                  , WaypointNavTool* wp_tool
                                  )
  : rviz_common::Panel(parent)
  , context_(context)
  , server_(server)
  , waypointNodeMap_(waypointNodeMap)
  , uniqueWaypointIndex_(uniqueWaypointIndex)
  , wp_nav_tool_(wp_tool)
  , frame_id_("map")
  , current_mission(0)
  , default_height_(0.0)
  , ui_(std::make_shared<Ui::WaypointNavigationWidget>())
  , selected_marker_name_(std::string(wp_name_prefix) + "1")
  , client_node_(createNewNode("waypoint_navigation_panel"))
{
  if (!context_) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Context is null!");
    return;
  }
  scene_manager_ = context_->getSceneManager();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(client_node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // set up the GUI
  ui_->setupUi(this);

  // set up all publisher & client
  initPublisher();
  initService();

  //connect the Qt signals and slots
  connect(ui_->publish_button, &QPushButton::clicked, this, &WaypointNavPanel::publishWaypoint);
  connect(ui_->insert_button, &QPushButton::clicked, this, &WaypointNavPanel::placeWaypoint);  
  connect(ui_->publish_all_button, &QPushButton::clicked, this, &WaypointNavPanel::publishAllWaypoint);
  connect(ui_->missionComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &WaypointNavPanel::setMission);

  connect(ui_->delete_all_button, &QPushButton::clicked, std::bind(&WaypointNavPanel::deleteAllWaypoints, this, true));
  connect(ui_->delete_button, &QPushButton::clicked, this, &WaypointNavPanel::deleteSelectedWaypoint);

  connect(ui_->save_wp_button, &QPushButton::clicked, this, &WaypointNavPanel::saveWaypoint);
  connect(ui_->load_wp_button, &QPushButton::clicked, this, &WaypointNavPanel::loadWaypoint);

  connect(ui_->command_button, &QPushButton::clicked, this, &WaypointNavPanel::autonomy);
  connect(ui_->path_button, &QPushButton::clicked, this, &WaypointNavPanel::recordRoute);

  connect(ui_->x_doubleSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &WaypointNavPanel::poseChanged);
  connect(ui_->y_doubleSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &WaypointNavPanel::poseChanged);
  connect(ui_->z_doubleSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &WaypointNavPanel::poseChanged);
  connect(ui_->yaw_doubleSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &WaypointNavPanel::poseChanged);

  // set default mission
  setMission(0);

  ui_->save_route_button->setEnabled(false);

  spin_thread_ = std::make_shared<std::thread>([this]() {
    rclcpp::spin(client_node_);
  });

  // timer_id_ = startTimer(100);

  RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Waypoint panel construction is complete");
}

WaypointNavPanel::~WaypointNavPanel() {
  waypointNodeMap_ = NULL;
}

void WaypointNavPanel::enable() {
  show();
}

void WaypointNavPanel::disable() {
  hide();
}

void WaypointNavPanel::initPublisher() {
  wp_pub_[0] = client_node_->create_publisher<nav_msgs::msg::Path>("/waypoints", 10);
  for(int i=0; i<21; i++){
    std::string current_topic =  "/waypoints";
    current_topic += "/mission_" + std::to_string(i/2+1);

    i%2 == 0? current_topic += "/pre" : current_topic += "/in";

    wp_pub_[i+1] = client_node_->create_publisher<nav_msgs::msg::Path>(current_topic, 10);
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Created topic %s", current_topic.c_str());
  }

  pose_pub_ = client_node_->create_publisher<nav_msgs::msg::Path>("/rviz/nala_path", 10);
}

void WaypointNavPanel::initService() {
  path_client_[0] = client_node_->create_client<nala2_interfaces::srv::SetPath>("/waypoints");
  for(int i=0; i<27; i++){
    std::string current_topic =  "/waypoints";
    current_topic += "/mission_" + std::to_string(i/2+1);

    i%2 == 0? current_topic += "/pre" : current_topic += "/in";

    path_client_[i+1] = client_node_->create_client<nala2_interfaces::srv::SetPath>(current_topic);
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Created topic %s", current_topic.c_str());
  }

  autonomy_client_ = client_node_->create_client<nala2_interfaces::srv::StringService>("/decisioning/mission_manager/command");
  route_client_ = client_node_->create_client<nala2_interfaces::srv::StringService>("/decisioning/path_record/cmd");
}

void WaypointNavPanel::setPath() {
  paths[current_mission].poses.clear();
  std::map<int, Ogre::SceneNode* >::iterator sn_it;
  for (sn_it = waypointNodeMap_->begin(); sn_it != waypointNodeMap_->end(); sn_it++) {
    geometry_msgs::msg::PoseStamped pos;
    Ogre::Vector3 position;
    position = sn_it->second->getPosition();
    // This is quite dangerous, so future nala please fix this for me -mundi :)
    if (position.x == 0.0 && position.y == 0.0) {
      RCLCPP_WARN(rclcpp::get_logger("waypoint_panel"), "Waypoint with position (0, 0) ignored");
      continue;
    }
    pos.pose.position.x = position.x;
    pos.pose.position.y = position.y;
    pos.pose.position.z = position.z;

    Ogre::Quaternion quat;
    quat = sn_it->second->getOrientation();
    pos.pose.orientation.x = quat.x;
    pos.pose.orientation.y = quat.y;
    pos.pose.orientation.z = quat.z;
    pos.pose.orientation.w = quat.w;

    pos.header.frame_id = "map";
    pos.header.stamp = client_node_->now();

    paths[current_mission].poses.push_back(pos);
  }
}

void WaypointNavPanel::drawPath(int current_mission, bool is_base_mission){
  mission_str = std::to_string((current_mission+1)/2);
  mission_str += (current_mission%2? " pre" : " in");
  // std::cout << "current mission: " << current_mission << " | str: " << mission_str << std::endl;
  wp_nav_tool_->marker_id_ = 0;
  for(int i = 0; i < paths[current_mission].poses.size();i++) {
    geometry_msgs::msg::PoseStamped pos = paths[current_mission].poses[i];
    Ogre::Vector3 position;
    position.x = pos.pose.position.x;
    position.y = pos.pose.position.y;
    position.z = pos.pose.position.z;

    Ogre::Quaternion quat = Ogre::Quaternion(Ogre::Radian(M_PI / 2.), Ogre::Vector3::UNIT_Z);
    quat.x = pos.pose.orientation.x;
    quat.y = pos.pose.orientation.y;
    quat.z = pos.pose.orientation.z;
    quat.w = pos.pose.orientation.w;

    wp_nav_tool_->makeIm(position, quat, current_mission, is_base_mission);
  }

  if (!is_base_mission) setPath();
  RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Finished drawing path for mission %s", mission_str.c_str());
}

void WaypointNavPanel::setMission(int index) {
  current_mission = index;
  wp_nav_tool_->mission_index = index;
  deleteAllWaypoints();

  if(current_mission == 0){
    // for(int i =1; i<27;i++){
    //   drawPath(i, true);
    // }
    ui_->current_topic_label->setText("/waypoints");
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Currently in base mission");
  } else {  
    drawPath(current_mission);
    current_topic =  "/waypoints/mission_";
    current_topic += std::to_string((current_mission+1)/2);
    current_mission%2 == 0? current_topic += "/in" : current_topic += "/pre";
    ui_->current_topic_label->setText(current_topic.c_str());

    mission_str = std::to_string((current_mission+1)/2);
    current_mission%2 == 0? mission_str += " in" : mission_str += " pre";
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Currently in mission %s", mission_str.c_str());
  }
  ui_->insert_wp_spinBox->setMinimum(1);
  ui_->insert_wp_spinBox->setMaximum(paths[current_mission].poses.size());
}

void WaypointNavPanel::deleteAllWaypoints(bool button_clicked) {
  if (button_clicked) { 
    paths[current_mission].poses.clear();
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Delete all waypoints clicked");
  }

  std::map<int, Ogre::SceneNode* >::iterator sn_it;
  for (sn_it = waypointNodeMap_->begin(); sn_it != waypointNodeMap_->end(); sn_it++) {
    scene_manager_->destroySceneNode(sn_it->second);
  }
  
  waypointNodeMap_->clear();
  *uniqueWaypointIndex_ = 0;
  server_->clear();
  server_->applyChanges();
  RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Clear waypoint success");
}

void WaypointNavPanel::deleteSelectedWaypoint() {
  std::cout << "delete wp entry" << std::endl;
  int from_wp = ui_->delete_wp_from->value() - 1;
  int to_wp = ui_->delete_wp_to->value();

  std::cout << "delete wp entry 2" << std::endl;
  if (from_wp < 0 || to_wp > paths[current_mission].poses.size() || from_wp > to_wp) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Invalid range to delete waypoints");
    return;
  }
  paths[current_mission].poses.erase(paths[current_mission].poses.begin()+ from_wp, paths[current_mission].poses.begin() +to_wp);
  deleteAllWaypoints();
  drawPath(current_mission);
  RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Success delete waypoint from %d to %d", from_wp, to_wp);
}

void WaypointNavPanel::placeWaypoint() {
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
      transformStamped = tf_buffer_->lookupTransform("map", "asv/base_link", tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex) {
      std::cout << "TF Exception: " << ex.what() << std::endl;
      return;
  }

  geometry_msgs::msg::PoseStamped temp;
  temp.pose.position.x = transformStamped.transform.translation.x;
  temp.pose.position.y = transformStamped.transform.translation.y;
  temp.pose.position.z = default_height_;

  // Ignore if position x and y are both 0
  if (temp.pose.position.x == 0.0 && temp.pose.position.y == 0.0) {
      RCLCPP_WARN(rclcpp::get_logger("waypoint_panel"), "Waypoint with position (0, 0) ignored");
      return;
  }

  temp.pose.orientation.w = transformStamped.transform.rotation.w;
  temp.pose.orientation.x = transformStamped.transform.rotation.x;
  temp.pose.orientation.y = transformStamped.transform.rotation.y;
  temp.pose.orientation.z = transformStamped.transform.rotation.z;

  paths[current_mission].poses.push_back(temp);
  mission_str = std::to_string((current_mission+1)/2);
  mission_str += (current_mission%2? " pre" : " in");
  RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Success to insert waypoint in mission %s", mission_str.c_str());

  deleteAllWaypoints();
  drawPath(current_mission);
}

void WaypointNavPanel::insertWaypoint(const Ogre::Vector3& pos, const Ogre::Quaternion& quat) {
  mission_str = std::to_string((current_mission + 1) / 2);
  mission_str += (current_mission % 2 ? " pre" : " in");

  int index_to_insert;
  if (current_mission == 0) {
    index_to_insert = 0;
    deleteAllWaypoints();
    for (int i = 1; i < 27; i++) {
      drawPath(i);
    }
    RCLCPP_WARN(rclcpp::get_logger("waypoint_panel"), "Cannot insert waypoint in this mission");
  } else {
    index_to_insert = paths[current_mission].poses.size();
    geometry_msgs::msg::PoseStamped tmp_wp;
    
    tmp_wp.pose.position.x = pos.x;
    tmp_wp.pose.position.y = pos.y;
    tmp_wp.pose.position.z = pos.z;

    tmp_wp.pose.orientation.w = quat.w;
    tmp_wp.pose.orientation.x = quat.x;
    tmp_wp.pose.orientation.y = quat.y;
    tmp_wp.pose.orientation.z = quat.z;

    // Check the distance with the last waypoint
    // if (index_to_insert > 0 && ui_->insert_mode_checkBox->isChecked()) {
    //   const auto& last_wp = paths[current_mission].poses.back().pose.position;
    //   double dx = pos.x - last_wp.x;
    //   double dy = pos.y - last_wp.y;
    //   double dz = pos.z - last_wp.z;
    //   double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    //   if (distance > 5.0) {
    //     int num_intermediate_points = static_cast<int>(std::floor(distance / 5.0));
    //     for (int i = 1; i <= num_intermediate_points; ++i) {
    //       double ratio = static_cast<double>(i) / (num_intermediate_points + 1);

    //       geometry_msgs::msg::PoseStamped intermediate_wp;
    //       intermediate_wp.pose.position.x = last_wp.x + dx * ratio;
    //       intermediate_wp.pose.position.y = last_wp.y + dy * ratio;
    //       intermediate_wp.pose.position.z = last_wp.z + dz * ratio;

    //       intermediate_wp.pose.orientation = tmp_wp.pose.orientation; // Same orientation

    //       paths[current_mission].poses.push_back(intermediate_wp);
    //     }
    //   }
    // }

    // Insert the new waypoint
    paths[current_mission].poses.push_back(tmp_wp);
    deleteAllWaypoints();
    drawPath(current_mission);
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Success create waypoint %d in mission %s", index_to_insert + 1, mission_str.c_str());
  }

  // Update UI components
  ui_->insert_wp_spinBox->setValue(paths[current_mission].poses.size());
  ui_->insert_wp_spinBox->setMinimum(1);
  ui_->insert_wp_spinBox->setMaximum(paths[current_mission].poses.size());
}

void WaypointNavPanel::publishAllWaypoint() {
  // using topic
  // for(int i = 0;i < 27; i++) {
  //   wp_pub_[i+1]->publish(paths[i+1]);
  // }

  // using service
  for(int i = 0;i < 27; i++) {
    mission_str = std::to_string((i/2)+1);
    mission_str += (((i/2)+1)%2? " pre" : " in");
    if(paths[i].poses.size() > 0) {
      if (path_client_[i]->wait_for_service(0.25s)) {
        auto future_result = std::shared_future<std::shared_ptr<nala2_interfaces::srv::SetPath::Response>>(); 

        auto request = std::make_shared<nala2_interfaces::srv::SetPath::Request>();
        request->path = paths[i];

        future_result = path_client_[i]->async_send_request(request);
        auto response = future_result.get();
        RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Mission %s have been called", mission_str.c_str());
      }
      else{ 
        RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "service for mission %s not available", mission_str.c_str());
      }
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "All mission have been called");
}

void WaypointNavPanel::publishWaypoint() {
  // using topic
  // wp_pub_[current_mission]->publish(paths[current_mission]);

  // using service
  if (!path_client_[current_mission]->wait_for_service(0.25s)) {
    mission_str = std::to_string((current_mission/2)+1);
    mission_str = (((current_mission/2)+1)%2? " pre" : " in");
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "service for mission %s not available", mission_str.c_str());
    return;
  }
  auto future_result = std::shared_future<std::shared_ptr<nala2_interfaces::srv::SetPath::Response>>(); 

  auto request = std::make_shared<nala2_interfaces::srv::SetPath::Request>();
  request->path = paths[current_mission];

  future_result = path_client_[current_mission]->async_send_request(request);
  auto response = future_result.get();
  RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Response: %s, Success: %s",
              response->message.c_str(), response->success ? "true" : "false");

  mission_str = std::to_string((current_mission/2)+1);
  ((current_mission/2)+1)%2? mission_str += " pre" : mission_str += " in";
  RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Mission %s have been called", mission_str.c_str());
}

void WaypointNavPanel::setSelectedMarkerName(std::string name)
{
  std::stringstream wp_label_name;
  mission_str = std::to_string((current_mission+1)/2);
  mission_str += (current_mission%2? " pre" : " in");
  
  wp_label_name << name << " , mission "<< current_mission;

  selected_marker_name_ = wp_label_name.str();
}

void WaypointNavPanel::poseChanged(double val) {
  auto sn_entry = waypointNodeMap_->end();
  try {
    const int selected_marker_idx=0;
    std::stoi(selected_marker_name_.substr(strlen(wp_name_prefix), 2));
    sn_entry = waypointNodeMap_->find(selected_marker_idx);
  } catch (const std::logic_error &e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), e.what());
    return;
  }

  if (sn_entry == waypointNodeMap_->end())
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "%s not found in map", selected_marker_name_.c_str());
  else {
    Ogre::Vector3 position;
    Ogre::Quaternion quat;
    getPose(position, quat);

    sn_entry->second->setPosition(position);
    sn_entry->second->setOrientation(quat);

    std::stringstream wp_name;
    wp_name << wp_name_prefix << sn_entry->first;
    std::string wp_name_str(wp_name.str());

    if(server_->get(wp_name_str, int_marker))
    {
      int_marker.pose.position.x = position.x;
      int_marker.pose.position.y = position.y;
      int_marker.pose.position.z = position.z;

      int_marker.pose.orientation.x = quat.x;
      int_marker.pose.orientation.y = quat.y;
      int_marker.pose.orientation.z = quat.z;
      int_marker.pose.orientation.w = quat.w;

      server_->setPose(wp_name_str, int_marker.pose, int_marker.header);
    }
    server_->applyChanges();
  }
}

void WaypointNavPanel::getPose(Ogre::Vector3& position, Ogre::Quaternion& quat)
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  position.x = ui_->x_doubleSpinBox->value();
  position.y = ui_->y_doubleSpinBox->value();
  position.z = ui_->z_doubleSpinBox->value();
  double yaw = ui_->yaw_doubleSpinBox->value();

  tf2::Quaternion qt;
  qt.setRPY(0, 0, yaw);
  quat.x = qt.x();
  quat.y = qt.y();
  quat.z = qt.z();
  quat.w = qt.w();
}

void WaypointNavPanel::setPose(const Ogre::Vector3& position, const Ogre::Quaternion& quat, const int wp_index)
{
  ui_->x_doubleSpinBox->blockSignals(true);
  ui_->y_doubleSpinBox->blockSignals(true);
  ui_->z_doubleSpinBox->blockSignals(true);
  ui_->yaw_doubleSpinBox->blockSignals(true);

  ui_->x_doubleSpinBox->setValue(position.x);
  ui_->y_doubleSpinBox->setValue(position.y);
  ui_->z_doubleSpinBox->setValue(position.z);

  double roll, pitch, yaw;
  tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
  ui_->yaw_doubleSpinBox->setValue(yaw);

  //enable the signals
  ui_->x_doubleSpinBox->blockSignals(false);
  ui_->y_doubleSpinBox->blockSignals(false);
  ui_->z_doubleSpinBox->blockSignals(false);
  ui_->yaw_doubleSpinBox->blockSignals(false);

  mission_str = std::to_string((current_mission+1)/2);
  mission_str += (current_mission%2? " pre" : " in");

  // std::cout << "wp index: " << wp_index << " | total wp: " << paths[current_mission].poses.size() << std::endl;
  if (wp_index >= 0 && (wp_index-1) < paths[current_mission].poses.size()) {
    paths[current_mission].poses[wp_index-1].pose.position.x = position.x;
    paths[current_mission].poses[wp_index-1].pose.position.y = position.y;
    paths[current_mission].poses[wp_index-1].pose.position.z = position.z;
    
    paths[current_mission].poses[wp_index-1].pose.orientation.x = quat.x;
    paths[current_mission].poses[wp_index-1].pose.orientation.y = quat.y;
    paths[current_mission].poses[wp_index-1].pose.orientation.z = quat.z;
    paths[current_mission].poses[wp_index-1].pose.orientation.w = quat.w;

    paths[current_mission].poses[wp_index-1].header.frame_id = "map";
    paths[current_mission].poses[wp_index-1].header.stamp = client_node_->now();
    // RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "updated waypoint %d on mission %s", (wp_index-1), mission_str.c_str());
  }
}

void WaypointNavPanel::setWpLabel() {
  std::ostringstream stringStream;
  stringStream << selected_marker_name_;
  std::string label = stringStream.str();

  ui_->sel_wp_label->setText(QString::fromStdString(label));
}

void WaypointNavPanel::saveWaypoint() {
  ui_->save_wp_button->setEnabled(false);

  std::string waypoint_yaml_path =
    ament_index_cpp::get_package_share_directory("rviz_plugin") + "/config";
  std::string default_filename = "waypoint.yaml";
  QString path = QFileDialog::getSaveFileName(
    this, tr("Save Waypoint Yaml"), (waypoint_yaml_path + "/" + default_filename).c_str(),
    tr("Waypoint Files (*.yaml *.json)"));

  if (path.isEmpty()) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Failed to Save Waypoint: Path Input Error");
  }
  const std::string path_str = path.toStdString();
  RVIZ_COMMON_LOG_INFO_STREAM("loading waypoints from " << path_str);
  if (path.endsWith(".json")) writeToJson(path_str);
  else writeToYaml(path_str);

  ui_->save_wp_button->setEnabled(true);
}

void WaypointNavPanel::writeToYaml(const std::string &filename) {
  YAML::Emitter out;  
  out << YAML::BeginMap;
  for (int i=1; i<paths.size(); i++) {
    nav_msgs::msg::Path &tempPath = paths[i];

    out << YAML::Key << "Mission";
    out << YAML::Value;
    out << YAML::BeginMap;

    for (const auto& pos : tempPath.poses) {
      out << YAML::Key << "Position";
      out << YAML::Value;
      out << YAML::Flow;
      out << YAML::BeginSeq;
      out << pos.pose.position.x;
      out << pos.pose.position.y;
      out << pos.pose.position.z;
      out << YAML::EndSeq;
    }
    out << YAML::EndMap;  
  }
  out << YAML::EndMap;

  std::ofstream fout(filename);
  if (fout.is_open()) {
    fout << out.c_str();
    fout.close();
  } else 
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_nav_frame"), "Could not open file for writing: %s", filename.c_str());

}

void WaypointNavPanel::writeToJson(const std::string &filename) {
  std::ofstream file(filename);

  json allPathsMission;
  for (int i=1; i<paths.size(); i++) { 
      nav_msgs::msg::Path *tempPath = &paths[i];
      json pathDetail;
      for (int a = 0; a < tempPath->poses.size(); a++) { 
          json poseDetail;
          poseDetail["x"] = tempPath->poses[a].pose.position.x;
          poseDetail["y"] = tempPath->poses[a].pose.position.y;
          poseDetail["z"] = tempPath->poses[a].pose.position.z;
          pathDetail.push_back(poseDetail);
      }
      allPathsMission.push_back(pathDetail);
  }
  file << allPathsMission;
  file.close();
}

void WaypointNavPanel::loadWaypoint() {
  ui_->load_wp_button->setEnabled(false);

  std::string waypoint_yaml_path =
    ament_index_cpp::get_package_share_directory("rviz_plugin") + "/config";
  std::string default_filename = "waypoint.yaml";
  QString path = QFileDialog::getOpenFileName(
    this, tr("load Waypoint Yaml"), (waypoint_yaml_path + "/" + default_filename).c_str(),
    tr("Waypoint Files (*.yaml *.json)"));

  if (path.isEmpty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("waypoint_panel"), 
      "Failed to load Waypoint: Path Input Error");
  }

  const std::string path_str = path.toStdString();
  RVIZ_COMMON_LOG_INFO_STREAM("loading waypoints from " << path_str);
  if (path.endsWith(".json")) readFromJson(path_str);
  else readFromYaml(path_str);

  ui_->load_wp_button->setEnabled(true);
}

void WaypointNavPanel::readFromYaml(const std::string &filename) {
  RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Start load file yaml");
  YAML::Node file = YAML::LoadFile(filename);
  paths[0].poses.clear();
  int i=1;

  for (const auto& temp_mission : file) {
    std::string mission_name = temp_mission.first.as<std::string>();
    YAML::Node mission = temp_mission.second;

    if (!mission.IsNull()) {
      int j=1;      

      for (const auto& temp_pose : mission) {
        std::string mission_name = temp_pose.first.as<std::string>();
        auto pose = temp_pose.second.as<std::vector<double>>();

        if (pose.size() == 3) {
          geometry_msgs::msg::PoseStamped pos;
          pos.pose.position.x = pose[0];
          pos.pose.position.y = pose[1];
          pos.pose.position.z = pose[2];
          paths[i].poses.push_back(pos);
          j++;
        } 
      }
    }
    i++;
  }
  RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "Load file yaml complete");  

  // ui_->missionComboBox->setCurrentIndex(0);
  // current_mission = 0;
  setMission(current_mission);
}

void WaypointNavPanel::readFromJson(const std::string &filename) {
  QString buffer;

  std::ifstream openFile(filename);
  json allPathMissionLoad;
  openFile >> allPathMissionLoad;
  paths[0].poses.clear();
  for(int i = 0; i < allPathMissionLoad.size(); i++) {
      if(!allPathMissionLoad[i].is_null()) {
          json pathDetail = allPathMissionLoad[i];
          paths[i+1].poses.clear();
          for(int a = 0; a <pathDetail.size(); a++) {
              json poseDetail = pathDetail[a];
              geometry_msgs::msg::PoseStamped pos;
              pos.pose.position.x = poseDetail["x"];
              pos.pose.position.y = poseDetail["y"];
              pos.pose.position.z = poseDetail["z"];
              paths[i+1].poses.push_back(pos);                
          }
      }
  }

  ui_->missionComboBox->setCurrentIndex(0);
  current_mission = 0;
  setMission(0);
}

void WaypointNavPanel::autonomy() {
  const QString button_style_template = "QPushButton {"
                                        "border-radius: 20px;"
                                        "background-color: %1;"
                                        "color: black;"
                                        "border: 1px solid black;"
                                        "padding: 10px 20px;"
                                        "}";

  const QString start_color = "#33b249";
  const QString stop_color = "#ED0800";

  if (!autonomy_client_->wait_for_service(0.25s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "service for starting mission not available");
    return;
  }
  auto future_result = std::shared_future<std::shared_ptr<nala2_interfaces::srv::StringService::Response>>();  

  QString command = ui_->command_button->text();
  if (command.contains("start", Qt::CaseInsensitive)) {
    ui_->command_button->setText("Stop Autonomy");
    ui_->command_button->setStyleSheet(button_style_template.arg(stop_color));

    auto request = std::make_shared<nala2_interfaces::srv::StringService::Request>();
    request->data = "start";

    auto future_result = autonomy_client_->async_send_request(request);
    RCLCPP_INFO(client_node_->get_logger(), "Service to start the mission is called");
  } 
  else {
    ui_->command_button->setText("Start Autonomy");
    ui_->command_button->setStyleSheet(button_style_template.arg(start_color));

    auto request = std::make_shared<nala2_interfaces::srv::StringService::Request>();
    request->data = "stop";

    auto future_result = autonomy_client_->async_send_request(request);
    RCLCPP_INFO(client_node_->get_logger(), "Service to stop the mission is called");
  }
}

void WaypointNavPanel::recordRoute() {
    const QString button_style_template = "QPushButton {"
    "border-radius: 20px;"
    "background-color: %1;"
    "color: black;"
    "border: 1px solid black;"
    "padding: 10px 20px;"
    "}";

  const QString start_color = "rgb(152, 106, 68)";
  const QString stop_color = "#ED0800";

  if (!route_client_->wait_for_service(0.25s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("waypoint_panel"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
      RCLCPP_INFO(rclcpp::get_logger("waypoint_panel"), "service for recording path not available");
      return;
  }
  auto future_result = std::shared_future<std::shared_ptr<nala2_interfaces::srv::StringService::Response>>();  

  QString command = ui_->path_button->text();
  if (command.contains("start", Qt::CaseInsensitive)) {
    ui_->path_button->setText("Stop Path");
    ui_->path_button->setStyleSheet(button_style_template.arg(stop_color));

    auto request = std::make_shared<nala2_interfaces::srv::StringService::Request>();
    request->data = "start";

    auto future_result = route_client_->async_send_request(request);
    RCLCPP_INFO(client_node_->get_logger(), "Service to start the recording of the path is called");
  } 
  else if (command.contains("clear", Qt::CaseInsensitive)) {
    ui_->path_button->setText("Clear Path");
    ui_->path_button->setStyleSheet(button_style_template.arg(start_color));

    auto request = std::make_shared<nala2_interfaces::srv::StringService::Request>();
    request->data = "clear";

    auto future_result = route_client_->async_send_request(request);
    RCLCPP_INFO(client_node_->get_logger(), "Service to clear the recorded path is called");
  }
  else {
    ui_->path_button->setText("Start Path");
    ui_->path_button->setStyleSheet(button_style_template.arg(start_color));

    auto request = std::make_shared<nala2_interfaces::srv::StringService::Request>();
    request->data = "stop";

    auto future_result = route_client_->async_send_request(request);
    RCLCPP_INFO(client_node_->get_logger(), "Service to stop the recording of the path is called");
  }
}

bool WaypointNavPanel::isPoseFarEnough(const geometry_msgs::msg::PoseStamped::SharedPtr& new_pose, 
                                        const geometry_msgs::msg::PoseStamped& last_pose) {
  double dx = new_pose->pose.position.x - last_pose.pose.position.x;
  double dy = new_pose->pose.position.y - last_pose.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  return distance >= 0.5; // Check if distance is at least 0.5 meters
}


double WaypointNavPanel::getDefaultHeight() {
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return default_height_;
}

QString WaypointNavPanel::getFrameId() {
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return frame_id_;
}

QString WaypointNavPanel::getOutputTopic() {
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return output_topic_;
}

rclcpp::Node::SharedPtr WaypointNavPanel::createNewNode(const std::string & node_name) {
  std::string node = "__node:=" + node_name;
  auto options = rclcpp::NodeOptions().arguments({"--ros-args", "--remap", node, "--"});
  return std::make_shared<rclcpp::Node>("_", options);
}

void WaypointNavPanel::timerEvent(QTimerEvent * event) {
  if (event->timerId() == timer_id_) rclcpp::spin_some(client_node_);
}
} // namespace

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoint_nav_plugin::WaypointNavPanel, rviz_common::Panel)