//
// Created by ljq on 2022/5/17.
//

#include "rm_referee/referee_base.h"
#include <codecvt>

namespace rm_referee
{
RefereeBase::RefereeBase(ros::NodeHandle& nh, Base& base) : base_(base), nh_(nh)
{
  dbus_topic_ = getParam(nh, "dbus_topic", std::string("/rm_ecat_hw/dbus"));
  RefereeBase::joint_state_sub_ =
      nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, &RefereeBase::jointStateCallback, this);
  RefereeBase::actuator_state_sub_ =
      nh.subscribe<rm_msgs::ActuatorState>("/actuator_states", 10, &RefereeBase::actuatorStateCallback, this);
  RefereeBase::dbus_sub_ = nh.subscribe<rm_msgs::DbusData>(dbus_topic_, 10, &RefereeBase::dbusDataCallback, this);
  RefereeBase::chassis_cmd_sub_ =
      nh.subscribe<rm_msgs::ChassisCmd>("/cmd_chassis", 10, &RefereeBase::chassisCmdDataCallback, this);
  RefereeBase::vel2D_cmd_sub_ =
      nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RefereeBase::vel2DCmdDataCallback, this);
  RefereeBase::shoot_state_sub_ = nh.subscribe<rm_msgs::ShootState>("/controllers/shooter_controller/state", 10,
                                                                    &RefereeBase::shootStateCallback, this);
  RefereeBase::gimbal_cmd_sub_ = nh.subscribe<rm_msgs::GimbalCmd>("/controllers/gimbal_controller/command", 10,
                                                                  &RefereeBase::gimbalCmdDataCallback, this);
  RefereeBase::card_cmd_sub_ = nh.subscribe<rm_msgs::StateCmd>("/controllers/card_controller/command", 10,
                                                               &RefereeBase::cardCmdDataCallback, this);
  RefereeBase::engineer_cmd_sub_ =
      nh.subscribe<rm_msgs::EngineerUi>("/engineer_ui", 10, &RefereeBase::engineerUiDataCallback, this);
  RefereeBase::manual_data_sub_ =
      nh.subscribe<rm_msgs::ManualToReferee>("/manual_to_referee", 10, &RefereeBase::manualDataCallBack, this);
  RefereeBase::camera_name_sub_ = nh.subscribe("/camera_name", 10, &RefereeBase::cameraNameCallBack, this);
  RefereeBase::balance_state_sub_ = nh.subscribe("/state", 10, &RefereeBase::balanceStateCallback, this);
  RefereeBase::track_sub_ = nh.subscribe<rm_msgs::TrackData>("/track", 10, &RefereeBase::trackCallBack, this);
  RefereeBase::map_sentry_sub_ =
      nh.subscribe<rm_msgs::MapSentryData>("/map_sentry_data", 10, &RefereeBase::mapSentryCallback, this);
  RefereeBase::radar_receive_sub_ =
      nh.subscribe<rm_msgs::ClientMapReceiveData>("/rm_radar", 10, &RefereeBase::radarReceiveCallback, this);
  RefereeBase::sentry_cmd_sub_ =
      nh.subscribe<rm_msgs::SentryCmd>("/sentry_cmd", 1, &RefereeBase::sendSentryCmdCallback, this);
  RefereeBase::radar_cmd_sub_ =
      nh.subscribe<rm_msgs::RadarInfo>("/radar_cmd", 1, &RefereeBase::sendRadarCmdCallback, this);
  RefereeBase::sentry_state_sub_ =
      nh.subscribe<std_msgs::String>("/custom_info", 1, &RefereeBase::sendCustomInfoCallback, this);
  RefereeBase::drone_pose_sub_ =
      nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1, &RefereeBase::dronePoseCallBack, this);
  RefereeBase::shoot_cmd_sub_ = nh.subscribe<rm_msgs::ShootCmd>("/controllers/shooter_controller/command", 1,
                                                                &RefereeBase::shootCmdCallBack, this);
  RefereeBase::sentry_to_referee_sub_ = nh.subscribe<rm_msgs::SentryAttackingTarget>(
      "/sentry_target_to_referee", 1, &RefereeBase::sentryAttackingTargetCallback, this);
  RefereeBase::radar_to_referee_sub_ =
      nh.subscribe<rm_msgs::RadarToSentry>("/radar_to_referee", 1, &RefereeBase::radarToRefereeCallBack, this);
  RefereeBase::customize_display_cmd_sub_ =
      nh.subscribe<std_msgs::UInt32>("/customize_display_ui", 1, &RefereeBase::customizeDisplayCmdCallBack, this);
  RefereeBase::visualize_state_data_sub_ =
      nh.subscribe<rm_msgs::VisualizeStateData>("/visualize_state", 1, &RefereeBase::visualizeStateDataCallBack, this);

  XmlRpc::XmlRpcValue rpc_value;
  send_ui_queue_delay_ = getParam(nh, "send_ui_queue_delay", 0.15);
  add_ui_frequency_ = getParam(nh, "add_ui_frequency", 5);
  add_ui_max_times_ = getParam(nh, "add_ui_max_times", 10);
  interactive_data_sender_ = new InteractiveSender(rpc_value, base_);
  if (nh.hasParam("ui"))
  {
    ros::NodeHandle ui_nh(nh, "ui");
    graph_queue_sender_ = new GroupUiBase(rpc_value, base_);
    ui_nh.getParam("trigger_change", rpc_value);
    for (int i = 0; i < rpc_value.size(); i++)
    {
      if (rpc_value[i]["name"] == "chassis")
        chassis_trigger_change_ui_ = new ChassisTriggerChangeUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "shooter")
        shooter_trigger_change_ui_ = new ShooterTriggerChangeUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "gimbal")
        gimbal_trigger_change_ui_ = new GimbalTriggerChangeUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "target")
        target_trigger_change_ui_ = new TargetTriggerChangeUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "target_view_angle")
        target_view_angle_trigger_change_ui_ =
            new TargetViewAngleTriggerChangeUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "camera")
        camera_trigger_change_ui_ = new CameraTriggerChangeUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "friction_speed")
        friction_speed_trigger_change_ui_ =
            new FrictionSpeedTriggerChangeUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "gripper")
        gripper_state_trigger_change_ui_ =
            new StringTriggerChangeUi(rpc_value[i], base_, "gripper", &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "servo_mode")
        servo_mode_trigger_change_ui_ =
            new StringTriggerChangeUi(rpc_value[i], base_, "servo_mode", &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "stone")
        stone_num_trigger_change_ui_ =
            new StringTriggerChangeUi(rpc_value[i], base_, "stone_num", &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "visualize_state")
        visualize_state_trigger_change_ui_ =
            new VisualizeStateTriggerChangeUi(rpc_value[i], base_, "visualize_state", &graph_queue_, &character_queue_);
    }

    ui_nh.getParam("time_change", rpc_value);
    for (int i = 0; i < rpc_value.size(); i++)
    {
      if (rpc_value[i]["name"] == "capacitor")
        capacitor_time_change_ui_ = new CapacitorTimeChangeUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "effort")
        effort_time_change_ui_ = new EffortTimeChangeUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "progress")
        progress_time_change_ui_ = new ProgressTimeChangeUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "dart_status")
        dart_status_time_change_ui_ = new DartStatusTimeChangeUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "rotation")
        rotation_time_change_ui_ = new RotationTimeChangeUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "lane_line")
        lane_line_time_change_ui_ =
            new LaneLineTimeChangeGroupUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "pitch")
        pitch_angle_time_change_ui_ = new PitchAngleTimeChangeUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "balance_pitch")
        balance_pitch_time_change_group_ui_ =
            new BalancePitchTimeChangeGroupUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "engineer_joint1")
        engineer_joint1_time_change_ui =
            new JointPositionTimeChangeUi(rpc_value[i], base_, &graph_queue_, &character_queue_, "joint1");
      if (rpc_value[i]["name"] == "engineer_joint2")
        engineer_joint2_time_change_ui =
            new JointPositionTimeChangeUi(rpc_value[i], base_, &graph_queue_, &character_queue_, "joint2");
      if (rpc_value[i]["name"] == "engineer_joint3")
        engineer_joint3_time_change_ui =
            new JointPositionTimeChangeUi(rpc_value[i], base_, &graph_queue_, &character_queue_, "joint3");
      if (rpc_value[i]["name"] == "bullet")
        bullet_time_change_ui_ = new BulletTimeChangeUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "target_distance")
        target_distance_time_change_ui_ =
            new TargetDistanceTimeChangeUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "drone_towards")
        drone_towards_time_change_group_ui_ =
            new DroneTowardsTimeChangeGroupUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "friend_bullets")
        friend_bullets_time_change_group_ui_ =
            new FriendBulletsTimeChangeGroupUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
    }

    ui_nh.getParam("fixed", rpc_value);
    fixed_ui_ = new FixedUi(rpc_value, base_, &graph_queue_, &character_queue_);

    ui_nh.getParam("flash", rpc_value);
    for (int i = 0; i < rpc_value.size(); i++)
    {
      if (rpc_value[i]["name"] == "cover")
        cover_flash_ui_ = new CoverFlashUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "spin")
        spin_flash_ui_ = new SpinFlashUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "hero_hit")
        hero_hit_flash_ui_ = new HeroHitFlashUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "exceed_bullet_speed")
        exceed_bullet_speed_flash_ui_ =
            new ExceedBulletSpeedFlashUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
      if (rpc_value[i]["name"] == "customize_display")
        customize_display_flash_ui_ =
            new CustomizeDisplayFlashUi(rpc_value[i], base_, &graph_queue_, &character_queue_);
    }
  }
  if (nh.hasParam("interactive_data"))
  {
    nh.getParam("interactive_data", rpc_value);
    for (int i = 0; i < rpc_value.size(); i++)
    {
      if (rpc_value[i]["name"] == "custom_info")
        custom_info_sender = new CustomInfoSender(rpc_value[i], base_);
      if (rpc_value[i]["name"] == "bullet_num_share")
        bullet_num_share_ = new BulletNumShare(rpc_value[i], base_);
      if (rpc_value[i]["name"] == "sentry_to_radar")
        sentry_to_radar_ = new SentryToRadar(rpc_value[i], base_);
      if (rpc_value[i]["name"] == "radar_to_sentry")
        radar_to_sentry_ = new RadarToSentry(rpc_value[i], base_);
    }
  }

  add_ui_timer_ =
      nh.createTimer(ros::Duration(1. / add_ui_frequency_), std::bind(&RefereeBase::addUi, this), false, false);
  send_serial_data_timer_ = nh.createTimer(ros::Duration(send_ui_queue_delay_),
                                           std::bind(&RefereeBase::sendSerialDataCallback, this), false, true);
}
void RefereeBase::addUi()
{
  if (add_ui_times_ > add_ui_max_times_)
  {
    ROS_INFO_THROTTLE(2.0, "End adding");
    add_ui_timer_.stop();
    if (!graph_queue_.empty())
    {
      graph_queue_.clear();
      ROS_WARN_THROTTLE(0.5, "Some UI is not add completely, now clear the queue");
    }
    is_adding_ = false;
    send_serial_data_timer_.setPeriod(ros::Duration(send_ui_queue_delay_));
    return;
  }

  ROS_INFO_THROTTLE(1.0, "Adding ui... %.1f%%", (add_ui_times_ / static_cast<double>(add_ui_max_times_)) * 100);
  if (chassis_trigger_change_ui_)
    chassis_trigger_change_ui_->addForQueue(2);
  if (gimbal_trigger_change_ui_)
    gimbal_trigger_change_ui_->addForQueue(2);
  if (shooter_trigger_change_ui_)
    shooter_trigger_change_ui_->addForQueue(2);
  if (target_trigger_change_ui_)
    target_trigger_change_ui_->addForQueue();
  if (target_view_angle_trigger_change_ui_)
    target_view_angle_trigger_change_ui_->addForQueue();
  if (camera_trigger_change_ui_)
    camera_trigger_change_ui_->addForQueue();
  if (fixed_ui_)
    fixed_ui_->addForQueue();
  if (effort_time_change_ui_)
    effort_time_change_ui_->addForQueue();
  if (progress_time_change_ui_)
    progress_time_change_ui_->addForQueue();
  if (dart_status_time_change_ui_)
    dart_status_time_change_ui_->addForQueue();
  if (capacitor_time_change_ui_)
    capacitor_time_change_ui_->addForQueue();
  if (rotation_time_change_ui_)
    rotation_time_change_ui_->addForQueue();
  if (lane_line_time_change_ui_)
    lane_line_time_change_ui_->addForQueue();
  if (balance_pitch_time_change_group_ui_)
    balance_pitch_time_change_group_ui_->addForQueue();
  if (pitch_angle_time_change_ui_)
    pitch_angle_time_change_ui_->addForQueue();
  if (engineer_joint1_time_change_ui)
    engineer_joint1_time_change_ui->addForQueue();
  if (engineer_joint2_time_change_ui)
    engineer_joint2_time_change_ui->addForQueue();
  if (engineer_joint3_time_change_ui)
    engineer_joint3_time_change_ui->addForQueue();
  if (drone_towards_time_change_group_ui_)
    drone_towards_time_change_group_ui_->addForQueue();
  if (gripper_state_trigger_change_ui_)
    gripper_state_trigger_change_ui_->addForQueue();
  if (stone_num_trigger_change_ui_)
    stone_num_trigger_change_ui_->addForQueue();
  if (servo_mode_trigger_change_ui_)
    servo_mode_trigger_change_ui_->addForQueue();
  if (friction_speed_trigger_change_ui_)
    friction_speed_trigger_change_ui_->addForQueue();
  if (bullet_time_change_ui_)
  {
    bullet_time_change_ui_->reset();
    bullet_time_change_ui_->addForQueue();
  }
  if (target_distance_time_change_ui_)
    target_distance_time_change_ui_->addForQueue();
  if (friend_bullets_time_change_group_ui_)
    friend_bullets_time_change_group_ui_->addForQueue();
  if (visualize_state_trigger_change_ui_)
    visualize_state_trigger_change_ui_->addForQueue();
  add_ui_times_++;
}

void RefereeBase::sendSerialDataCallback()
{
  if (bullet_num_share_ && bullet_num_share_->needSendInteractiveData())
  {
    bullet_num_share_->sendBulletData();
    return;
  }
  if (sentry_to_radar_ && sentry_to_radar_->needSendInteractiveData())
  {
    sentry_to_radar_->sendSentryToRadarData();
    return;
  }
  if (radar_to_sentry_ && radar_to_sentry_->needSendInteractiveData())
  {
    radar_to_sentry_->sendRadarToSentryData();
    return;
  }

  if (graph_queue_.empty() && character_queue_.empty())
    return;

  if (!is_adding_)
  {
    if (graph_queue_.size() > 50)
    {
      ROS_WARN_THROTTLE(0.5, "Sending graph UI too frequently, please modify the configuration file or code to"
                             "reduce the frequency . Now pop the queue");
      while (graph_queue_.size() > 50)
        graph_queue_.pop_front();
    }

    if (character_queue_.size() > 8)
    {
      ROS_WARN_THROTTLE(0.5, "Sending character UI too frequently, please modify the configuration file or code to"
                             "reduce the frequency . Now pop the queue");
      while (character_queue_.size() > 8)
        character_queue_.pop_front();
    }
    sendQueue();
  }
  else
    sendQueue();

  if (base_.robot_id_ == 0)
    ROS_WARN_THROTTLE(1.0, "robot base id = 0, the serial or referee system may not be connected");

  if (base_.client_id_ == 0)
    ROS_WARN_THROTTLE(1.0, "client base id = 0, the serial or referee system may not be connected\"");
  send_serial_data_timer_.start();
}

void RefereeBase::sendQueue()
{
  if (!character_queue_.empty() && graph_queue_.size() <= 14)
  {
    graph_queue_sender_->sendCharacter(ros::Time::now(), &character_queue_.at(0));
    character_queue_.pop_front();
  }
  else if (graph_queue_.size() >= 7)
  {
    graph_queue_sender_->sendSevenGraph(ros::Time::now(), &graph_queue_.at(0), &graph_queue_.at(1), &graph_queue_.at(2),
                                        &graph_queue_.at(3), &graph_queue_.at(4), &graph_queue_.at(5),
                                        &graph_queue_.at(6));
    for (int i = 0; i < 7; i++)
      graph_queue_.pop_front();
  }
  else if (graph_queue_.size() >= 5)
  {
    graph_queue_sender_->sendFiveGraph(ros::Time::now(), &graph_queue_.at(0), &graph_queue_.at(1), &graph_queue_.at(2),
                                       &graph_queue_.at(3), &graph_queue_.at(4));
    for (int i = 0; i < 5; i++)
      graph_queue_.pop_front();
  }
  else if (graph_queue_.size() >= 2)
  {
    graph_queue_sender_->sendDoubleGraph(ros::Time::now(), &graph_queue_.at(0), &graph_queue_.at(1));
    for (int i = 0; i < 2; i++)
      graph_queue_.pop_front();
  }
  else if (graph_queue_.size() == 1)
  {
    graph_queue_sender_->sendSingleGraph(ros::Time::now(), &graph_queue_.at(0));
    graph_queue_.pop_front();
  }
}

void RefereeBase::robotStatusDataCallBack(const rm_msgs::GameRobotStatus& data, const ros::Time& last_get_data_time)
{
  if (fixed_ui_ && !is_adding_)
    fixed_ui_->updateForQueue();
}

void RefereeBase::updateHeroHitDataCallBack(const rm_msgs::GameRobotHp& game_robot_hp_data)
{
  if (hero_hit_flash_ui_)
    hero_hit_flash_ui_->updateHittingConfig(game_robot_hp_data);
}
void RefereeBase::gameStatusDataCallBack(const rm_msgs::GameStatus& data, const ros::Time& last_get_data_time)
{
}
void RefereeBase::capacityDataCallBack(const rm_msgs::PowerManagementSampleAndStatusData& data,
                                       ros::Time& last_get_data_time)
{
  if (capacitor_time_change_ui_ && !is_adding_)
    capacitor_time_change_ui_->updateRemainCharge(data.capacity_remain_charge, last_get_data_time);
  if (chassis_trigger_change_ui_ && !is_adding_)
    chassis_trigger_change_ui_->updateCapacityResetStatus();
}
void RefereeBase::powerHeatDataCallBack(const rm_msgs::PowerHeatData& data, const ros::Time& last_get_data_time)
{
}
void RefereeBase::robotHurtDataCallBack(const rm_msgs::RobotHurt& data, const ros::Time& last_get_data_time)
{
}
void RefereeBase::bulletRemainDataCallBack(const rm_msgs::BulletAllowance& bullet_allowance,
                                           const ros::Time& last_get_data_time)
{
  if (bullet_time_change_ui_ && !is_adding_)
    bullet_time_change_ui_->updateBulletData(bullet_allowance, last_get_data_time);
  if (bullet_num_share_ && !is_adding_)
    bullet_num_share_->updateBulletRemainData(bullet_allowance);
}
void RefereeBase::interactiveDataCallBack(const rm_referee::InteractiveData& data, const ros::Time& last_get_data_time)
{
}
void RefereeBase::eventDataCallBack(const rm_msgs::EventData& data, const ros::Time& last_get_data_time)
{
}
void RefereeBase::jointStateCallback(const sensor_msgs::JointState::ConstPtr& data)
{
  if (effort_time_change_ui_ && !is_adding_)
    effort_time_change_ui_->updateJointStateData(data, ros::Time::now());
  if (rotation_time_change_ui_ && !is_adding_)
    rotation_time_change_ui_->updateForQueue();
  if (lane_line_time_change_ui_ && !is_adding_)
    lane_line_time_change_ui_->updateJointStateData(data, ros::Time::now());
  if (pitch_angle_time_change_ui_ && !is_adding_)
    pitch_angle_time_change_ui_->updateJointStateData(data, ros::Time::now());
  if (engineer_joint1_time_change_ui && !is_adding_)
    engineer_joint1_time_change_ui->updateJointStateData(data, ros::Time::now());
  if (engineer_joint2_time_change_ui && !is_adding_)
    engineer_joint2_time_change_ui->updateJointStateData(data, ros::Time::now());
  if (engineer_joint3_time_change_ui && !is_adding_)
    engineer_joint3_time_change_ui->updateJointStateData(data, ros::Time::now());
}
void RefereeBase::actuatorStateCallback(const rm_msgs::ActuatorState::ConstPtr& data)
{
}
void RefereeBase::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  if (add_ui_flag_ && data->s_r == rm_msgs::DbusData::UP)
  {
    add_ui_flag_ = false;
    is_adding_ = true;
    if (!graph_queue_.empty())
      graph_queue_.clear();
    send_serial_data_timer_.setPeriod(ros::Duration(0.05));
    add_ui_timer_.start();
    add_ui_times_ = 0;
  }
  if (data->s_r != rm_msgs::DbusData::UP)
  {
    add_ui_flag_ = true;
    add_ui_timer_.stop();
  }
  if (chassis_trigger_change_ui_)
    chassis_trigger_change_ui_->updateDbusData(data);
}
void RefereeBase::chassisCmdDataCallback(const rm_msgs::ChassisCmd::ConstPtr& data)
{
  if (chassis_trigger_change_ui_)
    chassis_trigger_change_ui_->updateChassisCmdData(data);
  if (spin_flash_ui_ && !is_adding_)
    spin_flash_ui_->updateChassisCmdData(data, ros::Time::now());
  if (rotation_time_change_ui_ && !is_adding_)
    rotation_time_change_ui_->updateChassisCmdData(data);
}
void RefereeBase::vel2DCmdDataCallback(const geometry_msgs::Twist::ConstPtr& data)
{
}
void RefereeBase::shootStateCallback(const rm_msgs::ShootState::ConstPtr& data)
{
  if (target_trigger_change_ui_ && !is_adding_)
    target_trigger_change_ui_->updateShootStateData(data);
  if (shooter_trigger_change_ui_ && !is_adding_)
    shooter_trigger_change_ui_->updateShootStateData(data);
}
void RefereeBase::gimbalCmdDataCallback(const rm_msgs::GimbalCmd::ConstPtr& data)
{
  if (gimbal_trigger_change_ui_ && !is_adding_)
    gimbal_trigger_change_ui_->updateGimbalCmdData(data);
}
void RefereeBase::cardCmdDataCallback(const rm_msgs::StateCmd::ConstPtr& data)
{
}
void RefereeBase::engineerUiDataCallback(const rm_msgs::EngineerUi::ConstPtr& data)
{
  if (gripper_state_trigger_change_ui_ && !is_adding_)
    gripper_state_trigger_change_ui_->updateStringUiData(data->gripper_state);
  if (stone_num_trigger_change_ui_ && !is_adding_)
    stone_num_trigger_change_ui_->updateStringUiData(std::to_string(data->stone_num));
  if (servo_mode_trigger_change_ui_ && !is_adding_)
    servo_mode_trigger_change_ui_->updateStringUiData(data->control_mode);
  if (customize_display_flash_ui_ && !is_adding_)
    customize_display_flash_ui_->updateCmdData(data->symbol);
}
void RefereeBase::manualDataCallBack(const rm_msgs::ManualToReferee::ConstPtr& data)
{
  if (chassis_trigger_change_ui_)
    chassis_trigger_change_ui_->updateManualCmdData(data);
  if (shooter_trigger_change_ui_ && !is_adding_)
    shooter_trigger_change_ui_->updateManualCmdData(data);
  if (gimbal_trigger_change_ui_ && !is_adding_)
    gimbal_trigger_change_ui_->updateManualCmdData(data);
  if (target_trigger_change_ui_ && !is_adding_)
    target_trigger_change_ui_->updateManualCmdData(data);
  if (cover_flash_ui_ && !is_adding_)
    cover_flash_ui_->updateManualCmdData(data, ros::Time::now());
  if (customize_display_flash_ui_ && !is_adding_)
    customize_display_flash_ui_->updateCmdData(data->ui_display_symbol);
}
void RefereeBase::radarDataCallBack(const std_msgs::Int8MultiArrayConstPtr& data)
{
}
void RefereeBase::cameraNameCallBack(const std_msgs::StringConstPtr& data)
{
  if (camera_trigger_change_ui_ && !is_adding_)
    camera_trigger_change_ui_->updateCameraName(data);
}
void RefereeBase::trackCallBack(const rm_msgs::TrackDataConstPtr& data)
{
  if (target_view_angle_trigger_change_ui_ && !is_adding_)
    target_view_angle_trigger_change_ui_->updateTrackID(data->id);
  if (target_distance_time_change_ui_ && !is_adding_)
    target_distance_time_change_ui_->updateTargetDistanceData(data);
}
void RefereeBase::balanceStateCallback(const rm_msgs::BalanceStateConstPtr& data)
{
  if (balance_pitch_time_change_group_ui_)
    balance_pitch_time_change_group_ui_->calculatePointPosition(data, ros::Time::now());
}
void RefereeBase::sentryAttackingTargetCallback(const rm_msgs::SentryAttackingTargetConstPtr& data)
{
  if (sentry_to_radar_)
    sentry_to_radar_->updateSentryAttackingTargetData(data);
}
void RefereeBase::radarReceiveCallback(const rm_msgs::ClientMapReceiveData::ConstPtr& data)
{
  if (ros::Time::now() - radar_interactive_data_last_send_ <= ros::Duration(0.2))
    return;
  else
  {
    interactive_data_sender_->sendRadarInteractiveData(data);
    radar_interactive_data_last_send_ = ros::Time::now();
  }
}
void RefereeBase::mapSentryCallback(const rm_msgs::MapSentryDataConstPtr& data)
{
  rm_referee::MapSentryData map_sentry_data;
  map_sentry_data.intention = data->intention;
  map_sentry_data.start_position_x = data->start_position_x;
  map_sentry_data.start_position_y = data->start_position_y;
  for (int i = 0; i < 49; i++)
  {
    map_sentry_data.delta_x[i] = data->delta_x[i];
    map_sentry_data.delta_y[i] = data->delta_y[i];
  }
  if (ros::Time::now() - sentry_interactive_data_last_send_ <= ros::Duration(1.0))
    return;
  else
  {
    interactive_data_sender_->sendMapSentryData(map_sentry_data);
    sentry_interactive_data_last_send_ = ros::Time::now();
  }
}
void RefereeBase::sendSentryCmdCallback(const rm_msgs::SentryCmdConstPtr& data)
{
  if (ros::Time::now() - sentry_cmd_data_last_send_ <= ros::Duration(0.15))
    return;
  else
  {
    interactive_data_sender_->sendSentryCmdData(data);
    sentry_cmd_data_last_send_ = ros::Time::now();
  }
}
void RefereeBase::sendRadarCmdCallback(const rm_msgs::RadarInfoConstPtr& data)
{
  if (ros::Time::now() - radar_cmd_data_last_send_ <= ros::Duration(0.15))
    return;
  else
  {
    interactive_data_sender_->sendRadarCmdData(data);
    radar_cmd_data_last_send_ = ros::Time::now();
  }
}

void RefereeBase::sendCustomInfoCallback(const std_msgs::StringConstPtr& data)
{
  std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
  if (custom_info_sender)
    custom_info_sender->sendCustomInfoData(converter.from_bytes(data->data));
}

void RefereeBase::supplyBulletDataCallBack(const rm_msgs::SupplyProjectileAction& data)
{
}

void RefereeBase::dronePoseCallBack(const geometry_msgs::PoseStampedConstPtr& data)
{
  if (drone_towards_time_change_group_ui_ && !is_adding_)
    drone_towards_time_change_group_ui_->updateTowardsData(data);
}

void RefereeBase::updateShootDataDataCallBack(const rm_msgs::ShootData& msg)
{
  if (exceed_bullet_speed_flash_ui_ && !is_adding_)
    exceed_bullet_speed_flash_ui_->updateShootData(msg);
}

void RefereeBase::shootCmdCallBack(const rm_msgs::ShootCmdConstPtr& data)
{
  if (friction_speed_trigger_change_ui_ && !is_adding_)
    friction_speed_trigger_change_ui_->updateFrictionSpeedUiData(data);
}

void RefereeBase::updateBulletRemainData(const rm_referee::BulletNumData& data)
{
  if (friend_bullets_time_change_group_ui_ && !is_adding_)
    friend_bullets_time_change_group_ui_->updateBulletsData(data);
}

void RefereeBase::radarToRefereeCallBack(const rm_msgs::RadarToSentryConstPtr& data)
{
  if (radar_to_sentry_)
    radar_to_sentry_->updateRadarToSentryData(data);
}

void RefereeBase::customizeDisplayCmdCallBack(const std_msgs::UInt32ConstPtr& data)
{
  if (customize_display_flash_ui_ && !is_adding_)
    customize_display_flash_ui_->updateCmdData(data->data);
}

void RefereeBase::visualizeStateDataCallBack(const rm_msgs::VisualizeStateDataConstPtr& data)
{
  if (visualize_state_trigger_change_ui_ && !is_adding_)
  {
    std::vector<bool> state;
    for (auto state_data : data->state)
      state.push_back(state_data);
    visualize_state_trigger_change_ui_->updateUiColor(state);
  }
}

}  // namespace rm_referee
