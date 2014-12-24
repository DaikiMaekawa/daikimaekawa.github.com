---
title: "ros_controlでノード間通信に依らずロボット間のハードウェアの違いを吸収する"
layout: post
category : ROS
tagline:
tags : [ROS, ros_control]
---

{% include JB/setup %}

この投稿は[ROS Advent Calendar 2014](http://qiita.com/advent-calendar/2014/ros)の19日目の記事です。

## はじめに

ros_controlを使用することでROSのノード間通信に依らずロボット間のハードウェアの違いを吸収し、モジュールの可搬性を確保することが可能です。

元はpr2_mechanismとして開発されていたものをPR2以外のロボットにも適用可能なように一般化したものです。

リアルタイム性のある制御コントローラを作成してシステムを構築できることが強みであり、詳しくは[こちら](https://github.com/ros-controls/ros_control/issues/130#issue-22769080)を参照してください。

また、シミュレータと実機で共通のコントローラを使用することも可能で、ros_controlの概要と合わせてこれからその方法を解説します。

http://wiki.ros.org/ros_control

## ros_controlの概要

![transmission]({{BASE_PATH}}/images/ros_control/gazebo_transmission.png)

大まかにコントローラ、ハードウェアインタフェース、トランスミッションの三つの要素から構成されます。

### 1. Controllers

コントローラはプラグインとして入れ替えが可能で、シミュレータと実機との互換性も確保されています。

プラグインは以下の種類が[ros_controllers](http://wiki.ros.org/ros_controllers)に用意されています。

 * effort_controllers
    * joint_effort_controller
    * joint_position_controller
    * joint_velocity_controller 

 * joint_state_controller
    * joint_state_controller 

 * position_controllers
    * joint_position_controller 

 * velocity_controllers
    * joint_velocity_controllers 

もちろん独自のプラグインも作成可能で詳細は[こちら](https://github.com/ros-controls/ros_control/wiki/controller_interface)

### 2. Hardware Interfaces

ロボットに必要なハードウェアインタフェースを以下のリストから使用することができます。（独自のインタフェースを作成する方法は[こちら](https://github.com/ros-controls/ros_control/wiki/hardware_interface)）

 * Joint Command Interfaces
    * Effort Joint Interface
    * Velocity Joint Interface
    * Position Joint Interface 

 * Joint State Interfaces
 * Actuator State Interfaces
 * Actuator Command Interfaces
    * Effort Actuator Interface
    * Velocity Actuator Interface
    * Position Actuator Interface 

 * Force-torque sensor Interface
 * IMU sensor Interface 

### 3. Transmissions

アクチュエータと関節との関係を記述するためのものでギア比や平行リンク等の概念をモデル化することができます。

詳細は[こちら](https://github.com/ros-controls/ros_control/wiki/transmission_interface)

## ロボットのモデルを定義

これらを差動二輪型のロボットに適用した場合のURDFの一例を示します。(細かい部分は省略)

モデルを記述するためにsample_robot.xacroというファイルを作成し、まずは回転ジョイントを二つ定義します。

**sample_robot.xacro**

```xml
<robot name="sample_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  ...
  
  <joint type="revolute" name="left_wheel_hinge">
    <origin xyz="0 ${wheel_dist} ${wheel_height}" rpy="0 0 0"/>
    <child link="left_wheel">left_wheel</child>
    <parent link="base_link">base_link</parent>
    <axis xyz="0 1 0"/>
    <limit effort="100" velocity="100.0" lower="-500" upper="500" />
  </joint> 

  <joint type="revolute" name="right_wheel_hinge">
    <origin xyz="0 -${wheel_dist} ${wheel_height}" rpy="0 0 0"/>
    <child link="right_wheel">right_wheel</child>
    <parent link="base_link">base_link</parent>
    <axis xyz="0 1 0"/>
    <limit effort="100" velocity="100.0" lower="-500" upper="500" />
  </joint>

  ...
</robot>
```

アクチュエータと関節との関係は以下のように記述できます。

**sample_robot.xacro**

```xml
<transmission name="tran1">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_wheel_hinge">
    <hardwareInterface>VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_motor">
    <hardwareInterface>VelocityJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>

<transmission name="tran2">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="right_wheel_hinge">
    <hardwareInterface>VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="right_motor">
    <hardwareInterface>VelocityJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

これをlaunchファイルで読み込んでrobot_descriptionパラメータとして共有しましょう。

**sample_robot_driver.launch**

```xml
<param name="robot_description"
    command="$(find xacro)/xacro.py '$(find sample_robot_description)/urdf/sample_robot.xacro'" />
```

## コントローラの読み込み

使用するコントローラの設定を記述します。

**sample_robot.yaml**

```yaml
joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 100

sample_robot:
    type        : "diff_drive_controller/DiffDriveController"
    left_wheel  : 'left_wheel_hinge'
    right_wheel : 'right_wheel_hinge'
    publish_rate: 100
    
    pose_covariance_diagonal : [0.00001, 0.00001, 1000000000000.0, 1000000000000.0, 1000000000000.0, 0.001]
    twist_covariance_diagonal: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    wheel_separation_multiplier: 1.0
    wheel_radius_multiplier    : 1.0
    cmd_vel_timeout: 20
    base_frame_id: base_link

    linear:
      x:
        has_velocity_limits    : true
        max_velocity           : 0.9  # m/s
        min_velocity           : -0.9 # m/s
        has_acceleration_limits: true
        max_acceleration       : 1.7  # m/s^2
        min_acceleration       : -0.4 # m/s^2
    angular:
      z:
        has_velocity_limits    : true
        max_velocity           : 0.5  # rad/s
        has_acceleration_limits: true
        max_acceleration       : 1.5  # rad/s^2
```

DiffDriveControllerはVelocityJointInterfaceが設定されている回転ジョイントをleft_wheelとright_wheelに指定することでgeometry_msgs::Twistメッセージから必要な車輪の回転数を計算し、モデルの進んだ量からnav_msgs::Odometryメッセージのパブリッシュとtfの発行も行われます。

controller_managerに設定を読み込ませます。

**sample_robot_control.yaml**

```xml
<!-- Load joint controller configurations from YAML file to parameter server -->
<rosparam file="$(find sample_robot_control)/config/sample_robot_control.yaml" command="load"/>

<!-- load the controllers -->
<node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
    output="screen" args="sample_robot joint_state_controller">
</node>

```

## RobotHW


コントローラへの入出力と実機への入出力関係を管理するためRobotHWを継承したSampleRobotHWを定義する。

**sample_robot_driver.cpp**

```cpp
class SampleRobotHW : public hardware_interface::RobotHW
{
public:
  SampleRobotHW(){
    pos_[0] = 0.0; pos_[1] = 0.0;
    vel_[0] = 0.0; vel_[1] = 0.0;
    eff_[0] = 0.0; eff_[1] = 0.0;
    cmd_[0] = 0.0; cmd_[1] = 0.0;

    hardware_interface::JointStateHandle state_handle_1("right_wheel_hinge", &pos_[0], &vel_[0], &eff_[0]);
    jnt_state_interface_.registerHandle(state_handle_1);

    hardware_interface::JointStateHandle state_handle_2("left_wheel_hinge", &pos_[1], &vel_[1], &eff_[1]);
    jnt_state_interface_.registerHandle(state_handle_2);

    registerInterface(&jnt_state_interface_);

    hardware_interface::JointHandle vel_handle_1(jnt_state_interface_.getHandle("right_wheel_hinge"), &cmd_[0]);
    jnt_vel_interface_.registerHandle(vel_handle_1);

    hardware_interface::JointHandle vel_handle_2(jnt_state_interface_.getHandle("left_wheel_hinge"), &cmd_[1]);
    jnt_vel_interface_.registerHandle(vel_handle_2);

    registerInterface(&jnt_vel_interface_);
  }
  
  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.01);}
  
  void read(){
    ROS_INFO_STREAM("Commands for joints: " << cmd_[0] << ", " << cmd_[1]);
  }

  void write(){
    //Update pos_ and vel_
  }

private:
  hardware_interface::JointStateInterface    jnt_state_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  double cmd_[2];
  double pos_[2];
  double vel_[2];
  double eff_[2];
};
```

SampleRobotHWのコンストラクタではJointStateInterfaceとVelocityJointInterfaceを使うための初期設定が記述されています。

cmd_[0]とcmd_[1]にはそれぞれright_wheel_hingeとleft_wheel_hingeへの速度の指令値がコントローラ(ここではDiffDriveController)によって更新されており、read関数ではこれを元に実機の車輪を駆動します。

また、実機からのセンサデータを元にwrite関数でpos_とvel_を更新することでJointStateInterfaceを介してコントローラに伝達されます。

DiffDriveControllerはこの情報からnav_msgs::Odometryメッセージのパブリッシュとtfの発行を行います。

main文は以下のようになります。

**sample_robot_driver.cpp**

```cpp
int main(int argc, char **argv){
  ros::init(argc, argv, "sample_robot");
  ros::NodeHandle nh;
    
  SampleRobotHW robot;
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok()){
    robot.read();
    cm.update(robot.getTime(), robot.getPeriod());
    robot.write();
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
```

## RobotHWSim

実機の時はRobotHW, シミュレータの時はRobotHWSimを継承したクラスをそれぞれ定義して使用します。

RobotHWSimはgazeboへはプラグインとして読み込ませる必要があります。

また、シミュレータ用には標準実装のDefaultRobotHWSimが用意されていて振る舞いを拡張する必要がない場合はこちらを使用します。

継承関係を以下の図に示します。

![robot_hw_sim]({{BASE_PATH}}/images/ros_control/robot_hw_sim.png)

**sample_robot_hw_sim.cpp**

```cpp
class SampleRobotHWSim : public gazebo_ros_control::RobotHWSim
{
private:
  static const double max_drive_joint_torque_ = 20.0;
  double cmd_[2];
  double pos_[2];
  double vel_[2];
  double eff_[2];
        
  gazebo::physics::JointPtr joint_[2];
      
  hardware_interface::JointStateInterface js_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;

public:
  bool initSim(const std::string& robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model,
      const urdf::Model* const urdf_model, std::vector<transmission_interface::TransmissionInfo> transmissions)
  {
    pos_[0] = 0.0; pos_[1] = 0.0;
    vel_[0] = 0.0; vel_[1] = 0.0;
    eff_[0] = 0.0; eff_[1] = 0.0;
    cmd_[0] = 0.0; cmd_[1] = 0.0;

    joint_[0] = parent_model->GetJoint("right_wheel_hinge");
    joint_[1] = parent_model->GetJoint("left_wheel_hinge");
            
    js_interface_.registerHandle(
            hardware_interface::JointStateHandle("right_wheel_hinge", &pos_[0], &vel_[0], &eff_[0]));
            
    js_interface_.registerHandle(
            hardware_interface::JointStateHandle("left_wheel_hinge", &pos_[1], &vel_[1], &eff_[1]));
            
    vj_interface_.registerHandle(
            hardware_interface::JointHandle(js_interface_.getHandle("right_wheel_hinge"), &cmd_[0]));
            
    vj_interface_.registerHandle(
            hardware_interface::JointHandle(js_interface_.getHandle("left_wheel_hinge"), &cmd_[1]));

    registerInterface(&js_interface_);
    registerInterface(&vj_interface_);
            
    return true;
  }

  void readSim(ros::Time time, ros::Duration period){
    for(int i=0; i < 2; i++){
        pos_[i] += angles::shortest_angular_distance(pos_[i], joint_[i]->GetAngle(0).Radian());
        vel_[i] = joint_[i]->GetVelocity(0);
        eff_[i] = joint_[i]->GetForce((unsigned int)(0));
    }
    //Add Custom Code
  }

  void writeSim(ros::Time time, ros::Duration period){
    for(int i=0; i < 2; i++){
        joint_[i]->SetVelocity(0, cmd_[i]);
        joint_[i]->SetMaxForce(0, max_drive_joint_torque_);
    }
    //Add Custom Code
  }
};

typedef boost::shared_ptr<SampleRobotHWSim> SampleRobotSimPtr;

PLUGINLIB_EXPORT_CLASS(SampleRobotHWSim, gazebo_ros_control::RobotHWSim)
```

RobotHWSim::initSim, RobotHWSim::readSim, RobotHWSim::writeSimは純粋仮想関数として定義されているので派生クラスでオーバーライドする必要があります。

SampleRobotHWSimはpluginlibを使用することによりプラグインとして生成します。

**sample_robot_hw_sim_plugins.xml**

```xml
<library path="lib/libsample_robot_hw_sim">
  <class
    name="SampleRobotHWSim"
    type="SampleRobotHWSim"
    base_class_type="gazebo_ros_control::RobotHWSim">
    <description>
        TODO
    </description>
  </class>
</library>
```

**package.xml**

```xml
  <export>
    <gazebo_ros_control plugin="${prefix}/sample_robot_hw_sim_plugins.xml"/>
  </export>
</package>
```

生成したSampleRobotHWSimを読み込むにはsample_robot.xacroに以下のコードを追加します。

**sample_robot.xacro**

```xml
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
  <robotSimType>SampleRobotHWSim</robotSimType>
  </plugin>
</gazebo>
```

sample_robotのモデルをgazeboに読み込ませます。

**sample_robot_gazebo.launch**

```xml
<node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
          args="-urdf -model sample_robot -param robot_description"/>
```

ros_controlを使うことで実機とシミュレータの互換性をシステムを簡便に構築することができます。

また、各要素はプラグインとして入れ替えが可能なので、可搬性を保ちつつそれぞれのロボットのハードウェアに合わせた処理を実装することもできます。

