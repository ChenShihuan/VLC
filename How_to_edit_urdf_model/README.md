修改机器人urdf模型的方法
1.将mvcam.stl相机模型放入turtlebot机器人的传感器模型路径：例如此路径：~./catkin_ws/src/turtlebot3/turtlebot3_description/meshes/sensors/mvcam.stl
2.修改urdf.xacro模型描述文件，路径如下：~./catkin_ws/src/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf.xacro
在文件中仿照前文格式增加以下内容，例子如同本文件夹下的turtlebot3_burger.urdf.xacro所示：

  <joint name="vlc_joint" type="fixed">
    <parent link="base_link"/>
    <child link="base_vlc_mvcam"/>
    <origin xyz="0.060 0 0.155" rpy="0 0 0"/>
  </joint>

  <link name="base_vlc_mvcam">
    <visual>
      <origin xyz="0.0 0 -0.08" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://turtlebot3_description/meshes/sensors/mvcam.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="dark"/>
    </visual>

    <collision>
      <origin xyz="0.015 0 -0.0065" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.0315" radius="0.055"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.114" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0"
               izz="0.001" />
    </inertial>
  </link>

joint和tf变换树相关，描述了变换关系，link则和模型本身有关，描述了模型的文件路径、相对tf变换树所表述的位置和旋转关系、颜色等。概括起来，真正影响到编程中的tf变换关系的是joint部分，link部分完全是为了观感的真实和美观
