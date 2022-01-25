码头 A
471.253，321.876，-12.068

码头 B

停放码头

rosparam list

待提高：
基于局部坐标系建立，近似水面在同一高度，实际的航线用经纬度来表示

ros 使用 python3

欧拉角转换
https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html
https://stackoverflow.com/questions/53033620/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr?rq=1

键盘控制
https://github.com/osrf/vrx/wiki/driving_teleop_tutorial
$roslaunch vrx_gazebo usv_keymodelstate.launch

  <!-- Spawn model in Gazebo warship -->
  <!-- warship -->
  <arg name="x_warship" default="-540" />
  <arg name="y_warship" default="162" />
  <arg name="warship_urdf" default="$(find wamv_gazebo)/urdf/warship/war_ship_gray.xacro" />

  <param name="warship_description" 
         command="$(find xacro)/xacro &#x002D;&#x002D;inorder $(arg warship_urdf)
         namespace:=warship" />
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" 
        args="-x $(arg x_warship) -y $(arg y_warship) -z $(arg z)
              -R $(arg R) -P $(arg P) -Y $(arg Y)
              -urdf -param warship_description -model warship" />

macro语法
https://blog.csdn.net/sunbibei/article/details/52297524