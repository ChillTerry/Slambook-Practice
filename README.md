# Slambook-Practice
Specifically using for recording my study steps of visual slam. 记录我自己《视觉SLAM14讲》的学习过程

#ch2 2022/6/22

深入了解了一下cmake，对CMakeLists.txt的编写有了更好的理解，以前做智能车用ROS的时候只知道去用`add_excutable`,`target_link_libaries`,并不知道静态库，共享库的含义
![讯飞智能车](https://user-images.githubusercontent.com/64240681/175765877-ec8823df-98f5-4ab3-bd4e-987d5fd4cd60.jpg)

#ch3 2022/6/25

* 学习了旋转矩阵，旋转向量，欧拉角，四元数，欧拉变换

* 学习了Eigen库，并使用Eigen库对上面矩阵、向量进行相互转换
  * 旋转矩阵 Eigen::Matrix3d    `rotation_matrix = Eigen::Matrix3d::Identity()     // 初始化旋转矩阵为单位矩阵`
  * 旋转向量 Eigen::AngleAxisd  `ratation_vector (M_PI/4, Eigen::Vector3d(0,0,1))  // 沿Z轴旋转45°`
  * 欧拉角   Eigen::Vector3d    `eular_angle = rotation_matrix.eularAngles(2,1,0)  // 将旋转矩阵直接转换成欧拉角,yaw-pitch-roll顺序`
  * 四元数   Eigen::Quaternion  `q = Eigen::Quaternion(rotation_vector)            // 可以直接把AngleAxis赋值给四元数，也可以把旋转矩阵赋给它`
  * 欧拉变换 Eigen::Isometry    `Twr = Eigen::Isometry::Identity()                 // 虽然称为3d，实质上是4＊4的矩阵`
  *                             `Twr.rotate(rotation_vector)                       // 按照rotation_vector进行旋转`
  *                             `Twr.pretranslate(Eigen::Vector3d(1,3,4))          // 把平移向量设成(1,3,4)`
                               
-知道了ifstream读取txt文件用法  `ifstream fin(trajectory_file)`
                               `fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw`;
                               
-学会了vector的用法

BTW,不要在虚拟机里面make的时候用**多线程编译**，会变得不幸orz
![M3(}@I@URPA3 V{A7(8AQDG](https://user-images.githubusercontent.com/64240681/175765179-404a2b2c-d7a1-437b-b707-47ef742f653d.png)

