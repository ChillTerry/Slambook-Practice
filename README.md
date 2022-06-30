# Slambook-Practice
Specifically using for recording my study steps of visual slam. 记录我自己《视觉SLAM14讲》的学习过程

# ch2 2022/6/22

深入了解了一下cmake，对CMakeLists.txt的编写有了更好的理解，以前做智能车用ROS的时候只知道去用`add_excutable`,`target_link_libaries`,并不知道静态库，共享库的含义

![讯飞智能车](https://user-images.githubusercontent.com/64240681/176087403-f4675e18-f2bd-43e0-9143-8d69530c66d1.jpg)


# ch3 2022/6/25

* 学习了旋转矩阵，旋转向量，欧拉角，四元数，欧拉变换

* 学习了Eigen库，并使用Eigen库对上面矩阵、向量进行相互转换
  * 旋转矩阵 Eigen::Matrix3d    `rotation_matrix = Eigen::Matrix3d::Identity()     // 初始化旋转矩阵为单位矩阵` 
  * 旋转向量 Eigen::AngleAxisd  `ratation_vector (M_PI/4, Eigen::Vector3d(0,0,1))  // 沿Z轴旋转45°` 
  * 欧拉角   Eigen::Vector3d    `eular_angle = rotation_matrix.eularAngles(2,1,0)  // 将旋转矩阵直接转换成欧拉角,yaw-pitch-roll顺序` 
  * 四元数   Eigen::Quaternion  `q = Eigen::Quaternion(rotation_vector)            // 可以直接把AngleAxis赋值给四元数，也可以把旋转矩阵赋给它` 
  * 欧拉变换 Eigen::Isometry    `Twr = Eigen::Isometry::Identity()                 // 虽然称为3d，实质上是4＊4的矩阵` 
                                `Twr.rotate(rotation_vector)                       // 按照rotation_vector进行旋转`                           
                                `Twr.pretranslate(Eigen::Vector3d(1,3,4))          // 把平移向量设成(1,3,4)`
                               
* 知道了ifstream读取txt文件用法  `ifstream fin(trajectory_file)` 
                                 `fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw`
                              
* 学会了vector的用法

BTW,不要在虚拟机里面make的时候用**多线程编译**，会变得不幸orz

![M3(}@I@URPA3 V{A7(8AQDG](https://user-images.githubusercontent.com/64240681/175765179-404a2b2c-d7a1-437b-b707-47ef742f653d.png)

# ch4 2022/6/28

上科大毕业典礼期间不让访问生进校，然后这几天在公寓疯狂追凡人修仙传，修仙真爽！顺便学完了第四章的李群李代数，算是对SO(3),SE(3)有了初步了解，这次是边看高博士视频边学的，感觉还不错，有人带确实会好一点，会讲一些书上没有的。指数映射，扰动模型推起来有种很爽的感觉,不知道学数学的会不会像修仙一样，就是那种全部推完和前面融汇贯通之后，有种念头通达，经脉疏通的感觉hhh。但是课后题的那两个证明我没推出来，唉，还是太菜了。这次因为是看视频所以在纸上写的笔记，边看边写，看看情况后面能不能都用笔写，毕竟手写一遍比打字会记得更牢固一点？
![IMG_20220628_112250](https://user-images.githubusercontent.com/64240681/176085353-0172be17-4938-4016-a009-e7c43eb21ea3.jpg)

![IMG_20220628_112300](https://user-images.githubusercontent.com/64240681/176085376-0450d8a8-b3d7-45f7-881c-ded9b50d2c69.jpg)

![IMG_20220628_112155](https://user-images.githubusercontent.com/64240681/176085366-f16e4315-f12a-4d7d-80c2-65e5ea460f33.jpg)

Q: 扰动模型为什么是这么更新的？
![](png/扰动模型更新.png)

# ch5 2022/6/30
C++string类型的拼接确实比python麻烦许多，python可以直接两个string之间用+连接，而C++调用了boost库，mark一下这种写法的格式。

>boost::format fmt("./%s/%d.%s"); //图像文件格式  
>colorImgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));  

eigen里面这.head<>又是什么神仙用法？
>Vector6d p;  
p.head<3>() = pointWorld;

两种李群的赋值方法，对比一下
>Sophus::SE3d SE3_Rt(R,t);

>Sophus::SE3d pose(Eigen::Quaterniond(data[6], data[3], data[4], data[5]),Eigen::Vector3d(data[0], data[1], data[2]));  
poses.push_back(pose);  
Sophus::SE3d T = poses[i];

这两种指针用法等价，首选红框的，十分优美。
![](png/ch5-4.png)

for auto用法如下：
>拷贝range的元素时，使用for(auto x : range).  
修改range的元素时，使用for(auto && x : range).  
只读range的元素时，使用for(const auto & x : range).

Q: 为什么要用Eigen::aligned_allocator？  
>typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;  

**Answer**:我们一般情况下定义容器的元素都是C++中的类型，所以可以省略，这是因为在C++11标准中，aligned_allocator管理C++中的各种数据类型的内存方法是一样的，可以不需要着重写出来。但是在Eigen管理内存和C++11中的方法是不一样的，所以需要单独强调元素的内存分配和管理。 

Q: 这个96怎么来的？为什么最后显示SGBM视差图的时候还要/96？详见ch5 stereoVision.cpp
>if(disparity.at<float>(v,u) <= 0 || disparity.at<float>(v,u) >= 96.0) //96怎么来的？  
cv::imshow("disparity", disparity/96.0);

Q: 归一化坐标与像素坐标系的区别，归一化后的坐标不应该是在像素坐标系里面吗？像素坐标系原点在左上角，那么所有点应该没有负的才对，可是ch5 undistortImage.cpp里面归一化后的坐标有负数。详细见ch5 undistortImage.cpp和书P59 公式（5.5）公式（5.9）
>// 内参  
double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;  
>
>double x = (u-cx)/fx, y = (v-cy)/fy;   //（x,y）归一化后的坐标

Q: 这两种链接方法有什么不同，为什么注释的那一种就编译报错？
![](png/Q2.png)
这两个又有什么区别？为什么前面一种会报错？
>target_link_libraries(stereoVision ${{Pangolin_LIBS})  
>target_link_libraries(stereoVision ${Pangolin_LIBRARIES})  

双目摄像头的视差图
![](png/ch5-2.png)

双目摄像头将归一化后坐标与深度结合得到的点云图
![](png/ch5-1.png)

RGB-D将RGB彩图与深度图结合得到的点云图
![](png/ch5-3.png)








 





