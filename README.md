# Slambook-Practice
Specifically using for recording my study steps of visual slam. 记录我自己《视觉SLAM14讲》的学习过程

# ch2 2022/6/22

以前做智能车用ROS的时候只知道去用`add_excutable`,`target_link_libaries`，这次深入了解了一下cmake，对CMakeLists.txt的编写有了更好的理解

![讯飞智能车](https://user-images.githubusercontent.com/64240681/176087403-f4675e18-f2bd-43e0-9143-8d69530c66d1.jpg)


# ch3 2022/6/25

* 学习了旋转矩阵，旋转向量，欧拉角，四元数，欧拉变换

* 学习了Eigen库，并使用Eigen库对上面矩阵、向量进行相互转换  
  * 旋转矩阵 Eigen::Matrix3d        
  `rotation_matrix = Eigen::Matrix3d::Identity()     // 初始化旋转矩阵为单位矩阵`     
  * 旋转向量 Eigen::AngleAxisd      
  `ratation_vector (M_PI/4, Eigen::Vector3d(0,0,1))  // 沿Z轴旋转45°`     
  * 欧拉角   Eigen::Vector3d        
  `eular_angle = rotation_matrix.eularAngles(2,1,0)  // 将旋转矩阵直接转换成欧拉角,yaw-pitch-roll顺序`   
  * 四元数   Eigen::Quaternion      
  `q = Eigen::Quaternion(rotation_vector)            // 可以直接把AngleAxis赋值给四元数，也可以把旋转矩阵赋给它`   
  * 欧拉变换 Eigen::Isometry        
  `Twr = Eigen::Isometry::Identity()                 // 虽然称为3d，实质上是4＊4的矩阵`   
  `Twr.rotate(rotation_vector)                       // 按照rotation_vector进行旋转`                             
  `Twr.pretranslate(Eigen::Vector3d(1,3,4))          // 把平移向量设成(1,3,4)`  
                              
BTW,不要在虚拟机里面make的时候用**多线程编译**，会变得不幸orz

![M3(}@I@URPA3 V{A7(8AQDG](https://user-images.githubusercontent.com/64240681/175765179-404a2b2c-d7a1-437b-b707-47ef742f653d.png)

# ch4 2022/6/28

上科大毕业典礼期间不让访问生进校，然后这几天在公寓疯狂追凡人修仙传，修仙真爽！顺便学完了第四章的李群李代数，算是对SO(3),SE(3)有了初步了解，这次是边看高博士视频边学的，感觉还不错，有人带确实会好一点，会讲一些书上没有的。指数映射，扰动模型推起来有种很爽的感觉，不知道学数学的会不会像修仙一样，就是那种全部推完和前面融汇贯通之后，有种念头通达，经脉疏通的感觉hhh。这次因为是看视频所以在纸上写的笔记，边看边写，看看情况后面能不能都用笔写，毕竟手写一遍比打字会记得更牢固一点？

![IMG_20220628_112250](https://user-images.githubusercontent.com/64240681/176085353-0172be17-4938-4016-a009-e7c43eb21ea3.jpg)

![IMG_20220628_112300](https://user-images.githubusercontent.com/64240681/176085376-0450d8a8-b3d7-45f7-881c-ded9b50d2c69.jpg)

![IMG_20220628_112155](https://user-images.githubusercontent.com/64240681/176085366-f16e4315-f12a-4d7d-80c2-65e5ea460f33.jpg)

Q: 扰动模型为什么是这么更新的？
![扰动模型更新](https://user-images.githubusercontent.com/64240681/176814857-ca364c2c-84cc-4720-9bbd-335664e2967c.png)

# ch5 2022/6/30

![IMG_20220704_102952](https://user-images.githubusercontent.com/64240681/177071301-54eff28d-d12a-4f2c-b8dd-896437102973.jpg)


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
![ch5-4](https://user-images.githubusercontent.com/64240681/176814937-2d3cbebf-b284-4ec9-88ee-8896e50c261c.png)

for auto用法如下：
>拷贝range的元素时，使用for(auto x : range).  
修改range的元素时，使用for(auto && x : range).  
只读range的元素时，使用for(const auto & x : range).

Q: 为什么要用Eigen::aligned_allocator？  
>typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> TrajectoryType;  

**Answer**: 我们一般情况下定义容器的元素都是C++中的类型，所以可以省略，这是因为在C++11标准中，aligned_allocator管理C++中的各种数据类型的内存方法是一样的，可以不需要着重写出来。但是在Eigen管理内存和C++11中的方法是不一样的，所以需要单独强调元素的内存分配和管理。 

Q: 这个96怎么来的？为什么最后显示SGBM视差图的时候还要/96？详见ch5 stereoVision.cpp
>if(disparity.at<float>(v,u) <= 0 || disparity.at<float>(v,u) >= 96.0) //96怎么来的？  
cv::imshow("disparity", disparity/96.0);

Q: 归一化坐标与像素坐标系的区别，归一化后的坐标不应该是在像素坐标系里面吗？像素坐标系原点在左上角，那么所有点应该没有负的才对，可是ch5 undistortImage.cpp里面归一化后的坐标有负数。详细见ch5 undistortImage.cpp和书P59 公式（5.5）公式（5.9）
>// 内参  
double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;  
>
>double x = (u-cx)/fx, y = (v-cy)/fy;   //（x,y）归一化后的坐标

Q: 这两种链接方法有什么不同，为什么注释的那一种就编译报错？
![Q2](https://user-images.githubusercontent.com/64240681/176815012-ac4748eb-2959-4e1e-bf37-7b9804d6d254.png)  

这两个又有什么区别？为什么前面一种会报错？
>target_link_libraries(stereoVision ${{Pangolin_LIBS})  
>target_link_libraries(stereoVision ${Pangolin_LIBRARIES})  

双目摄像头的视差图
![ch5-2](https://user-images.githubusercontent.com/64240681/176815050-c40cf012-ca2f-4ee0-a07a-aad790ec3323.png)

双目摄像头将归一化后坐标与深度结合得到的点云图
![ch5-1](https://user-images.githubusercontent.com/64240681/176815063-46295108-185f-47eb-8498-d6693192a5f7.png)

RGB-D将RGB彩图与深度图结合得到的点云图
![ch5-3](https://user-images.githubusercontent.com/64240681/176815074-a5525eed-97be-47da-a3b1-d44101f88312.png)

# ch6 2022/7/4

![IMG_20220704_104818](https://user-images.githubusercontent.com/64240681/177073066-1840e0b2-9aa3-4932-89e2-7d1c3b9d185e.jpg)

![IMG_20220704_104842](https://user-images.githubusercontent.com/64240681/177073084-e3aaeb83-1426-40de-8366-978431781769.jpg)


Ceres算法大致流程：

```c++
// 代价函数的计算模型
struct CURVE_FITTING_COST {
  CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}

  // 残差的计算
  template<typename T>
  bool operator()(
    const T *const abc, // 模型参数，有3维
    T *residual) const {
    residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]); // y-exp(ax^2+bx+c)
    return true;
  }

  const double _x, _y;    // x,y数据
};
 
  // 构建最小二乘问题
  ceres::Problem problem;
  for (int i = 0; i < N; i++) {
    problem.AddResidualBlock(     // 向问题中添加误差项
      // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(x_data[i], y_data[i])
      ),
      nullptr,            // 核函数，这里不使用，为空
      abc                 // 待估计参数
    );
  }
 
  // 配置求解器
  ceres::Solver::Options options;     // 这里有很多配置项可以填
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  // 增量方程如何求解
  options.minimizer_progress_to_stdout = true;   // 输出到cout
  ceres::Solver::Summary summary;                // 优化信息
  ceres::Solve(options, &problem, &summary);  // 开始优化
```

G2O算法大致流程：

```c++
typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,1> > Block;  // 每个误差项优化变量维度为3，误差值维度为1

// 第1步：创建一个线性求解器LinearSolver
Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); 

// 第2步：创建BlockSolver。并用上面定义的线性求解器初始化
Block* solver_ptr = new Block( linearSolver );      

// 第3步：创建总求解器solver。并从GN, LM, DogLeg 中选一个，再用上述块求解器BlockSolver初始化
g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );

// 第4步：创建终极大boss 稀疏优化器（SparseOptimizer）
g2o::SparseOptimizer optimizer;     // 图模型
optimizer.setAlgorithm( solver );   // 设置求解器
optimizer.setVerbose( true );       // 打开调试输出

// 第5步：定义图的顶点和边。并添加到SparseOptimizer中
CurveFittingVertex* v = new CurveFittingVertex(); //往图中增加顶点
v->setEstimate( Eigen::Vector3d(0,0,0) );
v->setId(0);
optimizer.addVertex( v );
for ( int i=0; i<N; i++ )    // 往图中增加边
{
  CurveFittingEdge* edge = new CurveFittingEdge( x_data[i] );
  edge->setId(i);
  edge->setVertex( 0, v );                // 设置连接的顶点
  edge->setMeasurement( y_data[i] );      // 观测数值
  edge->setInformation( Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma) ); // 信息矩阵：协方差矩阵之逆
  optimizer.addEdge( edge );
}

// 第6步：设置优化参数，开始执行优化
optimizer.initializeOptimization();
optimizer.optimize(100); 
```

使用G2O时候报错了 

> g2oCurveFitting.cpp:(.text.startup+0x546)：对‘g2o::OptimizationAlgorithmGaussNewton::OptimizationAlgorithmGaussNewton(g2o::Solver*)’未定义的引用

我猜测有两种可能

- CmakeLists那边没有编译找到OptimizationAlgorithmGaussNewton这个函数的头文件?
- 这个函数语法写错?  

但是网上找了一圈没有发现正确的解决方案，所以暂时搁置了  

# ch7 2022/7/12

![IMG_20220712_201534](https://user-images.githubusercontent.com/64240681/178487697-8d14129e-c9d1-4faa-955d-569dac2fc129.jpg)

![IMG_20220712_201631](https://user-images.githubusercontent.com/64240681/178487753-b2db1598-a857-4834-8537-5ba63ba69514.jpg)

![IMG_20220712_201610](https://user-images.githubusercontent.com/64240681/178487782-45a96821-dba9-4c91-8c20-dd1474e5648b.jpg)


特征点匹配的时候，图片1中的keypoint1可能会和图片2中的多个keypoint2对应，那么match.size应该是大于descriptor_1.rows的？
```c++
  //-- 第四步:匹配点对筛选
  double min_dist = 10000, max_dist = 0;

  //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
  for (int i = 0; i < descriptors_1.rows; i++) {
    double dist = match[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist);
  printf("-- Min dist : %f \n", min_dist);

  //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
  for (int i = 0; i < descriptors_1.rows; i++) {
    if (match[i].distance <= max(2 * min_dist, 30.0)) {
      matches.push_back(match[i]);
    }
  }
```   

三角测距的时候，代码里的T1和T2是什么？为什么和书里的计算公式不一样？
```c++
  Mat T1 = (Mat_<float>(3, 4) <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0);
  Mat T2 = (Mat_<float>(3, 4) <<
    R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
  );
```

F不是由E再加上内参K得出的吗？那为什么计算E要带上相机光心，相机光心，而计算F不用？
```c++
    //-- 计算基础矩阵 F
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat(points1, points2, CV_FM_8POINT);
    cout << "fundamental_matrix is " << endl << fundamental_matrix << endl;

    //-- 计算本质矩阵 E
    Point2d principal_point(325.1, 249.7);  //相机光心, TUM dataset标定值
    double focal_length = 521;      //相机光心, TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
    cout << "essential_matrix is " << endl << essential_matrix << endl;
```

d1, d2为什么要 / 5000.0? 最后push_back的时候为什么x, y要 * dd1, dd2?
```c++
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat Img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "一共找到了" << matches.size() << "组匹配点" << endl;

    Mat depth_1 = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);
    Mat depth_2 = imread(argv[4], CV_LOAD_IMAGE_UNCHANGED);
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    vector<Point3f> pts1, pts2;

    for (DMatch m:matches)
    {
        ushort d1 = depth_1.ptr<double>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
        ushort d2 = depth_2.ptr<double>(int(keypoints_2[m.trainIdx].pt.y))[int(keypoints_2[m.trainIdx].pt.x)];
        if (d1 == 0 || d2 == 0) continue;
        float dd1 = float(d1) / 5000.0;
        float dd2 = float(d2) / 5000.0;

        Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        Point2d p2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);

        pts1.push_back(Point3f(p1.x * dd1, p1.y * dd1, dd1));
        pts2.push_back(Point3f(p2.x * dd2, p2.y * dd2, dd2));
    }
```

# ch8 2022/7/19

![IMG_20220719_111211](https://user-images.githubusercontent.com/64240681/179678455-4b851f1b-18f9-4395-b154-bead2a55bec4.jpg)


不管是光流法还是直接法都有一步是做内插，为什么？
```c++
 /* get a gray scale value from reference image (bi-linear interpolated)*/
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    // boundary check
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= img.cols - 1) x = img.cols - 2;
    if (y >= img.rows - 1) y = img.rows - 2;
    
    float xx = x - floor(x);
    float yy = y - floor(y);
    int x_a1 = std::min(img.cols - 1, int(x) + 1);
    int y_a1 = std::min(img.rows - 1, int(y) + 1);
    
    return (1 - xx) * (1 - yy) * img.at<uchar>(y, x)
    + xx * (1 - yy) * img.at<uchar>(y, x_a1)
    + (1 - xx) * yy * img.at<uchar>(y_a1, x)
    + xx * yy * img.at<uchar>(y_a1, x_a1);
}

opencv并行运算。
‵‵`c++
cv::parallel_for_(cv::Range(0, px_ref.size()),
                  std::bind(&JacobianAccumulator::accumulate_jacobian, &jaco_accu, std::placeholders::_1));
‵‵‵






 





