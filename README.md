# AIM_LEARNING BY Qmian

## ArmorDetectors

#### InferenceAPI

- int FindIndexOfMaxNumber(const float* ptr, const int len)
  
  功能：找到最大值的索引
  
  步骤：先通过max_element()找到最大元素，再减去容器的开始的差值即得索引

- inline cv::Mat ScaledResize(const cv::Mat& img, Eigen::Matrix<float, 3, 3>& transformMatrix)
  
  功能：调整图像尺寸
  
  步骤：计算图像比例，根据输入图像的宽高进行缩放和填充，并奖padding和缩放比例放入转换矩阵**transformMatrix**中
  
  说明：没有直接用opencv的缩放是要保持图像纵横比，避免失真

- void GenerateGridsFromStride(const int width, const int height,  
  
                               const std::vector<int>& strides, std::vector<GridAndStride>& grids)
  
  功能：获得grid cell
  
  步骤：根据strides计算长宽上grid cell的个数，然后奖grid的宽高索引g0,g1以及stride放入grids中

- void GenerateYoloXProposals(const std::vector<GridAndStride>& grids, const float* feat_ptr,  const Eigen::Matrix<float, 3, 3>& transform_matrix, const float prob_threshold,  std::vector<ArmorObject>& objects)
  
  功能：生成候选框并存储候选对象
  
  步骤：根据生成的grid-cell检测内的类别，过滤掉低置信度的目标。并将角点转至原图像并储存

- float IntersectionArea(const ArmorObject& a, const ArmorObject& b)
  
  功能：获得两检测目标外接矩形的交集

- void qsort_descent_inplace(std::vector<ArmorObject>& face_objects, int left, int right)
  
  功能：根据置信度对检测目标进行快速排序
  
  说明：用到了递归实现

- void nms_sorted_bboxes(std::vector<ArmorObject>& face_objects, std::vector<int>& picked, float nms_threshold)
  
  功能：非极大值抑制，过滤掉重复的框
  
  步骤：首先计算每个候选框的面积，然后遍历所有候选框，通过计算交并比来判断是否合并或保留候选框。如果两个候选框的交并比大于阈值，则将它们合并；否则，保留当前候选框。

- void DecodeOutputs(const float* prob, std::vector<ArmorObject>& objects,  
  
                     const Eigen::Matrix<float, 3, 3>& transform_matrix)
  
  功能：联合前面的函数，生成grid-cell，生成候选框，根据置信度排序，非极大值抑制进而得到最终的检测结果

- float GetAreaOfTetragon(cv::Point2f points[4])
  
  功能：将角点连成的区域分成两个三角形，用海伦公式计算其面积

- bool OpenVINOArmorDetector::Detect(const cv::Mat& src, std::vector<ArmorObject>& objects)
  
  功能：运用openvino进行装甲板检测推理
  
  步骤：将图像转为32位浮点数，拆分通道，输入张量，进行推理解码。
  
  若目标点集大于等于8（执行了非极大值抑制的合并），那就将其取平均。得到最终检测目标的角点

#### NumberClassifier.cpp

- bool AffineNumber(const cv::Mat& frame, const std::vector<cv::Point2f>& corners, cv::Mat& numberROI)
  
  功能：对装甲板进行仿射变换以便于数字识别
  
  步骤：根据比率和宽高计算包围角点的最大矩形，获得矫正点，并确保其在图像范围内。然后将矫正点**以左上角为原点**得到坐标，以标准矩形为目标进行仿射变换，最后，将变换后的区域调整为固定大小（32x32）

- cv::Mat AutoGammaCorrect(const cv::Mat& image)
  
  功能：自动进行gamma矫正
  
  说明：主要判断图像是否为高对比度（转为HSV，计算V的均值和方差）而利用不同的值计算gamma值（指数/对数），若不是高对比度，要进行自动对比度调整。

- int NumberClassifier::Predict(const cv::Mat& frame, const std::vector<cv::Point2f>& corners)
  
  功能：用HOG算子提取特征，用SVM进行预测装甲板数字
  
  说明：用到了多个描述算子，可以看作多个滤波结果，更全面。然后把多个组成一个行向量，丢进svm预测器

### ROIAccelerator

使用ROI加速推理

## PoseSolver.cpp

分大小装甲板进行解算，若未解算成功，输出点坐标和世界坐标信息

**说明：PoseSolver.hpp可修改相机内参**

## NormalEKF.cpp

对线性运动的扩展卡尔曼滤波器

- NormalEKF::NormalEKF()
  
  初始化卡尔曼滤波器的标志 `is_kalman_init` 为 `false`
  
  初始化后验位置，速度
  
  设置过程噪声矩阵

- void NormalEKF::rebootKalman(const Eigen::Vector3d &new_armor_pose)
  
  重启EKF滤波
  
  当前装甲板的位置作为位置后验，速度初始化为0
  
  重置误差协方差矩阵为单位矩阵
  
  重置状态转移矩阵为单位阵

- void NormalEKF::setUpdateTime(const double &delta_t)
  
  设置更新时间（不小于8ms）

- void NormalEKF::setTransitionMatrix()
  
  设置状态转移矩阵为3个**线性运动模型**

- Eigen::Vector3d NormalEKF::runKalman(const Eigen::Vector3d &new_armor_pose, const double &delta_t) & Eigen::Vector3d NormalEKF::correct(const Eigen::Vector3d &armor_pose)
  
  - 如果卡尔曼滤波器未初始化，则初始化卡尔曼滤波器并返回新装甲位置
  
  - 否则，设置更新时间，更新状态转移矩阵，并调用 `correct` 函数进行状态更新
    
    - 设置过程噪声和测量噪声
    
    - 测量装甲位置并转换为pyd（pitch, yaw, distance）
    
    - 如果更新时间超过0.3秒，则重置卡尔曼滤波器
    
    - 调用 `predict` 函数进行预测
    
    - 进行检验，如果检验失败则重置卡尔曼滤波器
    
    - 更新卡尔曼滤波器的状态
    
    - 更新后验位置和速度并返回

- void NormalEKF::setMeasurementNoise(const Eigen::Vector3d &armor_pose) & 
  
  设置yaw,pitch,distance测量噪声。其中distance测量噪声与PNP距离有关

- void NormalEKF::setProcessNoise()
  
  设置过程噪声（关注矩阵相乘过程）

## ExtendedKalman.hpp

### EKF原理

EKF的核心思想是在每个时间步对非线性方程进行泰勒展开，取其雅可比矩阵（Jacobian matrix）进行线性化处理。

#### 预测步骤

1. **状态预测**：利用系统的非线性状态方程预测下一时刻的状态
   
   $\hat{x}_{k|k-1} = f(\hat{x}_{k-1|k-1}, u_k)$

2. **误差协方差预测**：预测估计值的不确定性
   
   $P_{k|k-1} = F_{k-1} P_{k-1|k-1} F_{k-1}^T + Q_{k-1}$

### 更新步骤

1. **计算雅可比矩阵**：对状态方程和观测方程进行线性化。
   
   $F_k = \frac{\partial f}{\partial x} \bigg|_{\hat{x}_{k|k-1}, u_k}$
   
   $H_k = \frac{\partial h}{\partial x} \bigg|_{\hat{x}_{k|k-1}}$

2. **观测预测**：预测当前时刻的观测值
   
   $\hat{z}_k = h(\hat{x}_{k|k-1})$

3. **计算卡尔曼增益**：确定预测状态值和观测值的权重
   
   $K_k = P_{k|k-1} H_k^T (H_k P_{k|k-1} H_k^T + R_k)^{-1}$

4. **状态更新**：根据观测值更新状态估计
   
   $\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k (z_k - \hat{z}_k)$

5. **误差协方差更新**：更新估计误差的协方差
   
   $P_{k|k} = (I - K_k H_k) P_{k|k-1}$

## VehicleTracking.cpp

1. VehicleTracking::VehicleTracking()
   
   初始化一个9*4的EKF滤波器和观测矩阵

2. Eigen::Vector3d VehicleTracking::predictVehicleState(const Sophus::SE3<double> &armor_pose, const Sophus::SE3<double> &armor_pose_sec, bool is_get_second_armor, const int &detect_mode, float shoot_time, float frame_delta_t, float yaw_gimbal)
   
   功能：预测装甲板
   
   流程：
   
   - 辅瞄模式下先判断有没有第二块装甲板，若有则判断和预测位置的差，差小的则放入预测器更新
   
   - 若最小的差过大，则“处理装甲板跳变”
   
   - 若装甲板半径小于0.2，则置为0.2

3. void VehicleTracking::getVehicleState(Eigen::Vector4d &measure)
   
   功能：由测量获得车状态
   
   说明：vehicle={x中心,x速度,y中心,y速度,z中心,z速度,yaw,yaw转速,r半径}

4. Eigen::Vector3d VehicleTracking::getArmorPositionFromState(Eigen::Matrix<double, 9, 1> x)
   
   功能：由车的状态获得装甲板位置
   
   说明：返回armor={x中心,y中心,z中心(zcar)}

5. void VehicleTracking::rebootKalman(const bool is_get_sec, Eigen::Vector4d armor_1, Eigen::Vector4d armor_2)
   
   功能：重启卡尔曼滤波
   
   说明：一块就根据那个装甲板重启，两块则选距离近的

6. void VehicleTracking::handleArmorJump(Eigen::Vector4d &measure)
   
   功能：处理装甲板跳变
   
   步骤：yaw变换过大，推断位置和实际位置差别过大。用测量位置代替装甲板状态并更新滤波器

7. Eigen::Vector3d VehicleTracking::getPredictPoint(const Eigen::Matrix<double, 9, 1>& state, const float &shootTime, const float yawAngle)
   
   功能：获取预测点，预测在给定的射击时间、yaw和装甲板状态的情况下，子弹应该击中的目标点（根据预测yaw和当前yaw的偏差，排序并返回armor_to_hit)
   
   说明：**射击延时可调constexpr auto shoot_delay = 0.10f;**

8. void VehicleTracking::setUpdateTime(const double &delta_t)
   
   时间差至少为5ms

9. Eigen::Vector4d VehicleTracking::PoseToMeasurement(const Sophus::SE3<double> &pose)
   
   功能：解算出的数值到x,y,z&yaw
   
   说明：svd奇异值分解

10. void VehicleTracking::setTransitionMatrix() const
    
    功能：设置状态转移矩阵

11. void VehicleTracking::setObservationMatrix() const
    
    功能：设置观测矩阵

12. void VehicleTracking::setQandRMatrix() const
    
    功能：设置噪声矩阵
    
    **说明：噪声值或许可改**

## Predictor.cpp

预测器

- Predictor::Predictor()
  
  功能：初始化类，包含了上一时刻的状态向量{0,0,0}

- PredictedAngleType Predictor::Predict
  
  步骤：
  
  - 现根据当前云台pitch和目标点根据弹道方程大致解算出射击时间
  
  - 预测时间是当前帧时间+射击时间+射击延时
  
  - 根据装甲板和上一帧的距离判断选用哪个装甲板进行滤波
  
  - 更新上一帧的装甲板位姿
  
  - 根据预测时间去预测射击点prediction1
  
  - 根据VehicleTracking类中的预测，预测整车，得到prediction2
  
  - return no_predict ? shoot_now : (UseEKF ? prediction1 : prediction2);
    
    - 是否用预测-是否用EKF整车预测

- PredictedAngleType Predictor::ballistic_equation(float gim_pitch, const Eigen::Vector3d& armor_Position)
  
  功能：根据物理方程来计算设定pitch和yaw
  
  步骤：
  
  - 先计算yaw,distance和pitch
  
  - 根据弹道方程进行pitch修正
  
  - 返回最终的shootAngleTime_
    
    - pitch
    
    - 射击时间
    
    - yaw

- float Predictor::BulletModel(float x, float v, float angle)
  
  弹道模型
  
  - x 为子弹的初始位置（水平距离）。
  
  - v 为子弹的初速度。
  
  - θ 为子弹发射角度（与水平方向的夹角）。
  
  - kwind​ 为风速系数。
  
  - t 为子弹到达位置 x 所需的时间。
  
  - y 为子弹在垂直方向上的位移。

首先，风速影响的系数 kwind​ 被定义为 0.0001。

接下来，计算子弹到达位置 x 所需的时间 t：$t=kwind​⋅v⋅cos(θ)exp(kwind​⋅x)−1​$

然后，计算子弹在垂直方向上的位移 y： $y=v⋅sin(θ)⋅t−21​⋅GRAVITY⋅t2$

将 t 的表达式代入 y 的计算中，可以得到发射到达的垂直高度 $y=v⋅sin(θ)⋅(kwind​⋅v⋅cos(θ)exp(kwind​⋅x)−1​)−\frac{1}{2}⋅GRAVITY⋅(kwind​⋅v⋅cos(θ)exp(kwind​⋅x)−1​)^{2}$

- float Predictor::calShootTime(const Eigen::Vector3d& armor_trans, float gimbal_pitch)
  
  根据所给的三维坐标直接计算打击时间

## Gimbals.hpp

实现一个云台控制系统的通信和数据处理，包括数据定义、消息分发、任务调度和监控

- **GimbalControlData**：定义了云台控制数据的结构，包括头部标志、速度、云台角度、开火代码和尾部标志

- **GimbalMessageDispatcher** 类：用于管理和分发云台数据和比赛数据

- **EasyGimbalCommunicationTask** 模板：定义了一个通信任务模板，用于与云台进行通信

- **EasyGimbalCommunicationTaskDispatcherPoolsAndMonitors** 结构体：包含用于通信任务的池和监控器

- **GimbalCheatsheet** 结构体：包含辅喵的通信任务、工具和提供者

## PNPAimResult.hpp

从pnp解算的位置到相对云台的yaw&pitch

## 可改参数

- PoseSolver.hpp中相机内参

- Predictor.cpp
  
  可改shootdelay射击延迟

- PNPAimResult.hpp
  
  从PNP的相对位置计算出 yaw pitch
  
  可改相机相对云台的偏置：z_offset，x_offset

- VehicleTracking.cpp
  各种测量噪声
