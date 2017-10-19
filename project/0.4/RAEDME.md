#VO notes：
**东西比较杂，按照代码阅读过程依次记录。**  

## camera类：(相机模型)
```c
class Camera
{
public:
    typedef std::shared_ptr<Camera> Ptr;
    float   fx_, fy_, cx_, cy_, depth_scale_;

    Camera();
    Camera ( float fx, float fy, float cx, float cy, float depth_scale=0 ) :
        fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy ), depth_scale_ ( depth_scale )
    {}

    // coordinate transform: world, camera, pixel
    Vector3d world2camera( const Vector3d& p_w, const SE3& T_c_w );
    Vector3d camera2world( const Vector3d& p_c, const SE3& T_c_w );
    Vector2d camera2pixel( const Vector3d& p_c );
    Vector3d pixel2camera( const Vector2d& p_p, double depth=1 ); 
    Vector3d pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth=1 );
    Vector2d world2pixel ( const Vector3d& p_w, const SE3& T_c_w );

};

```
在整个工程里，所有的坐标（二维/三维）都是用`Vector2d/Vector3d`表示；位姿（pose）都是用`Sophus::SE3`表示。
首先说一说`Sophus::SE3`：

```c
  SE3                        (const SO3 & so3,
                              const Vector3d & translation);
  SE3                        (const Matrix3d & rotation_matrix,
                              const Vector3d & translation);
  SE3                        (const Quaterniond & unit_quaternion,
                              const Vector3d & translation_);
  SE3                        (const SE3 & other);
```
由上面构造函数可以看出，`SE3`大体上由旋转矩阵和平移`SO3/matrix3d`+`vector3d`,还可以用四元素。  
再来看几个常用的操作符:  

* SE3显示输出<< :
```c
inline std::ostream& operator <<(std::ostream & out_str,
                                 const SE3 &  se3)
{
  out_str << se3.so3() << se3.translation().transpose() << std::endl;
  return out_str;
}//先输出他的旋转矩阵，然后输出平移向量的转置
```
* SO3转换成矩阵`Matrix3d` rotation_matrix()
```c
Matrix3d rotation_matrix() const
  {
    return so3_.matrix();
  }
```
* inverse()

其次看一看
`Matrix3d`和`Vector3d`:  

*  初始化
*  转置a.transpose()
*  共轭转置a.adjoint()
*  vector的点乘v.dot()
*  mat.trace()
*  mat(0,0):取值
*  vec(0):取值

```c
Matrix2d a;  
a << 1, 2,3, 4;
Vector3d  v(1,2,3);
MatrixXf a(2,3); a << 1, 2, 3, 4, 5, 6;
cout << "Here is the initial matrix a:\n" << a << endl;
a.transposeInPlace();//a = a.transpose(); // !!! do NOT do this !!!
cout << "and after being transposed:\n" << a << endl;
```

Eigen Transform class:    
```c
VectorNf p1, p2;//Apply the transformation to a point   
p2 = t * p1;

VectorNf vec1, vec2;//Apply the transformation to a vector  
vec2 = t.linear() * vec1;

VectorNf n1, n2;
MatrixNf normalMatrix = t.linear().inverse().transpose();
n2 = (normalMatrix * n1).normalized();
n2 = t.linear() * n1;

glLoadMatrixf(t.data());

Affine3f aux(Affine3f::Identity());
aux.linear().topLeftCorner<2,2>() = t.linear();
aux.translation().start<2>() = t.translation();
glLoadMatrixf(aux.data());

t.matrix() = matN1xN1;    // N1 means N+1
matN1xN1 = t.matrix();

t(i,j) = scalar;   <=>   t.matrix()(i,j) = scalar;
scalar = t(i,j);   <=>   scalar = t.matrix()(i,j);

t.translation() = vecN;
vecN = t.translation();//translation part
vecN(0,0) vecN(1,0)  vecN(2,0) ;

t.linear() = matNxN;
matNxN = t.linear();//linear part   

matNxN = t.rotation();//extract the rotation matrix 


```
camera类构造函数初始化
```c
Camera::Camera()
{
    fx_ = Config::get<float>("camera.fx");
    fy_ = Config::get<float>("camera.fy");
    cx_ = Config::get<float>("camera.cx");
    cy_ = Config::get<float>("camera.cy");
    depth_scale_ = Config::get<float>("camera.depth_scale");
}//用config::get()方法，这是静态方法。在config类里会讲到。
```

## Config类：
### config.h
```c
#ifndef CONFIG_H
#define CONFIG_H

#include "myslam/common_include.h" 

namespace myslam 
{
class Config
{
private:
    static std::shared_ptr<Config> config_;  //私有静态成员变量 config_
    cv::FileStorage file_;//利用opencv  cv::FileStorage
    
    Config () {} // private constructor makes a singleton
                 //私有构造函数使之变成单件模式
public:
    ~Config();  // close the file when deconstructing 
    
    // set a new config file 
    static void setParameterFile( const std::string& filename ); //静态方法
    
    // access the parameter values
    template< typename T >
    static T get( const std::string& key )//静态方法
    {
        return T( Config::config_->file_[key] );
    }
};
}

#endif // CONFIG_H
```
### config.cpp
```c
#include "myslam/config.h"

namespace myslam 
{
    
void Config::setParameterFile( const std::string& filename )
//公有静态方法，静态变量是`config_`，只有这个*内部函数*能实例化`config()`
{
{
    if ( config_ == nullptr )
        config_ = shared_ptr<Config>(new Config);
    //内部调用私有构造函数`Config()`,返回对象“config_”的实例，这个实例也在内部定义好了，即静态成员变量  `private static std::shared_ptr<Config> config_;` 
     //这里体现了什么叫唯一性：如果对象为空，就初始化新的对象；若不为空，即不新建。则始终保持一个对象（只有这个静态方法）  
    config_->file_ = cv::FileStorage( filename.c_str(), cv::FileStorage::READ );
    if ( config_->file_.isOpened() == false )
    {
        std::cerr<<"parameter file "<<filename<<" does not exist."<<std::endl;
        config_->file_.release();
        return;
    }
}

Config::~Config()
{
    if ( file_.isOpened() )
        file_.release();
}

shared_ptr<Config> Config::config_ = nullptr;

}
```
### 讲一讲: 

1. 单件模式：静态工作方法；私有构造函数；私有静态成员变量。
```c
static T get( const std::string& key )//静态方法
    {
        return T( Config::config_->file_[key] );
    }
```
2. 公有静态方法（static），可以当成函数直接使用。调用一个被定义为static的方法，可以通过在它前面加上这个类的名称，也可以像调用非静态方法一样通过类对象调用。  
例如：`fx_ = Config::get<float>("camera.fx");`//直接调用Config::get()

3. cv::FileStorage  
XML/YAML/JSON file storage class that encapsulates all the information necessary for writing or reading data to/from a file.

    config_->file_ = cv::FileStorage( filename.c_str(), cv::FileStorage::READ );  
    config_->file_.isOpened()  
    config_->file_.release()
## Frame类：
```c
KeyPoint(Point2f _pt, float _size, float _angle=-1, float _response=0, int _octave=0, int _class_id=-1);
    /**
    @param _pt x & y coordinates of the keypoint
    @param _size keypoint diameter
    @param _angle keypoint orientation
    @param _response keypoint detector response on the keypoint (that is, strength of the keypoint)
    @param _octave pyramid octave in which the keypoint has been detected
    @param _class_id object id
     */
```  

Frame类：帧定义了ID/时间戳/位姿/相机模型/图像这几个量；方法：创建frame/寻找深度/获取相机光心/判断某个点是否在视野内等等。

## VisualOdometry  

### VisualOdometry.h
```c
#include "myslam/common_include.h"
#include "myslam/map.h"

#include <opencv2/features2d/features2d.hpp>

namespace myslam 
{
class VisualOdometry
{
public:
    typedef shared_ptr<VisualOdometry> Ptr;
    enum VOState {
        INITIALIZING=-1,
        OK=0,
        LOST
    };
    
    VOState     state_;     // current VO status
    Map::Ptr    map_;       // map with all frames and map points
    
    Frame::Ptr  ref_;       // reference key-frame 
    Frame::Ptr  curr_;      // current frame 

    //中间变量定义在内部，方便访问。
    cv::Ptr<cv::ORB> orb_;  // orb detector and computer 
    vector<cv::KeyPoint>    keypoints_curr_;    // keypoints in current frame
    Mat                     descriptors_curr_;  // descriptor in current frame 
    
    cv::FlannBasedMatcher   matcher_flann_;     // flann matcher
    vector<MapPoint::Ptr>   match_3dpts_;       // matched 3d points 
    vector<int>             match_2dkp_index_;  // matched 2d pixels (index of kp_curr)
   
    SE3 T_c_w_estimated_;    // the estimated pose of current frame 
    int num_inliers_;        // number of inlier features in icp
    int num_lost_;           // number of lost times
    
    // parameters 
    int num_of_features_;   // number of features
    double scale_factor_;   // scale in image pyramid
    int level_pyramid_;     // number of pyramid levels
    float match_ratio_;     // ratio for selecting  good matches
    int max_num_lost_;      // max number of continuous lost times
    int min_inliers_;       // minimum inliers
    double key_frame_min_rot;   // minimal rotation of two key-frames
    double key_frame_min_trans; // minimal translation of two key-frames
    double  map_point_erase_ratio_; // remove map point ratio
    
public: // functions 
    VisualOdometry();
    ~VisualOdometry();
    
    bool addFrame( Frame::Ptr frame );      // add a new frame 
    
protected:  
    // inner operation 
    void extractKeyPoints();
    void computeDescriptors(); 
    void featureMatching();
    void poseEstimationPnP(); 
    void optimizeMap();
    
    void addKeyFrame();
    void addMapPoints();
    bool checkEstimatedPose(); 
    bool checkKeyFrame();
    
    double getViewAngle( Frame::Ptr frame, MapPoint::Ptr point );
    
};
}
```

### VisualOdometry.cpp
*  构造函数初始化
```c
//构造函数初始化
VisualOdometry::VisualOdometry() :
    state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ), map_ ( new Map ), num_lost_ ( 0 ), num_inliers_ ( 0 ), matcher_flann_ ( new cv::flann::LshIndexParams ( 5,10,2 ) )
{
    num_of_features_    = Config::get<int> ( "number_of_features" );
    scale_factor_       = Config::get<double> ( "scale_factor" );
    level_pyramid_      = Config::get<int> ( "level_pyramid" );
    match_ratio_        = Config::get<float> ( "match_ratio" );
    max_num_lost_       = Config::get<float> ( "max_num_lost" );
    min_inliers_        = Config::get<int> ( "min_inliers" );
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
    map_point_erase_ratio_ = Config::get<double> ( "map_point_erase_ratio" );
    orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
```
* 析构函数
```c
VisualOdometry::~VisualOdometry()
{

}
```
* 方法之VisualOdometry::addFrame()
```c
bool VisualOdometry::addFrame ( Frame::Ptr frame )
{
    switch ( state_ )
    {
    case INITIALIZING://初始化时状态默认OK,当前帧与参考帧都设为第一个frame
    {
        state_ = OK;
        curr_ = ref_ = frame;
        // 从这个frame里提取关键点，计算特征点，然后加入keyframe.
        extractKeyPoints();//1
        computeDescriptors();//2
        addKeyFrame();      // the first frame is a key-frame
        break;
    }
    case OK://不是第一帧时
    {
        curr_ = frame;//更新当前帧
        curr_->T_c_w_ = ref_->T_c_w_;//更新位姿
        extractKeyPoints();
        computeDescriptors();提取特征点
        featureMatching();//这个比较0.2版本更新较多
        poseEstimationPnP();
        if ( checkEstimatedPose() == true ) // a good estimation
        {
            curr_->T_c_w_ = T_c_w_estimated_;
            optimizeMap();
            num_lost_ = 0;
            if ( checkKeyFrame() == true ) // is a key-frame
            {
                addKeyFrame();
            }
        }
        else // bad estimation due to various reasons
        {
            num_lost_++;
            if ( num_lost_ > max_num_lost_ )
            {
                state_ = LOST;
            }
            return false;
        }
        break;
    }
    case LOST:
    {
        cout<<"vo has lost."<<endl;
        break;
    }
    }

    return true;
}

```
* 方法之extractKeyPoints()和computeDescriptors()  
中间变量keypoints_curr_，descriptors_curr_都在类的pubilc里定义了，方便调用。
```c
void VisualOdometry::extractKeyPoints()
{
    orb_->detect ( curr_->color_, keypoints_curr_ );
}
void VisualOdometry::computeDescriptors()
{
    orb_->compute ( curr_->color_, keypoints_curr_, descriptors_curr_ );
}
```
* ***方法之featureMatching()***  

***特征匹配与两两帧结构不同，不是当前帧与上一帧匹配，而是当前帧与地图中的点去匹配***  
步骤：  
 1. 从地图中选取候选点`<MapPoint::Ptr> candidate`，但是要对这些点进行筛选  
 2. 判断某个点是否在视野内/是不是在当前帧中 `isInFrame()`  
 3. 是，添加到候选点中`candidate.push_back( p )`，加入临时地图中`Mat desp_map`；且计数（出现的次数）`p->visible_times_++`。  
 4. `matcher_flann_`匹配。前一参数`desp_map`是地图中候选点的描述子(第一帧的时候也是加入了的，也就是说，第一次匹配就是当前帧与第一张图片（已默认全部添加到地图中）的描述子匹配)  
 5. 匹配完成求出最佳匹配值，乘以一个系数与30，来判断这个match的好坏，至此，匹配完成。  
 6. 匹配后，将记录匹配对，这个对不是2d-2d，而是记录map点的3d坐标match_3dpts_.push_back( candidate[m.queryIdx] )与下一帧的2d图像坐标`match_2dkp_index_.push_back( m.trainIdx )`，这个3维坐标也就是前一帧的图像坐标在空间中的投影。  

 ___知识点补充：___  

* `DMatch`类

```c
 class CV_EXPORTS_W_SIMPLE DMatch
{
public:
    CV_WRAP DMatch();
    CV_WRAP DMatch(int _queryIdx, int _trainIdx, float _distance);
    CV_WRAP DMatch(int _queryIdx, int _trainIdx, int _imgIdx, float _distance);

    CV_PROP_RW int queryIdx; // query descriptor index
    CV_PROP_RW int trainIdx; // train descriptor index
    CV_PROP_RW int imgIdx;   // train image index

    CV_PROP_RW float distance;//distance between descriptors.

    // less is better
    bool operator<(const DMatch &m) const;
};
```
* STL里的`std::min_element`  
`ForwardIterator min_element (ForwardIterator first, ForwardIterator last,
                               Compare comp);`  
```c
//代码中如何取得最小
float min_dis = std::min_element 
(
                        matches.begin(), matches.end(),
                        //自定义比较函数
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
                            {return m1.distance < m2.distance;}//比较函数返回bool类型
)->distance;//返回最小值
```

```c
void VisualOdometry::featureMatching()
{
    boost::timer timer;
    vector<cv::DMatch> matches;
    // select the candidates in map 
    //从地图中选取候选点，但是要对这些点进行筛选
    Mat desp_map;//全部放候选点描述子，为了match
    vector<MapPoint::Ptr> candidate;
    for ( auto& allpoints: map_->map_points_ )//对地图类中的变量之一map_points_遍历
        //unordered_map<unsigned long, MapPoint::Ptr >  map_points_; 
    {
        MapPoint::Ptr& p = allpoints.second;//unordered_map<unsigned long, MapPoint::Ptr >  map_points_;MapPoint类中有descriptor_
        // 看看该点是不是在当前帧中
        if ( curr_->isInFrame(p->pos_) )
        {
            // add to candidate 
            p->visible_times_++;
            candidate.push_back( p );
            desp_map.push_back( p->descriptor_ );//
        }
    }
    
    matcher_flann_.match ( desp_map, descriptors_curr_, matches );
    // select the best matches
    float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;//返回最小值
//这两个是关键！！！每次都要清零，每一帧都要与地图中的点匹配。所以新的帧来了就要先清零。
//首先要知道PNP是怎么回事，一般的PNP是指上一帧的特征点投影到空间（地图）中的3维点，与下一帧图像对于匹配的特征点之间的最小化重投影误差。
//OK!在此过程中，上一帧，特征点匹配变成了（地图中的点（3d）<--->当前帧的特征点（2d)做最小化重投影误差)
    match_3dpts_.clear();
    match_2dkp_index_.clear();
    for ( cv::DMatch& m : matches )
    {
        if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            match_3dpts_.push_back( candidate[m.queryIdx] );
            match_2dkp_index_.push_back( m.trainIdx );
        }
    }
    cout<<"good matches: "<<match_3dpts_.size() <<endl;
    cout<<"match cost time: "<<timer.elapsed() <<endl;
}
```
* 方法之poseEstimationPnP()    
  1. 位置估计PNP
  2. g2o优化
```c
void VisualOdometry::poseEstimationPnP()
{
    // construct the 3d 2d observations
    vector<cv::Point3f> pts3d;//地图点坐标
    vector<cv::Point2f> pts2d;//像素点坐标
//转换格式，从`<KeyPoint>`格式转换成`<vector>`，为cv::solvePnPRansac(）传参做准备，其实就是一个复制过程
    for ( int index:match_2dkp_index_ )
    //vector<int>  match_2dkp_index_;  // matched 2d pixels (index of kp_curr)
    {
        pts2d.push_back ( keypoints_curr_[index].pt );
        //vector<cv::KeyPoint>    keypoints_curr_;    // 当前帧的关键点
    }
    for ( MapPoint::Ptr pt:match_3dpts_ )//vector<MapPoint::Ptr>   match_3dpts_;       // matched 3d points 
    {
        pts3d.push_back( pt->getPositionCV() );
        //inline cv::Point3f getPositionCV() const {
        // return cv::Point3f( pos_(0,0), pos_(1,0), pos_(2,0) );
        // }
    }

    Mat K = ( cv::Mat_<double> ( 3,3 ) <<
              ref_->camera_->fx_, 0, ref_->camera_->cx_,
              0, ref_->camera_->fy_, ref_->camera_->cy_,
              0,0,1
            );
    Mat rvec, tvec, inliers;
    cv::solvePnPRansac ( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
    num_inliers_ = inliers.rows;
    cout<<"pnp inliers: "<<num_inliers_<<endl;
    T_c_w_estimated_ = SE3 (
                           SO3 ( rvec.at<double> ( 0,0 ), rvec.at<double> ( 1,0 ), rvec.at<double> ( 2,0 ) ),
                           Vector3d ( tvec.at<double> ( 0,0 ), tvec.at<double> ( 1,0 ), tvec.at<double> ( 2,0 ) )
                       );

    //g2o
    // using bundle adjustment to optimize the pose
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solver_ptr = new Block ( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
        T_c_w_estimated_.rotation_matrix(), T_c_w_estimated_.translation()
    ));//估计的目标对象
    optimizer.addVertex ( pose );//添加顶点

    // edges
    for ( int i=0; i<inliers.rows; i++ )
    {
        int index = inliers.at<int> ( i,0 );
        // 3D -> 2D projection
        //一元边
        EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
        edge->setId ( i );
        edge->setVertex ( 0, pose );
        edge->camera_ = curr_->camera_.get();
        edge->point_ = Vector3d ( pts3d[index].x, pts3d[index].y, pts3d[index].z );
        edge->setMeasurement ( Vector2d ( pts2d[index].x, pts2d[index].y ) );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        // set the inlier map points 
        match_3dpts_[index]->matched_times_++;//匹配次数计数
    }

    optimizer.initializeOptimization();
    optimizer.optimize ( 10 );

    T_c_w_estimated_ = SE3 (
        pose->estimate().rotation(),
        pose->estimate().translation()
    );
    
    cout<<"T_c_w_estimated_: "<<endl<<T_c_w_estimated_.matrix()<<endl;
}
```
* 方法之checkEstimatedPose()和checkKeyFrame()
  1. 内点数量要求  
  2. 运动是否过于激烈   
  3. 关键帧不能取太相似

***知识补充：***
```c
//指数映射  对数映射
static SE3
  exp                        (const Vector6d & update);

  static Vector6d
  log                        (const SE3 & se3);
```

```c
bool VisualOdometry::checkEstimatedPose()
{
    // check if the estimated pose is good
    if ( num_inliers_ < min_inliers_ )
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    //T_c_w_estimated_已经在PNP后赋值
    //ref_->T_c_w_，参考系的世界到相机转换；
    //T_c_w_estimated_：上一帧到当前帧的T。他的逆就是c->r的转换。
    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();//对数映射
    if ( d.norm() > 5.0 )
    {
        cout<<"reject because motion is too large: "<<d.norm() <<endl;
        return false;
    }
    return true;
}

bool VisualOdometry::checkKeyFrame()
{
    //ref_->T_c_w_，参考系的世界到相机转换；
    //T_c_w_estimated_：上一帧到当前帧的T。他的逆就是c->r的转换。
    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();//T的对数映射---->6维向量
    Vector3d trans = d.head<3>(); //获取向量的前n个元素：vector.head(n);   
    Vector3d rot = d.tail<3>();//获取向量尾部的n个元素：vector.tail(n);
    if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
        return true;
    return false;
}
```
* 方法之addKeyFrame()

```c
void VisualOdometry::addKeyFrame()
{
    if ( map_->keyframes_.empty() )
    {
        // first key-frame, add all 3d points into map
        for ( size_t i=0; i<keypoints_curr_.size(); i++ )
        {
            double d = curr_->findDepth ( keypoints_curr_[i] );
            if ( d < 0 ) 
                continue;
            Vector3d p_world = ref_->camera_->pixel2world (
                Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ), curr_->T_c_w_, d
            );
            Vector3d n = p_world - ref_->getCamCenter();
            n.normalize();//归一化
            MapPoint::Ptr map_point = MapPoint::createMapPoint(
                p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
            );//生成mappoint
            map_->insertMapPoint( map_point );//地图里添加map_point
        }
    }
    
    map_->insertKeyFrame ( curr_ );
    ref_ = curr_;
}
```

## MapPoint类
### mappoint.cpp
```c
namespace myslam
{
//默认构造函数
MapPoint::MapPoint()
: id_(-1), pos_(Vector3d(0,0,0)), norm_(Vector3d(0,0,0)), good_(true), visible_times_(0), matched_times_(0)
{

}
//构造函数
MapPoint::MapPoint ( long unsigned int id, const Vector3d& position, const Vector3d& norm, Frame* frame, const Mat& descriptor )
: id_(id), pos_(position), norm_(norm), good_(true), visible_times_(1), matched_times_(1), descriptor_(descriptor)
{
    observed_frames_.push_back(frame);
}
//默认方法
MapPoint::Ptr MapPoint::createMapPoint()
{
    return MapPoint::Ptr( 
        new MapPoint( factory_id_++, Vector3d(0,0,0), Vector3d(0,0,0) )
    );
}
//方法
MapPoint::Ptr MapPoint::createMapPoint ( 
    const Vector3d& pos_world, 
    const Vector3d& norm, 
    const Mat& descriptor, 
    Frame* frame )
{
    return MapPoint::Ptr( 
        new MapPoint( factory_id_++, pos_world, norm, frame, descriptor )//调用构造函数
    );
}

unsigned long MapPoint::factory_id_ = 0;

}
```
## Map类
### map.cpp
* insertKeyFrame
* insertMapPoint  

没有就添加  
```c
namespace myslam
{

void Map::insertKeyFrame ( Frame::Ptr frame )
{
    cout<<"Key frame size = "<<keyframes_.size()<<endl;
    if ( keyframes_.find(frame->id_) == keyframes_.end() )
    {
        keyframes_.insert( make_pair(frame->id_, frame) );
    }
    else
    {
        keyframes_[ frame->id_ ] = frame;
    }
}

void Map::insertMapPoint ( MapPoint::Ptr map_point )
{
    if ( map_points_.find(map_point->id_) == map_points_.end() )
    {
        map_points_.insert( make_pair(map_point->id_, map_point) );
    }
    else 
    {
        map_points_[map_point->id_] = map_point;
    }
}

}
```
* ***方法之addMapPoints()***

```c
void VisualOdometry::addMapPoints()
{
    // add the new map points into map
    //用一个bool型的vector，大小为当前特征点数，初始化为false.
    vector<bool> matched(keypoints_curr_.size(), false); 
    for ( int index:match_2dkp_index_ )
        matched[index] = true;//特征点中匹配成功的设为true
    //对于不是匹配点的关键点
    for ( int i=0; i<keypoints_curr_.size(); i++ )
    {
        if ( matched[i] == true )   
            continue;
        double d = ref_->findDepth ( keypoints_curr_[i] );
        if ( d<0 )  
            continue;
        Vector3d p_world = ref_->camera_->pixel2world (
            Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ), 
            curr_->T_c_w_, d
        );
        Vector3d n = p_world - ref_->getCamCenter();
        n.normalize();
        MapPoint::Ptr map_point = MapPoint::createMapPoint(
            p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
        );
        map_->insertMapPoint( map_point );
    }
}
```
* 方法之optimizeMap()
```c
void VisualOdometry::optimizeMap()
{
    // remove the hardly seen and no visible points 
    for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end(); )
    {
        if ( !curr_->isInFrame(iter->second->pos_) )
        {
            iter = map_->map_points_.erase(iter);//如果不在当前帧画面中，就剔除掉
            continue;
        }
        float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
        if ( match_ratio < map_point_erase_ratio_ )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        
        double angle = getViewAngle( curr_, iter->second );
        if ( angle > M_PI/6. )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        if ( iter->second->good_ == false )
        {
            // TODO try triangulate this map point 
        }
        iter++;
    }
    //如果匹配点少于100个就调用addmappoint命令
    if ( match_2dkp_index_.size()<100 )
        addMapPoints();
    //如果大于1000，就减少
    if ( map_->map_points_.size() > 1000 )  
    {
        // TODO map is too large, remove some one 
        map_point_erase_ratio_ += 0.05;
    }
    else 
        map_point_erase_ratio_ = 0.1;
    cout<<"map points: "<<map_->map_points_.size()<<endl;
}
```