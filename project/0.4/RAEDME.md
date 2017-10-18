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
 ___知识点补充___： 
 5.
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