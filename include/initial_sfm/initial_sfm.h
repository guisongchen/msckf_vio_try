#ifndef MSCKF_VIO_INITIAL_SFM_H
#define MSCKF_VIO_INITIAL_SFM_H

#include "integration_base.h"
#include <map>
#include <list>
#include <queue>
#include <opencv2/calib3d.hpp>
#include <memory>
#include <mutex>

#define WINDOW_SIZE 10
#define SIZE_POSE 7
#define SIZE_SPEEDBIAS 9

namespace initial_sfm
{
    
class CovisibleFeature
{
public:
    CovisibleFeature(const Vector4d &_point)
    {
        point_.x() = _point(0);
        point_.y() = _point(1);
        pointRight_.x() = _point(2);
        pointRight_.y() = _point(3);
        
        point3d_.x() = _point(0);
        point3d_.y() = _point(1);
        point3d_.z() = 1.0;
    }

    Vector2d  point_;
    Vector2d  pointRight_;
    Vector3d  point3d_;

};
    
class Feature
{
public:
    Feature(int featureId, int startFrameId)
        : featureId_(featureId), startFrameId_(startFrameId),
          covisibleNum_(0), estimatedDepth_(-1.0) {}

    const int featureId_;
    int       startFrameId_;
    std::vector<CovisibleFeature> covisibleFeatures_;

    int       covisibleNum_;
    double    estimatedDepth_;

};
    
class ImageFrame
{
  
public:
    ImageFrame() {};
    ImageFrame(const std::map<int, Vector4d>  &image,
               double header) : header_ {header}, isKeyframe_ {false}
    {
        image_ = image;
    };
    
    std::map<int, Vector4d> image_;
    double      header_;
    Matrix3d    rotation_;
    Vector3d    translation_;
    IntegrationBase* preIntegration_;
    bool        isKeyframe_;
    
};

struct IMU_MSG
{
    double                  header_;
    Vector3d                linearAcc_;
    Vector3d                angularVel_;
};

typedef std::shared_ptr<initial_sfm::IMU_MSG> ImuPtr;
    
struct IMG_MSG
{
    double                  header_;
    std::map<int, Vector4d> features_;
};

typedef std::shared_ptr<initial_sfm::IMG_MSG> ImgPtr;

class InitSFM
{
public:
    
    InitSFM(Matrix3d R[], Vector3d t[]);
    ~InitSFM() {}
    
    void pubImuData(ImuPtr imu);
    void pubImageData(ImgPtr image);
    
    bool initResult();
    
    Vector3d gyrscopeBias() { return bg_; }
    Vector3d gravity() { return g; }
    
private:
    
    void estimate();
    void processIMU(ImuPtr imuMsg);
    void addFeature(const std::map<int, Vector4d> &image);
    void processImage(ImgPtr imageMsg);
    void setInitResult(bool flag);
    void slidingWindow();
    void removeOldFeatures();
    
    std::vector<std::pair<std::vector<ImuPtr>, ImgPtr>> getSyncMeasurements();
    
    void initFramePoseByPnP();
    void triangulate();
    bool solvePoseByPnP(Matrix3d &R, Vector3d &P, std::vector<cv::Point2f> &pts2d, std::vector<cv::Point3f> &pts3d);
    void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                          Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);
    
    void solveGyroscopeBias();
    bool linearAligment();
    
    std::map<double, ImageFrame> allFramesInWindow_;
    std::list<Feature>  trackedFeatures_;
    
    Matrix3d            ric[2];
    Vector3d            tic[2];
    
    bool                firstImuFlag_;
    double              lastImuTime_;
    double              lastProcessTime_;
    Vector3d            lastAcc_, lastGyr_;
    IntegrationBase*    preIntegrations_[WINDOW_SIZE + 1];
    IntegrationBase*    tmpPreIntegration_;
    int                 frameId_;

    std::queue<ImgPtr>  imageBuf_;
    std::queue<ImuPtr>  imuBuf_;
    double              headersInWindow_[WINDOW_SIZE + 1];
    
    // parameters in sliding window under body frame
    Vector3d            bg_;
    Vector3d            g;
    Vector3d            Ps[WINDOW_SIZE + 1];
    Vector3d            Vs[WINDOW_SIZE + 1];
    Matrix3d            Rs[WINDOW_SIZE + 1];
    Vector3d            Bas[WINDOW_SIZE + 1];
    Vector3d            Bgs[WINDOW_SIZE + 1];
    
    bool                initSuccFlag_;
    std::mutex          mutexBuf_;
    std::mutex          mutexEstimate_;
    std::mutex          mutexResult_;

};
    
}

#endif
