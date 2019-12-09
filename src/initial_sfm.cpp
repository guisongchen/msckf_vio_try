// ideas form Vins-Mono
// Modifier: Guisong Chen   guisongchen@163.com

#include "initial_sfm/initial_sfm.h"
#include "msckf_vio/math_utils.hpp"

#include <opencv2/core/eigen.hpp>
#include <iostream>

namespace initial_sfm
{

InitSFM::InitSFM(Matrix3d R[], Vector3d t[])
{
    for (int i = 0; i < 2; i++)
    {
        ric[i] = R[i];
        tic[i] = t[i];
    }
    
    for (size_t i = 0; i < WINDOW_SIZE + 1; i++)
    {
        preIntegrations_[i] = nullptr;
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
    }
    
    for(auto &it: allFramesInWindow_)
        it.second.preIntegration_ = nullptr;
    
    tmpPreIntegration_ = nullptr;
    
    frameId_ = 0;
    firstImuFlag_ = true;
    initSuccFlag_ = false;
    lastImuTime_ = -1.0;
    lastProcessTime_ = -1.0;
}

void InitSFM::estimate()
{
    mutexEstimate_.lock();
    
    if (!initResult())
    {
        std::vector<std::pair<std::vector<ImuPtr>, ImgPtr>> measurements =getSyncMeasurements();
        
        if (measurements.size() > 0)
        {
            for (auto &measurement : measurements)
            {
                auto imageMsg = measurement.second;
                
                for (auto &imuMsg : measurement.first)
                    processIMU(imuMsg);

                processImage(imageMsg);
                
            }
        }
    }
    
    mutexEstimate_.unlock();
}

std::vector<std::pair<std::vector<ImuPtr>, ImgPtr>> InitSFM::getSyncMeasurements()
{
    std::vector<std::pair<std::vector<ImuPtr>, ImgPtr>> measurements;

    while (true)
    {
        if (imuBuf_.empty() || imageBuf_.empty())
            return measurements;
        
        if (!(imuBuf_.back()->header_ > imageBuf_.front()->header_))
        {
            std::cerr << "wait for imu, only should happen at the beginning" << std::endl;
            return measurements;
        }

        if (!(imuBuf_.front()->header_ < imageBuf_.front()->header_))
        {
            std::cerr << "throw img, only should happen at the beginning" << std::endl;
            imageBuf_.pop();
            continue;
        }
        
        ImgPtr imgMsg = imageBuf_.front();
        imageBuf_.pop();

        std::vector<ImuPtr> IMUs;
        
        while (imuBuf_.front()->header_ <= imgMsg->header_)
        {
            IMUs.emplace_back(imuBuf_.front());
            imuBuf_.pop();
        }
        
        if (IMUs.empty())
            std::cerr << "no imu between two image" << std::endl;
        
        measurements.emplace_back(IMUs, imgMsg);
    }
    
    return measurements;
}


void InitSFM::pubImageData(ImgPtr image)
{
    mutexBuf_.lock();
    imageBuf_.push(image);
    mutexBuf_.unlock();
    
    estimate();
}

void InitSFM::pubImuData(ImuPtr imu_data)
{
    mutexBuf_.lock();
    imuBuf_.push(imu_data);
    mutexBuf_.unlock();
    
    estimate();
}

bool InitSFM::initResult() 
{
    std::unique_lock<std::mutex> lock(mutexResult_);
    return initSuccFlag_; 
}

void InitSFM::setInitResult(bool flag) 
{
    std::unique_lock<std::mutex> lock(mutexResult_);
    initSuccFlag_ = flag; 
}
    
void InitSFM::processIMU(ImuPtr imuMsg)
{
    if (firstImuFlag_)
    {
        firstImuFlag_ = false;
        lastAcc_ = imuMsg->linearAcc_;
        lastGyr_ = imuMsg->angularVel_;
        lastImuTime_ = imuMsg->header_;
    }
    
    Vector3d& linearAcc = imuMsg->linearAcc_;
    Vector3d& angularVel = imuMsg->angularVel_;


    if (!preIntegrations_[frameId_])
    {
        preIntegrations_[frameId_] = new IntegrationBase{lastAcc_, lastGyr_, Bas[frameId_], Bgs[frameId_]};
    }
    
    // if frameId == 0, we can't do integration for only one element
    if (frameId_ != 0)
    {

        double dt = imuMsg->header_ - lastImuTime_;

        preIntegrations_[frameId_]->push_back(dt, linearAcc, angularVel);
        tmpPreIntegration_->push_back(dt, linearAcc, angularVel);
        
        int currId = frameId_;
        
        // calculate delatq and transform to Rotation Matrix
        Vector3d predictGyr = 0.5 * (lastGyr_ + angularVel) - Bgs[currId];
        Rs[currId] *= msckf_vio::deltaQ(predictGyr * dt).toRotationMatrix();
        
        Vector3d lastLinearlizedAcc = Rs[currId] * (lastAcc_ - Bas[currId]) - g;
        Vector3d currLinearlizedAcc = Rs[currId] * (linearAcc - Bas[currId]) - g;
        Vector3d predictAcc = 0.5 * (lastLinearlizedAcc + currLinearlizedAcc);
        
        Ps[currId] += dt * Vs[currId] + 0.5 * dt * dt * predictAcc;
        Vs[currId] += dt * predictAcc;
    }
    
    // update measurement using current measurement to calculate next preIntegration
    lastAcc_ = linearAcc;
    lastGyr_ = angularVel;
    lastImuTime_ = imuMsg->header_;
}

void InitSFM::addFeature(const std::map<int, Vector4d> &image)
{
    for (auto &id_pts : image)
    {
        int featureId = id_pts.first;
        
        // use date from camera 0
        Vector4d positionAndPixel = id_pts.second;
        
        CovisibleFeature currCovFeature(positionAndPixel);
        
        // features tracked by different frames have the same featureId
        // check passing in featureId exist or not
        auto it = std::find_if(trackedFeatures_.begin(), trackedFeatures_.end(),
                               [featureId] (const Feature &it) { return it.featureId_ == featureId; });

        // if not exist, added and update
        if (it == trackedFeatures_.end())
        {
            // init Feature(including startFrameId_)
            trackedFeatures_.push_back(Feature(featureId, frameId_));
            trackedFeatures_.back().covisibleFeatures_.push_back(currCovFeature);
        }
        
        // if find match, which means this feature was tracked before, update CovisibleFeature info
        else if (it->featureId_ == featureId)
            it->covisibleFeatures_.push_back(currCovFeature);
    }
}

void InitSFM::processImage(ImgPtr imageMsg)
{

    addFeature(imageMsg->features_);
    
    // use this to map timestamp and frame
    headersInWindow_[frameId_] = imageMsg->header_;

    ImageFrame imageframe(imageMsg->features_, imageMsg->header_);
    imageframe.preIntegration_ = tmpPreIntegration_;
    allFramesInWindow_.insert(std::make_pair(imageMsg->header_, imageframe));
    
    // integration region: last frame ~ current frame
    tmpPreIntegration_ = new IntegrationBase{lastAcc_, lastGyr_, Bas[frameId_], Bgs[frameId_]};
    
    initFramePoseByPnP();
    triangulate();

    // check if the window is full
    if (frameId_ == WINDOW_SIZE)
    {
        std::map<double, ImageFrame>::iterator frame_it;
        int i = 0;
        for (frame_it = allFramesInWindow_.begin(); frame_it != allFramesInWindow_.end(); frame_it++)
        {
            frame_it->second.rotation_ = Rs[i];
            frame_it->second.translation_ = Ps[i];
            i++;
        }
        solveGyroscopeBias();
        
        bool flag = linearAligment();
        setInitResult(flag);
        
        if (flag)
            std::cout << "initSFM successed!" << std::endl;
        else
            slidingWindow();
        
    }
    else if (frameId_ < WINDOW_SIZE)  // window not full, predict next frame pose using current pose
    {
        frameId_++;
        int prev = frameId_ - 1;
        Ps[frameId_] = Ps[prev];
        Rs[frameId_] = Rs[prev];
        Bas[frameId_] = Bas[prev];
        Bgs[frameId_] = Bgs[prev];
    }
}

// match features, find correspond frames 
// 1) features first observed frame
//    -- use 2d normalized point and invdepth recover 3d position
//    -- then transform from first observed frame to current frame <-- pts3d
// 2) features observed by current frame(frameId)
//    -- use 2d normalized point <-- pts2d

// what's we need to prepare:
// 1) features observed by current features
// 2) for each feature, find out the first observed frame
// 3) coordinate on normalized plane and invdepth of each feature
// 4) pose in world frame for each frame in sliding window

void InitSFM::initFramePoseByPnP()
{
    if (frameId_ > 0)
    {
        std::vector<cv::Point2f> pts2d;
        std::vector<cv::Point3f> pts3d;
        
        for (auto &feature : trackedFeatures_)
        {
            if (feature.estimatedDepth_ > 0)
            {
                const int index = frameId_ - feature.startFrameId_;
                
                if (static_cast<int>(feature.covisibleFeatures_.size()) >= index+1)
                {
                    Vector3d ptsInCam = ric[0] * (feature.covisibleFeatures_[0].point3d_*feature.estimatedDepth_) + 
                                        tic[0];
                    Vector3d ptsInWorld = Rs[feature.startFrameId_]*ptsInCam + Ps[feature.startFrameId_];
                    
                    cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
                    cv::Point2f point2d(feature.covisibleFeatures_[index].point_.x(),
                                        feature.covisibleFeatures_[index].point_.y());
                    
                    pts3d.push_back(point3d);
                    pts2d.push_back(point2d);
                }
            }
        }
        
//         std::cout << "pnp points size: " << pts3d.size() << std::endl;
        
        Matrix3d RCam = Rs[frameId_-1] * ric[0];
        Vector3d PCam = Ps[frameId_-1] + Rs[frameId_-1]*tic[0];
        
        if (solvePoseByPnP(RCam, PCam, pts2d, pts3d))
        {
            Rs[frameId_] = RCam * ric[0].transpose();
            Ps[frameId_] = -RCam * ric[0].transpose() * tic[0] + PCam; 
        }
        else
            std::cout << "solve pnp failed" << std::endl;
    }
}

bool InitSFM::solvePoseByPnP(Matrix3d &R, Vector3d &P,
                             std::vector<cv::Point2f> &pts2d, std::vector<cv::Point3f> &pts3d)
{
    // w2c --> c2w
    Matrix3d R_init = R.inverse();
    Vector3d P_init = -(R_init * P);
    
    if (pts2d.size() < 4)
        return false;
    
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_init, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_init, t);
    cv::Mat K = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    
    bool result;
    result = cv::solvePnP(pts3d, pts2d, K, D, rvec, t, 1);
    
    if (!result)
        return false;
    
    cv::Rodrigues(rvec, r);
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);
    
    // c2w --> w2c
    R = R_pnp.transpose();
    P = R * (-T_pnp);
    
    return true;
}

void InitSFM::triangulate()
{
    for (auto &it_per_id : trackedFeatures_)
    {
        if (it_per_id.estimatedDepth_ > 0)
            continue;

        // using imu integration value as initial value
        int imu_i = it_per_id.startFrameId_;
        
        Eigen::Matrix<double, 3, 4> leftPose;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        leftPose.leftCols<3>() = R0.transpose();
        leftPose.rightCols<1>() = -R0.transpose() * t0;

        Eigen::Matrix<double, 3, 4> rightPose;
        Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[1];
        Eigen::Matrix3d R1 = Rs[imu_i] * ric[1];
        rightPose.leftCols<3>() = R1.transpose();
        rightPose.rightCols<1>() = -R1.transpose() * t1;

        Eigen::Vector2d point0, point1;
        Eigen::Vector3d point3d;
        point0 = it_per_id.covisibleFeatures_[0].point_;
        point1 = it_per_id.covisibleFeatures_[0].pointRight_;

        triangulatePoint(leftPose, rightPose, point0, point1, point3d);
        
        Eigen::Vector3d localPoint;
        localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
        double depth = localPoint.z();
        if (depth > 0)
            it_per_id.estimatedDepth_ = depth;
        else
            it_per_id.estimatedDepth_ = 5.0;

    }
}

void InitSFM::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                               Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    
    Eigen::Vector4d triangulated_point;
    triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

void InitSFM::solveGyroscopeBias()
{
    Matrix3d A;
    Vector3d b;
    Vector3d delta_bg;
    A.setZero();
    b.setZero();
    std::map<double, ImageFrame>::iterator frame_i;
    std::map<double, ImageFrame>::iterator frame_j;
    
    for (frame_i = allFramesInWindow_.begin(); next(frame_i) != allFramesInWindow_.end(); frame_i++)
    {
        frame_j = next(frame_i);
        MatrixXd tmp_A(3, 3);
        tmp_A.setZero();
        VectorXd tmp_b(3);
        tmp_b.setZero();
        Eigen::Quaterniond q_ij(frame_i->second.rotation_.transpose() * frame_j->second.rotation_);
        tmp_A = frame_j->second.preIntegration_->jacobian.template block<3, 3>(IntegrationBase::O_R, 
                                                                               IntegrationBase::O_BG);
        tmp_b = 2 * (frame_j->second.preIntegration_->delta_q.inverse() * q_ij).vec();
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;
    }
    
    delta_bg = A.ldlt().solve(b);
    std::cout << "gyroscope bias: " << delta_bg.transpose() << std::endl;
    
    bg_ = delta_bg;

    for (int i = 0; i <= WINDOW_SIZE; i++)
        Bgs[i] += delta_bg;

    for (frame_i = allFramesInWindow_.begin(); next(frame_i) != allFramesInWindow_.end( ); frame_i++)
    {
        frame_j = next(frame_i);
        frame_j->second.preIntegration_->repropagate(Vector3d::Zero(), Bgs[0]);
    }
}

bool InitSFM::linearAligment()
{
    int frameNum = allFramesInWindow_.size();
    
    // (vx,vy,vz) for each frame, g_c0(gx,gy,gz), s 
    int stateVectorNum = frameNum * 3 + 3 + 1;

    MatrixXd A{stateVectorNum, stateVectorNum};
    A.setZero();
    VectorXd b{stateVectorNum};
    b.setZero();
    VectorXd x{stateVectorNum};
    x.setZero();

    std::map<double, ImageFrame>::iterator frame_i;
    std::map<double, ImageFrame>::iterator frame_j;
    int i = 0;
    for (frame_i = allFramesInWindow_.begin(); next(frame_i) != allFramesInWindow_.end(); frame_i++, i++)
    {
        frame_j = next(frame_i);

        MatrixXd tmp_A(6, 10);
        tmp_A.setZero();
        VectorXd tmp_b(6);
        tmp_b.setZero();

        // construct H matrix and b vector
        
        double dt = frame_j->second.preIntegration_->sum_dt;
        tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
        tmp_A.block<3, 3>(0, 6) = frame_i->second.rotation_.transpose() * dt * dt / 2 * Matrix3d::Identity();
        
        // set arbitrary initial scale s = 0.01
        tmp_A.block<3, 1>(0, 9) = frame_i->second.rotation_.transpose() * 
                                 (frame_j->second.translation_ - frame_i->second.translation_) / 100.0;     
        
        tmp_b.block<3, 1>(0, 0) = frame_j->second.preIntegration_->delta_p + frame_i->second.rotation_.transpose() *
                                  frame_j->second.rotation_ * tic[0] - tic[0];

        tmp_A.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
        tmp_A.block<3, 3>(3, 3) = frame_i->second.rotation_.transpose() * frame_j->second.rotation_;
        tmp_A.block<3, 3>(3, 6) = frame_i->second.rotation_.transpose() * dt * Matrix3d::Identity();
        tmp_b.block<3, 1>(3, 0) = frame_j->second.preIntegration_->delta_v;

        // set information matrix as Identity
        Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Identity();

        // H^T * H * x = H^T * b
        MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
        VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

        A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
        b.segment<6>(i * 3) += r_b.head<6>();

        A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
        b.tail<4>() += r_b.tail<4>();

        A.block<6, 4>(i * 3, stateVectorNum - 4) += r_A.topRightCorner<6, 4>();
        A.block<4, 6>(stateVectorNum - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();
    }
    
    // solve x, by Ax = b
    // maybe for numerical reason, both side multiply 1000
    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b);
    
    // divide initial scale we set at first
    double s = x(stateVectorNum - 1) / 100.0;
    std::cout << "estimated scale: " << s << std::endl;
    
    g = x.segment<3>(stateVectorNum - 4);
    std::cout <<"result g: " << g.norm() << " " << g.transpose() << std::endl;
    
    if(fabs(g.norm() - G.norm()) > 1.0 || s < 0)
        return false;
    
    return true;
}

void InitSFM::slidingWindow()
{
    
    double t_0 = headersInWindow_[0];
    
    if (frameId_ == WINDOW_SIZE)
    {
        
        for (int i = 0; i < WINDOW_SIZE; i++)
        {
            std::swap(preIntegrations_[i], preIntegrations_[i + 1]);
            headersInWindow_[i] = headersInWindow_[i + 1];
            
            Rs[i].swap(Rs[i + 1]);
            Ps[i].swap(Ps[i + 1]);
            Vs[i].swap(Vs[i + 1]);
            Bas[i].swap(Bas[i + 1]);
            Bgs[i].swap(Bgs[i + 1]);
        }

        headersInWindow_[WINDOW_SIZE] = headersInWindow_[WINDOW_SIZE - 1];
        Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
        Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
        Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
        Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
        Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

        delete preIntegrations_[WINDOW_SIZE];
        preIntegrations_[WINDOW_SIZE] = new IntegrationBase{lastAcc_, lastGyr_, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};
        
        delete allFramesInWindow_[t_0].preIntegration_;
        allFramesInWindow_[t_0].preIntegration_ = nullptr;
        allFramesInWindow_.erase(t_0);

        removeOldFeatures();
    }
}

void InitSFM::removeOldFeatures()
{
    for (auto it = trackedFeatures_.begin(), it_next = trackedFeatures_.begin() ; 
         it != trackedFeatures_.end(); it = it_next)
    {
        it_next++;
        
        if (it->startFrameId_ != 0)
            it->startFrameId_--;
        else
        {
            it->covisibleFeatures_.erase(it->covisibleFeatures_.begin());
            
            if (it->covisibleFeatures_.empty())
                trackedFeatures_.erase(it);
        }
    }
}
    
}
