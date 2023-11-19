/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "LoopClosing.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"


#include<mutex>
#include<thread>

#include <pcl/octree/octree_search.h>


namespace ORB_SLAM2
{

LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
    mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(false), mNumSubmapKFs(4), mSubmapVoxleSize(0.05)
{
    mnCovisibilityConsistencyTh = 3;
    
    // Set lookahead position
    mlookAhead = cv::Mat::eye(4,4,CV_32F);
    mlookAhead.at<float>(2,3) = (0.1+4) * 0.5; // near distance threshold = 0.1m, far distance threshold = 4m

    mPairCandidateLoopPCD.first = static_cast<KeyFrame*>(nullptr);
    mPairCandidateLoopPCD.second = static_cast<KeyFrame*>(nullptr);

    mpRegistration = hdl_graph_slam::resgistrationNDTOMP();
    
}

void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}


void LoopClosing::Run()
{
    mbFinished =false;

    while(1)
    {
        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // Detect loop candidates and check covisibility consistency
            if(DetectLoop())
            {
               // Compute similarity transformation [sR|t]
               // In the stereo/RGBD case s=1
               if(ComputeSim3())
               {
                   // Perform loop fusion and pose graph optimization
                   CorrectLoop();
               }
            }
            // Detect loop candidates and check covisibility consistency
            else if(DetectLoopNDT())
            {
               // Compute similarity transformation [sR|t]
               // In the stereo/RGBD case s=1
                if(ComputeSim3NDT())
               {
                    // Perform loop fusion and pose graph optimization
                   CorrectLoop();
               }
            }
        }

        ResetIfRequested();

        if(CheckFinish())
            break;

        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }

    SetFinish();
}

void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty());
}

bool LoopClosing::DetectLoop()
{
    {// 키 프레임 추출 및 대기열 관리
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }

    // 초기 조건 검사 - 마지막으로 루프가 감지된 키프레임과의 키프레임 단위 거리가 10 이상이어야 함
    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    if(mpCurrentKF->mnId < mLastLoopKFid+10)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    // BoW 유사도 점수 계산 - 후보 필터링 하는데 사용
    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames(); // 임계값 이상의 공변성 관계를 가진 키프레임만 추출
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue;
        const DBoW2::BowVector &BowVec = pKF->mBowVec;

        float score = mpORBVocabulary->score(CurrentBowVec, BowVec);

        if(score<minScore)
            minScore = score;
    }
    // 루프 후보 탐색 - 최소 BoW 점수 이상인 루프 후보 탐색
    // Query the database imposing the minimum score
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mvConsistentGroups.clear();
        mpCurrentKF->SetErase();
        return false;
    }
    
    // 일관성 검사 - 후보 필터링 하는데 사용
    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    mvpEnoughConsistentCandidates.clear(); // 충분히 일관성 있는 후보들을 저장하는 벡터

    vector<ConsistentGroup> vCurrentConsistentGroups; // 현재 루프 후보들의 일관성 그룹을 저장할 수 있는 벡터
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false); // 이전 일관성 그룹과의 일치여부를 추적하는 불리언 벡터 초기화
   
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        // 루프 후보들을 순회하며 후보와 관련된 키프레임 그룹을 얻음
        KeyFrame* pCandidateKF = vpCandidateKFs[i]; 
        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF); // 자기 자신도 넣는거야?


        // 이전 일관성 그룹들과의 비교 - 현재 후보 그룹(), 이
        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {   
            // 이전에 감지된 일관성 있는 그룹들 중 하나를 가져옴
            set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first; 

            bool bConsistent = false;
            for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit)) // 이전 그룹들 중 현재 후보 그룹과 공유하는 키프레임이 있는지 확인
                {
                    bConsistent=true;
                    bConsistentForSomeGroup=true; // 있다면 일관성이 있다고 표시
                    break;
                }
            }

            if(bConsistent) // 일관성이 있다면
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second; // 이전 일관성 그룹의 일관성 수치를 가져옴
                int nCurrentConsistency = nPreviousConsistency + 1; // 현재 일관성 그룹의 일관성 수치를 1 증가시킴
                if(!vbConsistentGroup[iG]) // 이전 일관성 그룹과의 일치여부를 추적하는 불리언 벡터가 false라면
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency); // 현재 일관성 그룹을 생성 (후보 키프레임, 일관성 수치)
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                if(nCurrentConsistency >= mnCovisibilityConsistencyTh && !bEnoughConsistent) // 현재 일관성 그룹의 일관성 수치가 임계값 이상이고, 충분히 일관성 있는 후보들을 저장하는 벡터에 추가되지 않았다면
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0);
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    mvConsistentGroups = vCurrentConsistentGroups;


    // Add Current Keyframe to database
    mpKeyFrameDB->add(mpCurrentKF);

    if(mvpEnoughConsistentCandidates.empty())
    {
        mpCurrentKF->SetErase();
        return false;
    }
    else
    {
        return true;
    }

    mpCurrentKF->SetErase();
    return false;
}

bool LoopClosing::DetectLoopNDT()
{
    mvpSubMapKFs.clear();
    // Avoid that a keyframe can be erased while it is being process by this thread
    mpCurrentKF->SetNotErase();


    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    if(mpCurrentKF->mnId < mLastLoopKFid+30)
    {
        mpCurrentKF->SetErase();
        return false;
    }
    
    // Compare Look ahead distance between current KF and last loop KF
    // If the distance is less than 1m, return true
    // Look ahead distance is mean value of distance between near distance and far distance threshold
    mPairCandidateLoopPCD.first       = static_cast<KeyFrame*>(nullptr);
    mPairCandidateLoopPCD.second      = static_cast<KeyFrame*>(nullptr);
    mPairCandidateLoopPCDPoint.first  = cv::Mat();
    mPairCandidateLoopPCDPoint.second = cv::Mat();
    // mpMap에 접근해서 모든 키프레임 가져오기
    std::vector<KeyFrame*> keyframes = mpMap->GetAllKeyFrames();
    double minScore = 1.0;
    int minScoreIdx = -1;
    // Get current keyframe's pose
    cv::Mat currentKFPose =  mpCurrentKF->GetPoseInverse() ;
    // Check distance between all keyframes and current keyframe
    for ( int i = 0; i < keyframes.size(); i++ )
    {
        // Check accumulated distance between current keyframe and keyframe
        float accumDistance = mpCurrentKF->GetAccumDistance() - keyframes[i]->GetAccumDistance(); 
        if ( accumDistance < 10.0 ) // Accmulated moving distance[m] may be bigger than 10m.. 2.5 times of 4m(far distance threshold)
        {
            // ROS_INFO_STREAM("accumDistance() : " << accumDistance  );
            continue;
        }

        // check if keyframe has deleted
        if ( !keyframes[i]->mnId || keyframes[i]->isBad() || keyframes[i]->mnId == mpCurrentKF->mnId ) // 키프레임이 삭제되었거나, 현재 키프레임이거나, bad 키프레임이거나, 삭제된 키프레임이면
        {
            continue;
        }

        // Get Cadidate keyframe's pose
        cv::Mat CadidateKFPose = keyframes[i]->GetPoseInverse();
        if (mpCurrentKF->mnId < keyframes[i]->mnId + 25) // this 25 is tested value
        {
            continue;
        }

        // Get lookahead pose of keyframes
        cv::Mat lookaheadPoseSource   = currentKFPose * mlookAhead; // camera view point to world coordinate -> this is z(forward), x(right), y(down) coordinate..
        cv::Mat lookaheadPoseTarget = CadidateKFPose * mlookAhead;

        // Get distance between current keyframe and keyframe
        float distance = norm(lookaheadPoseSource - lookaheadPoseTarget, cv::NORM_L2);
        
        // If the distance is less than 1m, return true
        if ( distance < minScore ) // this value is teseted value
        {
            // check the smallest keyframe
            minScore = distance;
            // save the lookahead pose of candidate keyframe
            mPairCandidateLoopPCDPoint.first  = lookaheadPoseSource;
            mPairCandidateLoopPCDPoint.second = lookaheadPoseTarget;

            // set previous smallest distance keyframe to erased
            if ( minScoreIdx != -1 )
            {
                // release previous candidate should be before saving minscoreIdx
                keyframes[minScoreIdx]->SetNotErase(); 
                // set this keyframe not to erased
                keyframes[minScoreIdx]->SetErase();
            }
            // Save loop closing candiate keyframe
            mpMatchedKFPCD = keyframes[i];
            // save previous smallest distance keyframe index 
            minScoreIdx = i;
        }
    }
    // if minsScore is not changed, return false
    if(minScoreIdx != -1)
    {
        mPairCandidateLoopPCD.first  = mpCurrentKF;
        mPairCandidateLoopPCD.second = mpMatchedKFPCD;
        // Gather near keyframes of mpMatchedKFPCD

                // Get possible front and back number of accessible keyframes.
        int accessibleFront = minScoreIdx - mNumSubmapKFs;
        int accessibleBack = minScoreIdx + mNumSubmapKFs;
        minScoreIdx = mNumSubmapKFs; // normal state
        if( accessibleFront < 0 ) // not enough front keyframes
        {
            minScoreIdx = mNumSubmapKFs + accessibleFront; 
            accessibleFront = 0;
        }
        if( accessibleBack > keyframes.size() ) // not enough back keyframes
        {
            minScoreIdx = accessibleBack - mNumSubmapKFs; 
            accessibleBack = keyframes.size();
        }

        // Get a Submap accessible keyframes which is orientation is similar to the new_keyframe.
        for (int i = accessibleFront; i < accessibleBack; i++)
        {
            mvpSubMapKFs.push_back(keyframes[i]);
        }
        
        cout << "--- Keyframe view-point-based loop detected Submap Size: " << mvpSubMapKFs.size() << " ---" << endl;
        return true;
    }
    mpCurrentKF->SetErase();
    return false;
}



bool LoopClosing::ComputeSim3()
{
    // For each consistent loop candidate we try to compute a Sim3

    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver;
    ORBmatcher matcher(0.75,true);

    vector<Sim3Solver*> vpSim3Solvers;  // 솔버를 가지고 있는 벡터를 생성
    vpSim3Solvers.resize(nInitialCandidates); // 후보 개수만큼 크기를 설정

    vector<vector<MapPoint*> > vvpMapPointMatches;  // 포인트 클라우드(맵 포인트 벡터)를 가지고 있는 벡터를 생성
    vvpMapPointMatches.resize(nInitialCandidates);  // 후보자 만큼 생성

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches

    // 일관성이 보장된 후보들을 순회하며 ORB 매칭을 수행하고, Sim3Solver를 설정함
    for(int i=0; i<nInitialCandidates; i++)
    {
        KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

        // avoid that local mapping erase it while it is being processed in this thread
        pKF->SetNotErase();

        if(pKF->isBad())
        {
            vbDiscarded[i] = true;
            continue;
        }

        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]); // 현재 키프레임과 후보 키프레임 사이의 맵 포인트를 매칭함 (BoW 매칭 사용)
        // vvpMapPointMatches는 후보자와 매칭이 된 삼차원 상의 맵 포인트를 의미

        if(nmatches<20) // 개수가 적으면 버림
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mbFixScale);
            pSolver->SetRansacParameters(0.99,20,300);
            vpSim3Solvers[i] = pSolver; // 솔버 객체 자체를 저장함
        }

        nCandidates++;
    }

    bool bMatch = false;
    
    // 여기서 Sim3를 RANSAC으로 가이드 변환을 추정한 뒤, Optimizer를 사용해서 더 fine한 값을 추정함
    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++) // 후보자만큼 
        {
            if(vbDiscarded[i])
                continue;

            KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            Sim3Solver* pSolver = vpSim3Solvers[i];
            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers); // Sim m -> c (current)->  matched candidate keyframe (pKF) / 이 부분이 RANSAC을 수행하는 부분

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if(!Scm.empty()) // if there is a Scm matrix
            {
                vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                }


                // 이 위의 부분을 Registration Matcing으로 바꾸기
                cv::Mat R = pSolver->GetEstimatedRotation();
                cv::Mat t = pSolver->GetEstimatedTranslation();
                const float s = pSolver->GetEstimatedScale(); // mfixedScale이 true면 1.0
                matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5); // Sim3를 사용해서 매칭을 다시 수행함

                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s); // 얘가 결국 최종적으로 계산된 Sim3를 의미함
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale); // vpMapPointMatches가 비어있어도, 내부에서 새로운 매칭을 다시 찾는 시도를 함

                // If optimization is succesful stop ransacs and continue
                if(nInliers>=20)
                {
                    bMatch = true;
                    mpMatchedKF = pKF;
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
                    mg2oScw = gScm*gSmw;  // World -> matched candiate keyframe (pKF) -> current keyframe / 그리고 gScm는 최적화된 값이 들어감
                    mScw = Converter::toCvMat(mg2oScw);

                    mvpCurrentMatchedPoints = vpMapPointMatches; // NDT로 SE3를 찾아도 이 부분을 설정해줘야 다음 과정을 진행할 수 있음
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
             mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames(); // 매칭된 키프레임과 공변성 관계에 있는 키프레임들을 추출함
    vpLoopConnectedKFs.push_back(mpMatchedKF); // 
    mvpLoopMapPoints.clear();
    for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        KeyFrame* pKF = *vit;
        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches(); // 각 키 프레임 별로 맵 포인트들을 추출함
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId) // 맵 포인트가 bad가 아니고, 현재 키프레임과 연결된 맵 포인트가 아니라면
                {
                    mvpLoopMapPoints.push_back(pMP); // 맵 포인트를 저장함
                    pMP->mnLoopPointForKF = mpCurrentKF->mnId; // 현재 키프레임과 연결된 맵 포인트로 설정함
                }
            }
        }
    }
    /* 이 과정을 NDT로 수정하고, mScw를 구한 다음에  나머지 과정은 그대로 똑같이 수행할 수 있는지?*/

    // Find more matches projecting with the computed Sim3
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }

    if(nTotalMatches>=40)
    {
        for(int i=0; i<nInitialCandidates; i++)
            if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
                mvpEnoughConsistentCandidates[i]->SetErase();
        return true;
    }
    else
    {
        for(int i=0; i<nInitialCandidates; i++)
            mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

}

// Generate Sim3 from NDT registration result
bool LoopClosing::ComputeSim3NDT()
{

    mpCurrentKF->SetNotErase();
    mpMatchedKFPCD->SetNotErase();
    
    /* Generate Submap from mvpSubMapKFs using relative pose among closest keyframe and near keyframes */
    
    // Get relative pose between closest keyframe and current keyframe
    cv::Mat Tst = mpCurrentKF->GetPose() * mpMatchedKFPCD->GetPoseInverse(); // target(candiate keyframe:Tcw) -> source(current keyframe:Twc)
    Eigen::Matrix4f Tst_eigen = Eigen::Matrix4f::Identity();
    Tst_eigen = Converter::toMatrix4f(Tst);
    
    // Generate submap pointcloud 
    pcl::PointCloud<PointT>::Ptr submapCloud(new pcl::PointCloud<PointT>());
    submapCloud->reserve(mpMatchedKFPCD->GetPointCloud()->size() * mvpSubMapKFs.size());
    Eigen::Matrix4f Tnt_eigen = Eigen::Matrix4f::Identity();

    for (const auto& candiKF : mvpSubMapKFs)
    {
        // Calculate relative pose between closest keyframe to candiKF.
        cv::Mat Tnt = candiKF->GetPose() * mpMatchedKFPCD->GetPoseInverse(); // n:near(candiate keyframe:Tcw) <- target(closest keyframe:Twc)
        Tnt_eigen = Converter::toMatrix4f(Tnt);

        for (const auto& point : candiKF->GetPointCloud()->points)
        {
            PointT destPoint;
            destPoint.getVector4fMap() = Tnt_eigen * point.getVector4fMap();
            submapCloud->push_back(destPoint);
        }
    }

    // Voxel filter using octree! -> 어차피 NDT 복셀화 할건데 이거 필요한가?
    submapCloud->width = submapCloud->size();
    submapCloud->height = 1;
    submapCloud->is_dense = false;
    
/*     pcl::octree::OctreePointCloud<PointT> octree(mSubmapVoxleSize);
    octree.setInputCloud(submapCloud);
    octree.addPointsFromInputCloud();

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    octree.getOccupiedVoxelCenters(filtered->points);

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false; */
    
    // Now time to do registration ~

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    mpRegistration->setInputTarget(submapCloud);
    mpRegistration->setInputSource(mpCurrentKF->GetPointCloud());
    mpRegistration->align(*aligned, Tst_eigen);
    
    // Get Matching Score
    float score = mpRegistration->getFitnessScore();
    if ( !mpRegistration->hasConverged() || score > 0.3 ) // 0.3 is tested value
    {
        return false;
    }
    
    // Get Transformation Matrix
    Eigen::Matrix4f Tsc_eigen = mpRegistration->getFinalTransformation(); // candiate keyframe -> estimated current keyframe
    // registration을 CandiKF의 카메라 좌표계가 기준되도록 서브맵을 구성하고 있었고, 소스의 시작 위치가 동일하게 여기서 시작
    // Guess값을 통해 currentKF와 CandiKF 사이의 상대 변환을 하여 원래 상대위치로 움직임
    // getFianlTransformation()의 리턴값은 Guess값을 포함한 Source to Target이니 결국 원점이 같은 경우
    // CandiKF -> Estimated CurrentKF로의 변환을 의미함
    // Sim3 또한 CandiKF -> Estimated CurrentKF로의 변환을 의미함

    // Generate Sim3
    cv::Mat Tsc = Converter::toCvMat(Tsc_eigen);
    cv::Mat R = Tsc.rowRange(0,3).colRange(0,3);
    cv::Mat t = Tsc.rowRange(0,3).col(3);
    const float s = 1.0f; // mfixedScale이 true면 1.0
    vector<MapPoint*> vpMapPointMatches;
    
    ORBmatcher matcher(0.75,true); // 
    matcher.SearchBySim3(mpCurrentKF,mpMatchedKFPCD,vpMapPointMatches,s,R,t,7.5); // Sim3를 사용해서 매칭을 다시 수행함

    g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s); // 얘가 결국 최종적으로 계산된 Sim3를 의미함
    const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, mpMatchedKFPCD, vpMapPointMatches, gScm, 10, mbFixScale); // vpMapPointMatches가 비어있어도, 내부에서 새로운 매칭을 다시 찾는 시도를 함
    
    // Check wether optimization is succesful by number of Inliers(matched points)
    if(nInliers>=20)
    {
        mpMatchedKF = mpMatchedKFPCD;
        g2o::Sim3 gSmw(Converter::toMatrix3d(mpMatchedKFPCD->GetRotation()),Converter::toVector3d(mpMatchedKFPCD->GetTranslation()),1.0);
        mg2oScw = gScm*gSmw;  // World -> matched candiate keyframe (pKF) -> current keyframe / 그리고 gScm는 최적화된 값이 들어감
        mScw = Converter::toCvMat(mg2oScw);

        mvpCurrentMatchedPoints = vpMapPointMatches; // NDT로 SE3를 찾아도 이 부분을 설정해줘야 다음 과정을 진행할 수 있음
    }
    else
    {
        mpCurrentKF->SetErase();
        mpMatchedKFPCD->SetErase();
        return false;
    }
    
    /* Same Processing with ComputeSim3() */

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKFPCD->GetVectorCovisibleKeyFrames(); // 매칭된 키프레임과 공변성 관계에 있는 키프레임들을 추출함
    vpLoopConnectedKFs.push_back(mpMatchedKFPCD); // 
    mvpLoopMapPoints.clear();
    for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        KeyFrame* pKF = *vit;
        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches(); // 각 키 프레임 별로 맵 포인트들을 추출함
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId) // 맵 포인트가 bad가 아니고, 현재 키프레임과 연결된 맵 포인트가 아니라면
                {
                    mvpLoopMapPoints.push_back(pMP); // 맵 포인트를 저장함
                    pMP->mnLoopPointForKF = mpCurrentKF->mnId; // 현재 키프레임과 연결된 맵 포인트로 설정함
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }

    if(nTotalMatches>=40)
    {
        return true;
    }
    else
    {
        mpCurrentKF->SetErase();
        mpMatchedKFPCD->SetErase();
        return false;
    }

    return false;
}

void LoopClosing::CorrectLoop()
{
    cout << "Loop detected!" << endl;

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();

    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx = true;

        if(mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
    }

    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);
    // 최적화를 진행한 후의 위치를 CorrectedSim3에 저장함
    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oScw;
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();


    {
        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        
        // 현재 키프레임과 연결된 키프레임들을 순회
        for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++) 
        {
            KeyFrame* pKFi = *vit;

            cv::Mat Tiw = pKFi->GetPose();

            if(pKFi!=mpCurrentKF) // 현재꺼만 빼고, 
            {
                cv::Mat Tic = Tiw*Twc; //  여기서 i는 이웃 키프레임을 의미함 (current keyframe과 연결된 키프레임) / Tic는 current keyframe을 기준으로 이웃 키프레임의 위치를 의미함
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw; // world -> 최적회된 sim3 pose * 현재 키 프레임 -> 이웃 키 프레임 : 결국 sim3 사용해서 다른 키프레임 포즈 변경
                // Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[pKFi]=g2oCorrectedSiw;
            }

            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            NonCorrectedSim3[pKFi]=g2oSiw; // 변경 되지 않은 애들도 저장
        }

        // 현재 키 프레임과 연결된 이웃 키 프레임들에 의해 관측된 맵 포인트를 순회하며 위치를 수정
        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++) // correctSim3는 std::Map<KeyFrame, g2o::Sim3> 형태로 되어있음
        {
            KeyFrame* pKFi = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];

            // Covisible Landmark들의 위치를 보정 (이웃 키프레임들의 위치를 보정함)
            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches(); // 이웃 키프레임들의 맵 포인트들을 추출함 / 좌표계는 이웃 키프레임의 좌표계
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                MapPoint* pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                cv::Mat P3Dw = pMPi->GetWorldPos();
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw)); // map()는 맵포인트의 월드 좌표계 위치(eigP3Dw)를 키프레임 좌표계로 변환
                // 결국 월드 좌표계(보정 전) -> 키프레임 좌표계(보정전) -> 월드 좌표계(보정후)로 변환하는 것임

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                pMPi->SetWorldPos(cvCorrectedP3Dw); // 위치를 저장
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                pMPi->mnCorrectedReference = pKFi->mnId;
                pMPi->UpdateNormalAndDepth();
            }

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
            double s = g2oCorrectedSiw.scale();

            eigt *=(1./s); //[R t/s;0 1]

            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

            pKFi->SetPose(correctedTiw);

            // Make sure connections are updated
            pKFi->UpdateConnections();
        }

        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
        {
            if(mvpCurrentMatchedPoints[i])
            {
                MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
                MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
                if(pCurMP)
                    pCurMP->Replace(pLoopMP);
                else
                {
                    mpCurrentKF->AddMapPoint(pLoopMP,i);
                    pLoopMP->AddObservation(mpCurrentKF,i);
                    pLoopMP->ComputeDistinctiveDescriptors();
                }
            }
        }

    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3);


    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    map<KeyFrame*, set<KeyFrame*> > LoopConnections;

    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

        // Update connections. Detect new links.
        pKFi->UpdateConnections();
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    // Optimize graph
    Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);

    mpMap->InformNewBigChange();

    // Add loop edge
    mpMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment
    mbRunningGBA  = true;
    mbFinishedGBA = false;
    mbStopGBA     = false;
    mpThreadGBA   = new thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();

    mLastLoopKFid = mpCurrentKF->mnId;
}

void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap)
{
    ORBmatcher matcher(0.8);

    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        KeyFrame* pKF = mit->first;

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        vector<MapPoint*> vpReplacePoints(mvpLoopMapPoints.size(),static_cast<MapPoint*>(NULL));
        matcher.Fuse(pKF,cvScw,mvpLoopMapPoints,4,vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        const int nLP = mvpLoopMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPoint* pRep = vpReplacePoints[i];
            if(pRep)
            {
                pRep->Replace(mvpLoopMapPoints[i]);
            }
        }
    }
}


void LoopClosing::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
        unique_lock<mutex> lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }
}

void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid=0;
        mbResetRequested=false;
    }
}

void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF)
{
    cout << "Starting Global Bundle Adjustment" << endl;

    bool idx =  mnFullBAIdx;
    Optimizer::GlobalBundleAdjustemnt(mpMap,10,&mbStopGBA,nLoopKF,false);

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    {
        unique_lock<mutex> lock(mMutexGBA);
        if(idx!=mnFullBAIdx)
            return;

        if(!mbStopGBA)
        {
            cout << "Global Bundle Adjustment finished" << endl;
            cout << "Updating map ..." << endl;
            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped

            while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            // Get Map Mutex
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

            // Correct keyframes starting at map first keyframe
            list<KeyFrame*> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(),mpMap->mvpKeyFrameOrigins.end());

            while(!lpKFtoCheck.empty())
            {
                KeyFrame* pKF = lpKFtoCheck.front();
                const set<KeyFrame*> sChilds = pKF->GetChilds();
                cv::Mat Twc = pKF->GetPoseInverse();
                for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                {
                    KeyFrame* pChild = *sit;
                    if(pChild->mnBAGlobalForKF!=nLoopKF)
                    {
                        cv::Mat Tchildc = pChild->GetPose()*Twc;
                        pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                        pChild->mnBAGlobalForKF=nLoopKF;

                    }
                    lpKFtoCheck.push_back(pChild);
                }

                pKF->mTcwBefGBA = pKF->GetPose();
                pKF->SetPose(pKF->mTcwGBA);
                lpKFtoCheck.pop_front();
            }

            // Correct MapPoints
            const vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();

            for(size_t i=0; i<vpMPs.size(); i++)
            {
                MapPoint* pMP = vpMPs[i];

                if(pMP->isBad())
                    continue;

                if(pMP->mnBAGlobalForKF==nLoopKF)
                {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);
                }
                else
                {
                    // Update according to the correction of its reference keyframe
                    KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

                    if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                        continue;

                    // Map to non-corrected camera
                    cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                    cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

                    // Backproject using corrected camera
                    cv::Mat Twc = pRefKF->GetPoseInverse();
                    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                    cv::Mat twc = Twc.rowRange(0,3).col(3);

                    pMP->SetWorldPos(Rwc*Xc+twc);
                }
            }

            mpMap->InformNewBigChange();

            mpLocalMapper->Release();

            cout << "Map updated!" << endl;
        }

        mbFinishedGBA = true;
        mbRunningGBA = false;
    }
}

void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

// Get the loop closure candidate pair.
std::pair<KeyFrame*, KeyFrame*>  LoopClosing::GetLoopClosingPair()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return mPairCandidateLoopPCD;
}
// Get the point position loop closure candidate pair.
std::pair<cv::Mat, cv::Mat> LoopClosing::GetLoopClosingPairPoint()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return mPairCandidateLoopPCDPoint;
}

std::vector<KeyFrame*> LoopClosing::GetSubmapKFs()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return mvpSubMapKFs;
}

} //namespace ORB_SLAM
