//
//  PointPairFeatures.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 02.07.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#include "PPF_PoseEstimation.h"

#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision



#include "PPF.h"
#include "math.h"
//#include "PointCloudManipulation.h"
#include "CPUTimer.h"




namespace PPFPoseEstimation{


CPUTimer timer = CPUTimer(false);
    void double2float(const vector<Vector3d>& vins, vector<Vector3f>& vouts)
    {
        vouts.resize(vins.size());
        int k = 0;
        for (Vector3d v:vins) {
            Vector3f v_f = v.cast<float>();
            vouts[k++] = v_f;
        }

    }

    void summary(std::vector<double> v){  //Like R's summary
        double sum = std::accumulate(v.begin(), v.end(), 0.0);
        double mean = sum / v.size();

        double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
        double std = std::sqrt(sq_sum / v.size() - mean * mean);

        std::sort(v.begin(),v.end());
        double min=v[0];
        double firstQuantile=v[v.size()*0.25];
        double median=v[v.size()*0.5];
        double thirdQuantile=v[v.size()*0.75];
        double max=v[v.size()-1];

        cout<<"Summary of "<<v.size()<<" bucket sizes:"<<endl;
        cout<<"Min\t.25\tMed\tMean\t.75\tMax \tStd"<<endl;
        cout<<min<<" \t"<<firstQuantile<<" \t"<<median<<" \t"<<round(mean*100)*0.01<<" \t"<<thirdQuantile<<" \t"<<max<<" \t"<<round(std*100)*0.01<<endl;


    }

    vector<PPF> getPPFFeatures(PointCloud &pc) {
        int n_pts = pc.points_.size();
        int n_features = n_pts*(n_pts - 1);
        vector<Vector3f> pts_f,nors_f;
        double2float(pc.points_,pts_f);
        double2float(pc.normals_,nors_f);


        vector<PPF> features = vector<PPF>(n_features);
        int k = 0;

#ifndef _OPENMP
        fprintf(stderr, "OpenMP not supported");
#endif

//#pragma omp parallel for
//        timer.tic();
        for (int i = 0; i < n_pts; ++i) {
            for (int j = 0; j < n_pts; ++j) {
                if (i == j) continue;
                features[k++] = PPF(pts_f,nors_f,i,j);
            }
        }
//        timer.toc("计算ppf描述子");
//        timer.tic();
        std::sort(features.begin(),features.end());
//        timer.toc("排序");

        return features;
    }

    vector<MatrixXi> votingDense(PointCloud& mSmall, PointCloud& sSmall){
        //they are sorted
        timer.tic();
        vector<PPF> s1,s2;
#pragma omp parallel
        {
             s1 = getPPFFeatures(mSmall);
             s2 = getPPFFeatures(sSmall);
        }
        timer.toc("getPPFFeatrues for model and scene");
        //vector<uint32_t> votes;


        timer.tic();
        unsigned long Nm = mSmall.points_.size();
        unsigned long Ns = sSmall.points_.size();


        vector<MatrixXi> accVec(Ns);

        cout<<"Voting Dense nangle"<<Params::getInstance()->nangle<<endl;

        for (int i=0; i<Ns; i++) {
            accVec[i]=MatrixXi::Zero(Nm,Params::getInstance()->nangle);
        }

        //i and j start 0 i.e first element
        int i = 0 , j= 0;

        //while either of the two indices reaches end
        while ( i < s1.size() && j < s2.size() )
        {
            //if first array element is lesser, advance that index by one
            if( s1[i] < s2[j] )
            {
                i++;
            }
                //both elements are same, print it, and advance both the pointers
            else if (s1[i]==s2[j])
            {
                int sr=s2[j].i;
                float alpha_scene=s2[j].alpha;

                for(int k = i; s1[k]==s2[j]; k++){ //iterate over multiple same keys in model

                    float alpha=getAngleDiffMod2Pi(s1[k].alpha,alpha_scene);
                    int alphaDiscretised=alpha/Params::getInstance()->dangle;

                    int mr=s1[k].i;

                    //long r=accVec[sr].rows();
                    //long c=accVec[sr].cols();

                    accVec[sr](mr,alphaDiscretised)=accVec[sr](mr,alphaDiscretised)+1;

                    //int vote = (s2[j].i | (s1[k].i << 12) | (alpha << 24));
                    //votes.push_back(vote);
                }
                i++;
                j++;
            }       //otherwise advance second index
            else //if( s1[i] > s2[j] )
            {
                j++;
            }

        }

        timer.toc("dense voting");

        return accVec;
    }






    Poses getTransformationBetweenPointClouds(PointCloud& mSmall, PointCloud& sSmall, bool useVersion2){

        Poses Pests;

        if(true){
            vector<MatrixXi> accVec = votingDense(mSmall,sSmall);
            timer.tic();
            Pests = computePoses(accVec, mSmall, sSmall);
            timer.toc("compute Poses");
        }else{
            //   TrainedModel model = trainModel(mSmall);

//        GlobalModelDescription map = model.modelDescr;
//        PointCloud mSmall = model.mSmall;

//        MatchesWithSceneRefIdx pair = matchSceneAgainstModel(sSmall, map);

//        vector<MatrixXi> accVec = voting(pair,mSmall.pts.size());
//        Pests = computePoses(accVec, mSmall, sSmall);//,pair.second);
        }


    cout<<"beforeClustering: "<<Pests.size()<<endl;
//    printPoses(Pests);
        timer.tic();
        vector<Poses> clusters ;
        clusters = clusterPoses(Pests);
        timer.toc("聚类");
        timer.tic();
        Pests = averagePosesInClusters(clusters);
        timer.toc("平均");
    cout<<"afterClusteringAndAveraging: "<<Pests.size()<<endl;
        timer.printAllTimings();
//    printPoses(Pests);

        //Isometry3f P_meaned = Pests[0].first;

        //cout<<"Pmean "<<P_meaned.matrix()<<endl;

        //Isometry3f P_demeaned = Isometry3f(model.centroid).inverse() * P_meaned;

        //cout<<"Pdemean "<<P_demeaned.matrix()<<endl;

        return Pests; //P_demeaned;
    }


//    void printBucket(Bucket v){
//        cout<<v.size()<< "::::";
//        for(auto i : v){
//            i.print();
//        }
//    }



    float getAngleDiffMod2Pi(float modelAlpha, float sceneAlpha){
        float alpha =  sceneAlpha - modelAlpha; //correct direction

        //cout<<"modelAlpha: "<<degrees(modelAlpha)<<endl;
        //cout<<"sceneAlpha: "<<degrees(sceneAlpha)<<endl;
        //cout<<"alpha: "<<degrees(alpha)<<endl;

        while(alpha<0.0){
            alpha += M_PI*2;
        }
        alpha=fmod(alpha,M_PI*2.0f);

        //now alpha is in interval 0 -> 2*pi

        return alpha;
    }

    /**
     * 匹配阶段，计算了整个投票表格之后，推算最有可能的poses
     * @param accVec 投票表格，共有 Ns个
     * @param m
     * @param s
     * @return 最有可能的poses
     */
    Poses computePoses(vector<MatrixXi>& accVec, PointCloud& m, PointCloud& s){//,vector<int> sceneIndexToI){
        cout<<"PointPairFeatures::computePoses"<<endl;

        Poses vec;

        for (int index=0; index<accVec.size(); index++) {
            MatrixXi acc=accVec[index];

            int sr=index;
            //if(sceneIndexToI.size()>0) sr=sceneIndexToI[index];
            int mr;
            int alphaD;
            int score=acc.maxCoeff(&mr, &alphaD); //TODO detect multiple peaks, but ask betram if this only happens if there are multiple object instances in the scene


            float alpha=(alphaD+0.5f)*Params::getInstance()->dangle;

            //ref points (just one, not both of the ppf)
            Vector3f s_m=s.points_[sr].cast<float>();
            Vector3f s_n=s.normals_[sr].cast<float>();

            Vector3f m_m=m.points_[mr].cast<float>();
            Vector3f m_n=m.normals_[mr].cast<float>();


            Isometry3f P = alignModelToScene(s_m,s_n,m_m,m_n,alpha);

            //P = s.pose * P * m.pose.inverse();

            vec.push_back(std::make_pair(P,score));
        }

        return vec;
    }

    void printPoses(Poses vec){
        //cout<<"getTransformationBetweenPointClouds with pose scores";
        //int m=0;
        vector<double> scores;
        for(int i=0; i<vec.size();i++){
            scores.push_back(vec[i].second);
            //cout<<Pests[i].second<<",";
            //m+=Pests[i].second;
        }
        //float mean = m/Pests.size();
        //cout<<" mean: "<<mean;

        summary(scores);
    }

    Isometry3f alignModelToScene(Vector3f s_m,Vector3f s_n,Vector3f m_m,Vector3f m_n,double alpha){
        //Isometry3f Tgs(ppfScene.T.inverse()); //TODO: check if it makes sense to store and reuse T from ppf's
        Isometry3f Tgs = PPF::alignToOriginAndXAxis(s_m,s_n).inverse();

        AngleAxisf Rx(alpha, Vector3f::UnitX());

        //Isometry3f Tmg(ppfModel.T);
        Isometry3f Tmg = PPF::alignToOriginAndXAxis(m_m,m_n);

        Isometry3f Pest = Tgs*Rx*Tmg;

        return Pest;
    }

//returns true if farthest neighbors in cluster fit within threshold
//http://en.wikipedia.org/wiki/Complete-linkage_clustering
    bool isClusterSimilar(Poses cluster1, Poses cluster2, float thresh_rot_l, float thresh_tra_l){
        for(auto pose2 : cluster2){
            bool isSimilar = std::all_of(cluster1.begin(), cluster1.end(), [&](Pose pose1){return isPoseSimilar(pose1.first, pose2.first, thresh_rot_l, thresh_tra_l);});
            if(!isSimilar) return false;
        }

        return true;
    }

    vector<Pose> fromIsometry(vector<Isometry3f> &isom){
        vector<Pose> vec;
        for (int i = 0; i < isom.size(); ++i) {
            vec[i]=std::make_pair(isom[i],1);
        }
        return vec;
    }

    vector<Poses> clusterPoses (Poses vec, float rot, float tra){

        vec=sortPoses(vec);

        vector< Poses > clusters;

        for(auto pose : vec){
            Poses cluster;
            cluster.push_back(pose); //initially, each cluster contains just one pose;
            clusters.push_back(cluster);
        }

        int n=clusters.size();

        for(int i=0; i<n; n=clusters.size(),i++){
            for(int j=0; j<n; n=clusters.size(),j++){
                if(i==j) continue;
                //cout<<"Cluster1 "<<i<<"\\"<<n-1<<endl;
                Poses cluster1=clusters[i];
                //cout<<"Cluster2 "<<j<<"\\"<<n-1<<endl;
                Poses cluster2=clusters[j];
                //cout<<"size before merge:"<<cluster1.size()<<","<<cluster2.size()<<endl;

                if(isClusterSimilar(cluster1,cluster2,rot,tra)){
                    cluster1.insert(cluster1.end(),cluster2.begin(),cluster2.end());
                    clusters.erase(clusters.begin() + j);
                }
                //cout<<"size after merge:"<<cluster1.size()<<","<<cluster2.size()<<endl;
                clusters[i]=cluster1;
            }
        }

//    cout<<"Produced "<<clusters.size()<<" clusters with each #poses:"<<endl;
//    for(auto cluster : clusters){
//        cout<<cluster.size()<<endl;
//        printPoses(cluster);
//    }


        return clusters;
    }

    Pose averagePosesInCluster(Poses cluster){
        Quaternionf ref; //all quaternions in this cluster must live on same half sphere so mean gives correct result;
        if(cluster.size()==1){
            return cluster[0];
        }else{
            ref = Quaternionf(cluster[0].first.linear());
        }
        //cout<<cluster.size()<<endl;
        Vector3f tra(0,0,0);
        Vector4f rot(0,0,0,0); //w,x,y,z
        int votes=0;
        for(Pose pose : cluster){
            tra += pose.first.translation(); //TODO: maybe weight using number of votes?
            Quaternionf q = Quaternionf(pose.first.linear());
            float d = q.dot(ref);
            if(d<0) q.coeffs() *=-1; //flip to other half sphere
            rot += Vector4f(q.x(),q.y(),q.z(),q.w());  //w last http://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html#ad90ae48f7378bb94dfbc6436e3a66aa2
            votes += pose.second;
        }
        tra /= cluster.size();
        //my stackexchange posts:
        //http://stackoverflow.com/questions/12374087/average-of-multiple-quaternions/
        //http://math.stackexchange.com/questions/61146/averaging-quaternions/

        //mean is a good approx of quaternion interpolation:
        // http://www.mathworks.com/matlabcentral/fileexchange/40098-averaging-quaternions
        // http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
        // http://www.soest.hawaii.edu/wessel/courses/gg711/pdf/Gramkow_2001_JMIV.pdf
        // http://objectmix.com/graphics/132645-averaging-quaternions-2.html
        rot /= cluster.size();

        Isometry3f P = Translation3f(tra)*Quaternionf(rot);

        return std::make_pair(P,votes);

    }

    Vector4f avg_quaternion_markley(MatrixXf Q){
        Matrix4f A = Matrix4f::Zero();
        int M = Q.rows();

        for(int i=0; i<M; i++){
            Vector4f q = Q.row(i);
            A = q*q.adjoint() + A;
        }

        A=(1.0/M)*A;


        SelfAdjointEigenSolver<MatrixXf> eig(A);
//    cout<<"A"<<endl<<A<<endl;
//    cout<<"vecs"<<endl<<eig.eigenvectors()<<endl;
//    cout<<"vals"<<endl<<eig.eigenvalues()<<endl;
        Vector4f qavg=eig.eigenvectors().col(3);
        return qavg;
    }



    Vector4f avg_quaternion_markley(Poses cluster){

        int M = cluster.size();
        MatrixXf Q(M,4);

        for(int i=0; i<M; i++){
            Pose p = cluster[i];
            Quaternionf q = Quaternionf(p.first.linear());
            RowVector4f rot = RowVector4f(q.x(),q.y(),q.z(),q.w());
            Q.row(i)=rot;
        }


        return avg_quaternion_markley(Q);
    }

    Quaternionf avg_quaternion_markleyQ(Poses cluster){
        Vector4f q=avg_quaternion_markley(cluster);
        return Quaternionf(q(0),q(1),q(2),q(3));
    }


    Poses sortPoses(Poses vec){
        //cout<<"clusterPoses"<<endl;
        //printPoses(vec);
        std::sort(vec.begin(), vec.end(), [](const Pose & a, const Pose & b) -> bool{ return a.second > b.second; });
        //cout<<"sorted"<<endl;
        //printPoses(vec);
        return vec;
    }


    Poses averagePosesInClusters(vector<Poses> clusters){
        Poses vec;

        for(auto cluster : clusters){
            vec.push_back(averagePosesInCluster(cluster));
        }

        vec=sortPoses(vec);
        return vec;
    }



} //end namespace