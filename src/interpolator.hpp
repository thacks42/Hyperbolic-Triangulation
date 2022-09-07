#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <Eigen/Dense>
namespace solver{
    using namespace Eigen;


    Vector2f solve(Vector2f bs1, Vector2f bs2, Vector2f bs3, float R21, float R31, Vector2f est){    
        float R1 = (bs1-est).norm();
        float R2 = (bs2-est).norm();
        float R3 = (bs3-est).norm();
        
        Vector2f h_t(R21 - (R2 - R1), R31 - (R3 - R1));
        
        Matrix2f G_t;
        
        G_t <<  ((bs1[0] - est[0])/R1) - ((bs2[0] - est[0])/R2),
                ((bs1[1] - est[1])/R1) - ((bs2[1] - est[1])/R2),
                ((bs1[0] - est[0])/R1) - ((bs3[0] - est[0])/R3),
                ((bs1[1] - est[1])/R1) - ((bs3[1] - est[1])/R3);
        
        Vector2f delta = (G_t.transpose() * G_t).inverse() * G_t.transpose() * h_t;
        return delta;
    }


    Vector2f solve( Vector2f bs1, Vector2f bs2, Vector2f bs3, Vector2f bs4,
                    float R21, float R31, float R41,
                    Vector2f est){
        
        float R1 = (bs1-est).norm();
        float R2 = (bs2-est).norm();
        float R3 = (bs3-est).norm();
        float R4 = (bs4-est).norm();
        
        Vector3f h_t(R21 - (R2 - R1), R31 - (R3 - R1), R41 - (R4 - R1));
        
        Matrix<float, 3, 2> G_t;
        
        G_t <<  ((bs1[0] - est[0])/R1) - ((bs2[0] - est[0])/R2),
                ((bs1[1] - est[1])/R1) - ((bs2[1] - est[1])/R2),
                ((bs1[0] - est[0])/R1) - ((bs3[0] - est[0])/R3),
                ((bs1[1] - est[1])/R1) - ((bs3[1] - est[1])/R3),
                ((bs1[0] - est[0])/R1) - ((bs4[0] - est[0])/R4),
                ((bs1[1] - est[1])/R1) - ((bs4[1] - est[1])/R4);
        
        Vector2f delta = (G_t.transpose() * G_t).inverse() * G_t.transpose() * h_t;
        return delta;
    }
    
    Vector2f solve(std::vector<Vector2f>& base_stations, std::vector<float>& toa_diff, Vector2f est){
        std::vector<float> Rs;
        Rs.reserve(base_stations.size());
        for(size_t i = 0; i < base_stations.size(); i++){
            Rs.push_back( (base_stations[i] - est).norm() );
        }
        VectorXf h_t(toa_diff.size());
        for(size_t i = 0; i < toa_diff.size(); i++){
            h_t[i] = toa_diff[i] - (Rs[i+1] - Rs[0]);
        }
        MatrixXf G_t = MatrixXf::Zero(toa_diff.size(), 2);
        for(size_t i = 0; i < toa_diff.size(); i++){
            G_t(i, 0) = ((base_stations[0][0] - est[0]) / Rs[0]) - ((base_stations[i+1][0] - est[0]) / Rs[i+1]);
            G_t(i, 1) = ((base_stations[0][1] - est[1]) / Rs[0]) - ((base_stations[i+1][1] - est[1]) / Rs[i+1]);
        }
        Vector2f delta = (G_t.transpose() * G_t).inverse() * G_t.transpose() * h_t;
        return delta;
    }
}








