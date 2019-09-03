#include "common_utils.h"

namespace kamaz {
namespace hagen{


  void CommonUtils::get_roration_matrix(Eigen::Vector3f a
      , Eigen::Vector3f b, Eigen::Matrix3f& r){
        a = a/a.norm();
        float b_norm = b.norm();
        b = b/b_norm;
        Eigen::Vector3f v = a.cross(b);
        float s = v.norm();
        float c = a.dot(b);
        Eigen::Matrix3f vx;
        vx << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
        r = Eigen::Matrix3f::Identity(3,3);
        if(s != 0 ){
            r = r + vx + vx*vx*((1-c)/std::pow(s, 2));
        }
  }

  void CommonUtils::generate_samples_from_ellipsoid(Eigen::MatrixXf covmat, Eigen::Matrix3f rotation_mat, 
            Eigen::VectorXf cent, Eigen::MatrixXf& container){

        int ndims = container.cols();
        int npts = container.rows();
        Eigen::EigenSolver<Eigen::MatrixXf> eigensolver;
        eigensolver.compute(covmat);
        Eigen::VectorXf eigen_values = eigensolver.eigenvalues().real();
        Eigen::MatrixXf eigen_vectors = eigensolver.eigenvectors().real();
        std::vector<std::tuple<float, Eigen::VectorXf>> eigen_vectors_and_values; 

        for(int i=0; i<eigen_values.size(); i++){
            std::tuple<float, Eigen::VectorXf> vec_and_val(eigen_values[i], eigen_vectors.row(i));
            eigen_vectors_and_values.push_back(vec_and_val);
        }
        std::sort(eigen_vectors_and_values.begin(), eigen_vectors_and_values.end(), 
            [&](const std::tuple<float, Eigen::VectorXf>& a, const std::tuple<float, Eigen::VectorXf>& b) -> bool{ 
                return std::get<0>(a) <= std::get<0>(b); 
        });
        int index = 0;
        for(auto const vect : eigen_vectors_and_values){
            eigen_values(index) = std::get<0>(vect);
            eigen_vectors.row(index) = std::get<1>(vect);
            index++;
        }

        Eigen::MatrixXf eigen_values_as_matrix = eigen_values.asDiagonal();

        std::random_device rd{};
        std::mt19937 gen{rd()};  
        std::uniform_real_distribution<float> dis(0, 1);
        std::normal_distribution<double> normal_dis{0.0f, 1.0f};
 
        Eigen::MatrixXf pt = Eigen::MatrixXf::Zero(npts, ndims).unaryExpr([&](float dummy){return (float)normal_dis(gen);});
        Eigen::VectorXf rs = Eigen::VectorXf::Zero(npts).unaryExpr([&](float dummy){return dis(gen);});
        Eigen::VectorXf fac = pt.array().pow(2).rowwise().sum();
        Eigen::VectorXf fac_sqrt = fac.array().sqrt();
        Eigen::VectorXf rs_pow = rs.array().pow(1.0/ndims);
        fac = rs_pow.array()/fac_sqrt.array();
        Eigen::VectorXf d = eigen_values_as_matrix.diagonal().array().sqrt();
        for(auto i(0); i<npts; i++){
            container.row(i) = fac(i)*pt.row(i).array();
            Eigen::MatrixXf  fff = (container.row(i).array()*d.transpose().array());
            Eigen::VectorXf bn = rotation_mat*fff.transpose();
            container.row(i) = bn.array() + cent.head(3).array();
        }
        // std::cout << "points: " << container << std::endl;
    }
}
}