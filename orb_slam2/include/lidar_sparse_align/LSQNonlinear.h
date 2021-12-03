
#ifndef LSQ_NONLINEAR_H
#define LSQ_NONLINEAR_H

#include <iostream>
#include <Eigen/Core>

// #include <sophus/se3.hpp>
#include "sophus/se3.h"
#include "lidar_sparse_align/WeightFunction.h"

namespace ORB_SLAM2
{

 /**
     * \brief Abstract Class for solving nonlinear least-squares (NLLS) problems.
     *
     * The function implements two algorithms: Levenberg Marquardt and Gauss Newton
     *
     * Example implementations of this function can be found in the rpl_examples
     * package: img_align_2d.cpp, img_align_3d.cpp
     *
     * Template Parameters:
     * D  : dimension of the residual
     * T  : type of the model, e.g. SE2, SE3
     */
template <int D, typename T>// D是维度，E是模型类型
class LSQNonlinear
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef T ModelType;
    typedef std::shared_ptr<LSQNonlinear<D, T>> Ptr;

    void setVerbose(bool verbose){verbose_ = verbose;}

protected:
    Eigen::Matrix<float, D, D> H_;                 //!< Hessian approximation
    Eigen::Matrix<float, D, 1> Jres_;              //!< Jacobian x Residual
    Eigen::Matrix<float, D, 1> x_;                 //!< update step
    
    bool have_prior_;
    ModelType prior_;
    Eigen::Matrix<double, D, D> I_prior_;          //!< Prior information matrix (inverse covariance)
    double chi2_;

    

    virtual double build_LinearSystem (ModelType& model) = 0;

    virtual bool solve () = 0;

    virtual void update (const ModelType& old_model, ModelType& new_model) = 0;

    virtual void apply_prior (const ModelType& current_model) { }

    virtual void startIteration() {}

    virtual void finishIteration() {}

    virtual void finishTrial() {}

public:
    size_t iter_, max_iteration_;
    size_t n_measurement_;        //!< Number of measurements
    bool stop_;
    double eps_;
    bool status_;

    bool verbose_;               //!< Output Statistics
         
    // weight function
    // bool use_weights_;
    // float scale_;
    // ScaleEstimator::Ptr scale_estimator_;
    // WeightFunction::Ptr weight_function_;

    LSQNonlinear()
        : max_iteration_(100)
        , verbose_(false)
        , have_prior_(false)
        , chi2_(0.0f)
        , n_measurement_(0)
    { }

    LSQNonlinear(size_t max_iteration)
        : max_iteration_(max_iteration)
    { }
    virtual ~LSQNonlinear() {}

    virtual void optimize(ModelType &model) = 0;

    inline double norm_max(const Eigen::Matrix<float, D, 1>& v)
    {
      double max = -1;
      for (int i=0; i<v.size(); i++)
      {
        double abs = fabs(v[i]);
        if(abs>max){
          max = abs;
        }
      }
      return max;
    }
};

template <int D, typename T>
class LSQNonlinearGaussNewton : public LSQNonlinear <D, T>
{
public:
    typedef T ModelType;
    typedef std::shared_ptr<LSQNonlinearGaussNewton<D, T>> Ptr;

    using LSQNonlinear<D, T>::iter_;
    using LSQNonlinear<D, T>::max_iteration_;
    using LSQNonlinear<D, T>::H_;
    using LSQNonlinear<D, T>::Jres_;
    using LSQNonlinear<D, T>::x_;

    using LSQNonlinear<D, T>::stop_;
    using LSQNonlinear<D, T>::chi2_;
    using LSQNonlinear<D, T>::eps_;
    using LSQNonlinear<D, T>::status_;
    using LSQNonlinear<D, T>::norm_max;

    using LSQNonlinear<D, T>::have_prior_;
    using LSQNonlinear<D, T>::verbose_;
    using LSQNonlinear<D, T>::n_measurement_;

    

protected:
    virtual double build_LinearSystem (ModelType& model) = 0;

    virtual bool solve ();

    virtual void update (const ModelType& old_model, ModelType& new_model) = 0;

    virtual void apply_prior (const ModelType& current_model) { }

    virtual void startIteration() {}

    virtual void finishIteration() {}

    virtual void finishTrial() {}

public:
    LSQNonlinearGaussNewton()
        : LSQNonlinear<D,T>()//, max_iteration_(100), stop_(false), eps_(0.000001), status_(false), use_weights_(true), scale_(1.0)
    {
        iter_ = 0;
        max_iteration_ = 100;
        stop_ = false;
        eps_ = 1e-10;
        status_ = false;

        have_prior_    = true;
        verbose_       = false;
        n_measurement_ = 0;
    }

    virtual ~LSQNonlinearGaussNewton() {}

    virtual void optimize(ModelType &model);
};










template <int D, typename T>
class LSQNonlinearLevenbergMarquardt : public LSQNonlinear <D, T>
{
public:
    typedef T ModelType;
    typedef std::shared_ptr<LSQNonlinearLevenbergMarquardt<D, T>> Ptr;

    using LSQNonlinear<D, T>::iter_;
    using LSQNonlinear<D, T>::max_iteration_;
    using LSQNonlinear<D, T>::H_;
    using LSQNonlinear<D, T>::Jres_;
    using LSQNonlinear<D, T>::x_;

    using LSQNonlinear<D, T>::stop_;
    using LSQNonlinear<D, T>::chi2_;
    using LSQNonlinear<D, T>::eps_;
    using LSQNonlinear<D, T>::status_;
    using LSQNonlinear<D, T>::verbose_;

    using LSQNonlinear<D, T>::norm_max;
    using LSQNonlinear<D, T>::n_measurement_;

    double mu_;
    double rho_;
    double nu_;

    size_t n_trial_, n_trial_max_;

protected:
    virtual double build_LinearSystem (ModelType& model) = 0;

    virtual bool solve ();

    virtual void update (const ModelType& old_model, ModelType& new_model) = 0;

    virtual void apply_prior (const ModelType& current_model) { }

    virtual void startIteration() {}

    virtual void finishIteration() {}

    virtual void finishTrial() {}

public:
    LSQNonlinearLevenbergMarquardt()
        : LSQNonlinear<D,T>()//, max_iteration_(100), stop_(false), eps_(0.000001), status_(false), use_weights_(true), scale_(1.0)
    {
        iter_          = 0;
        max_iteration_ = 100;
        stop_          = false;
        eps_           = 1e-10;
        status_        = false;
        verbose_       = false;

        
        mu_            = 0.01f;
        nu_            = 2.0;

        n_trial_       = 0;
        n_trial_max_   = 5;
    }

    virtual ~LSQNonlinearLevenbergMarquardt() {}

    virtual void optimize(ModelType &model);
};

} // namespace dedvo


#endif//LSQ_NONLINEAR_H