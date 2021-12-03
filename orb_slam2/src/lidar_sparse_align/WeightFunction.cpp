#include "lidar_sparse_align/WeightFunction.h"

namespace ORB_SLAM2
{
    const float TDistributionScaleEstimator::INITIAL_SIGMA = 5.0;
    const float TDistributionScaleEstimator::DEFAULT_DOF = 5.0;
    const float TDistributionWeightFunction::DEFAULT_DOF = 5.0f;

    TDistributionScaleEstimator::TDistributionScaleEstimator(const float dof)
        : dof_(dof), initial_sigma_(INITIAL_SIGMA)
    {
    }

    float TDistributionScaleEstimator::compute(cv::Mat &errors)
    {
        float initial_lamda = 1.0f / (initial_sigma_ * initial_sigma_);
        int num = 0;
        float lambda = initial_lamda;
        int iterations = 0;
        do
        {
            ++iterations;
            initial_lamda = lambda;
            num = 0.0f;
            lambda = 0.0f;

            const float *data_ptr = errors.ptr<float>();

            for (size_t idx = 0; idx < errors.size().area(); ++idx, ++data_ptr)
            {
                const float &data = *data_ptr;

                if (std::isfinite(data))
                {
                    ++num;
                    lambda += data * data * ((dof_ + 1.0f) / (dof_ + initial_lamda * data * data));
                }
            }

            lambda = float(num) / lambda;
        } while (std::abs(lambda - initial_lamda) > 1e-3);

        return std::sqrt(1.0f / lambda);
    }

    float TDistributionScaleEstimator::compute(vector<float> &errors)
    {
        float initial_lamda = 1.0f / (initial_sigma_ * initial_sigma_);
        int num = 0;
        float lambda = initial_lamda;
        int iterations = 0;
        do
        {
            ++iterations;
            initial_lamda = lambda;
            num = 0;
            lambda = 0.0f;

            for (vector<float>::iterator it = errors.begin(); it != errors.end(); ++it)
            {
                if (std::isfinite(*it))
                {
                    ++num;
                    const float error2 = (*it) * (*it);
                    lambda += error2 * ((dof_ + 1.0f) / (dof_ + initial_lamda * error2));
                }
            }
            lambda = float(num) / lambda;
        } while (std::abs(lambda - initial_lamda) > 1e-3);

        return std::sqrt(1.0f / lambda);
    }

    /**********************************************************************/
    /***************    TDistribution Weight function     ****************/
    /**********************************************************************/
    TDistributionWeightFunction::TDistributionWeightFunction(const float dof)
    {
        parameters(dof);
    }

    void TDistributionWeightFunction::parameters(const float &param)
    {
        dof_ = param;
        normalizer_ = dof_ / (dof_ + 1.0);
    }

    float TDistributionWeightFunction::weight(const float &res)
    {
        // return std::max ( (dof_ + 1.0) / (dof_ + res*res), 0.001 );
        return (dof_ + 1.0) / (dof_ + res * res);
    }

    /**********************************************************************/
    /***************         Huber Weight function         ****************/
    /**********************************************************************/
    const float HuberWeightFunction::DEFAULT_K = 1.345f;

    HuberWeightFunction::HuberWeightFunction(const float k)
    {
        parameters(k);
    }

    void HuberWeightFunction::parameters(const float &param)
    {
        k = param;
    }

    float HuberWeightFunction::weight(const float &res)
    {
        // float sig = 5.000 / 255.00;
        // const float t_abs = std::abs(res);
        // if (t_abs < k * sig)
        //     return 1.0f;
        // else
        //     return k / t_abs;
        const float t_abs = std::abs(res);
        if (t_abs < k)
            return 1.0f;
        else
            return k / t_abs;
    }

    /**********************************************************************/
    /***************         Tukey Weight function         ****************/
    /**********************************************************************/
    const float TukeyWeightFunction::DEFAULT_B = 4.6851f;

    TukeyWeightFunction::TukeyWeightFunction(const float b)
    {
        parameters(b);
    }

    float TukeyWeightFunction::weight(const float &x) const
    {
        const float x_square = x * x;
        if (x_square <= b_square)
        {
            const float tmp = 1.0f - x_square / b_square;
            return tmp * tmp;
        }
        else
        {
            return 0.0f;
        }
    }

    void TukeyWeightFunction::parameters(const float &param)
    {
        b_square = param * param;
    }

    
} // namespace dedvo
