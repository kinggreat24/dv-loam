/*
 * @Author: Kinggreat24
 * @Date: 2020-08-13 10:52:59
 * @LastEditors: kinggreat24
 * @LastEditTime: 2021-08-18 21:33:24
 * @Description: 
 */
#ifndef WEIGHT_FUNCTION_H
#define WEIGHT_FUNCTION_H

#include <vector>
#include <memory>
#include <cmath>

#include <opencv2/opencv.hpp>

using namespace std;
namespace ORB_SLAM2 {

enum class ScaleEstimatorType{None, TDistributionScale, MADScale, NormalDistributionScale};
enum class WeightFunctionType{None, TDistributionWeight, HuberWeight, TukeyWeight};

class ScaleEstimator
{
public:
    typedef std::shared_ptr<ScaleEstimator> Ptr;

    ScaleEstimator() {}
    virtual ~ScaleEstimator() {}
    virtual float compute(vector<float>& errors) = 0;
    virtual float compute(cv::Mat& errors) = 0;
};

class UnitScaleEstimator : public ScaleEstimator
{
public:
  UnitScaleEstimator() {}
  virtual ~UnitScaleEstimator() {}
  virtual float compute(std::vector<float>& errors) const { return 1.0f; };
};


// T-分布尺度函数
class TDistributionScaleEstimator : public ScaleEstimator
{
public:
    typedef std::shared_ptr<TDistributionScaleEstimator> Ptr;
    TDistributionScaleEstimator(const float dof = DEFAULT_DOF);
    virtual ~TDistributionScaleEstimator() {}
    virtual float compute(vector<float>& errors);
    virtual float compute(cv::Mat& errors);

    static const float DEFAULT_DOF;
    static const float INITIAL_SIGMA;

protected:
    float dof_;
    float initial_sigma_;
};

// estimates scale by computing the median absolute deviation
class MADScaleEstimator : public ScaleEstimator
{
public:
  MADScaleEstimator() {};
  virtual ~MADScaleEstimator() {};
  virtual float compute(std::vector<float>& errors) const;

private:
  static const float NORMALIZER;;
};

// estimates scale by computing the standard deviation
class NormalDistributionScaleEstimator : public ScaleEstimator
{
public:
  NormalDistributionScaleEstimator() {};
  virtual ~NormalDistributionScaleEstimator() {};
  virtual float compute(std::vector<float>& errors) const;
private:
};


/*******************************************     Weight Function     *********************************************/
class WeightFunction
{
public:
    typedef std::shared_ptr<WeightFunction> Ptr;
    virtual ~WeightFunction() {}
    virtual void parameters(const float& param) = 0;
    virtual float weight(const float& x) = 0;
};


class UnitWeightFunction : public WeightFunction
{
public:
  UnitWeightFunction() {};
  virtual ~UnitWeightFunction() {};
  virtual float weight(const float& x) const { return 1.0f; };
};


class TDistributionWeightFunction : public WeightFunction
{
public:
    typedef std::shared_ptr<TDistributionWeightFunction> Ptr;
    TDistributionWeightFunction(const float dof = DEFAULT_DOF);
    virtual ~TDistributionWeightFunction() {}
    virtual void parameters(const float& param);
    virtual float weight(const float& res);

    static const float DEFAULT_DOF;

private:
    float dof_;
    float normalizer_;
};


/**
 * \brief Huber Cost Function
 *
 * Loss function as described by Huber
 * See http://en.wikipedia.org/wiki/Huber_loss_function
 *
 * If e^(1/2) < d
 *  rho(e) = e
 * else
 *               1/2    2
 * rho(e) = 2 d e    - d
 */
class HuberWeightFunction : public WeightFunction
{
public:
  typedef std::shared_ptr<HuberWeightFunction> Ptr;
  HuberWeightFunction(const float k = DEFAULT_K);
  virtual ~HuberWeightFunction() {}
  virtual void parameters(const float& param);
  virtual float weight(const float& res);

  static const float DEFAULT_K;
private:
  float k;
};





/**
 * \brief Tukey Cost Function
 * If e^(1/2) < d
 *  rho(e) = delta2(1-(1-e/delta2)^3)
 * else             
 *  rho(e) = delta2
 */
/**
 * Tukey's hard re-descending function.
 *
 * See:
 *   http://en.wikipedia.org/wiki/Redescending_M-estimator
 */
class TukeyWeightFunction : public WeightFunction
{
public:
  TukeyWeightFunction(const float b = DEFAULT_B);
  virtual ~TukeyWeightFunction() {};
  virtual void parameters(const float& param);
  virtual float weight(const float& x) const;
  
  static const float DEFAULT_B;
private:
  float b_square;
};

} // namespace dedvo


#endif//WEIGHT_FUNCTION_H