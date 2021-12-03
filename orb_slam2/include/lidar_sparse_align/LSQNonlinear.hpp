
#ifndef LSQ_NONLINEAR_HPP
#define LSQ_NONLINEAR_HPP

#include "lidar_sparse_align/LSQNonlinear.h"

namespace ORB_SLAM2
{

    template <int D, typename T>
    bool LSQNonlinearGaussNewton<D, T>::solve()
    {
        x_ = H_.ldlt().solve(Jres_);

        // if((bool) std::isnan((double) x_[0]))
        //     return false;

        if (((x_ - x_).array() == (x_ - x_).array()).all())
            return true;

        return false;
    }

    template <int D, typename T>
    void LSQNonlinearGaussNewton<D, T>::optimize(ModelType &model)
    {

        ModelType old_model = model;
        Eigen::Matrix<float, D, D> old_H = Eigen::Matrix<float, D, D>::Identity();

        for (iter_ = 0; iter_ < max_iteration_; ++iter_)
        {

            startIteration();

            // compute initial error
            n_measurement_ = 0;

            double new_chi2 = build_LinearSystem(model);

            // add prior
            if (have_prior_)
            {
                apply_prior(model);
            }

            if (!solve())
            {
                std::cout << "[LSQNonlinearGaussNewton] Hessian is closed to singular!" << endl;
                std::cout << "H = " << H_ << std::endl;
                std::cout << "Jres = " << Jres_ << std::endl;
                stop_ = true;
            }
            
            if (verbose_)
                cerr << iter_ << ", " << new_chi2 << ", " << chi2_ << endl;

            if ((iter_ > 0 && new_chi2 > chi2_) || stop_)
            {
                if (verbose_)
                {
                    std::cout << "It. " << iter_
                              << "\t Failure"
                              << "\t new_chi2 = " << new_chi2
                              << "\t Error increased. Stop optimizing."
                              << std::endl;
                }

                model = old_model; // rollback
                H_ = old_H;
                //            status_ = false;
                break;
            }
            
            ModelType new_model;
            update(model, new_model);
            old_model = model;
            old_H = H_;
            model = new_model;

            chi2_ = new_chi2;

            if (verbose_)
            {
                std::cout << "It. " << iter_
                          << "\t Success"
                          << "\t new_chi2 = " << new_chi2
                          << "\t n_meas = " << n_measurement_
                          << "\t x_norm = " << norm_max(x_)
                          << std::endl;
            }

            finishIteration();

            // stop when converged, i.e. update step too small
            if (norm_max(x_) < eps_)
            {
                status_ = true;

                if (((x_ - x_).array() != (x_ - x_).array()).all())
                    status_ = false;

                break;
            }
        }
    }

    template <int D, typename T>
    bool LSQNonlinearLevenbergMarquardt<D, T>::solve()
    {
        x_ = H_.ldlt().solve(Jres_);

        // if((bool) std::isnan((double) x_[0]))
        //     return false;

        if (((x_ - x_).array() == (x_ - x_).array()).all())
            return true;

        return false;
    }

    template <int D, typename T>
    void LSQNonlinearLevenbergMarquardt<D, T>::optimize(ModelType &model)
    {

        ModelType old_model = model;
        Eigen::Matrix<float, D, D> old_H = Eigen::Matrix<float, D, D>::Identity();

        chi2_ = build_LinearSystem(model);

        //    mu_ = 0.01f;

        if (mu_ < 0.0f)
        {
            float H_max_diag;
            double tau = 1e-4;
            for (size_t i = 0; i < D; ++i)
            {
                H_max_diag = max(H_max_diag, fabs(H_(i, i)));
            }

            mu_ = tau * H_max_diag;
        }
        //    cerr << "[LSQNonlinearGaussNewton] Residuals = " << res << endl;

        for (iter_ = 0; iter_ < max_iteration_; ++iter_)
        {

            rho_ = 0.0;
            n_trial_ = 0;
            ModelType new_model;

            cerr << "iteration : " << iter_ << endl;

            do
            {
                n_measurement_ = 0;
                double new_chi2 = -1.0f;
                H_.setZero(D, D);
                Jres_.setZero(D, 1);
                build_LinearSystem(model);
                H_ += (H_.diagonal() * mu_).asDiagonal();

                //            cerr << H_.matrix() << endl;
                cerr << "mu : " << mu_ << ", " << iter_ << ", " << n_trial_ << endl;
                cerr << "H_ diagonal vec : " << endl
                     << H_.diagonal() << endl;

                //            cerr << H_.matrix() << endl;

                if (!solve())
                {
                    cerr << "[LSQNonlinearLevenbergMarquardt] Hessian is closed to singular!" << endl;
                    //                stop_ = true;

                    rho_ = -1.0;
                }
                else
                {
                    update(model, new_model);

                    n_measurement_ = 0;
                    new_chi2 = build_LinearSystem(new_model);
                    rho_ = chi2_ - new_chi2;
                }

                if (rho_ > 0.0f)
                {
                    model = new_model;
                    chi2_ = new_chi2;
                    old_model = model;
                    old_H = H_;

                    if (norm_max(x_) < eps_)
                    {
                        stop_ = true;

                        cerr << "stop rho : " << rho_ << endl;

                        //                    if ( ((x_ - x_).array() != (x_ - x_).array()).all() )
                        //                        status_ = false;

                        //                    break;
                    }

                    mu_ *= max(1 / 3.0, min(1 - pow(2 * rho_ - 1, 3), 2. / 3.));
                    nu_ = 2.0;
                }
                else
                {
                    cerr << "dont stop rho : " << rho_ << endl;
                    mu_ *= nu_;
                    nu_ *= 2.;

                    ++n_trial_;
                    if (n_trial_ >= n_trial_max_)
                        stop_ = true;
                }

            } while (!(rho_ > 0.0 || stop_));

            if (stop_)
            {
                if (rho_ < 0)
                {
                    model = old_model;
                    H_ = old_H;
                }
                break;
            }
        }
    }

} // namespace dedvo

#endif //LSQ_NONLINEAR_HPP