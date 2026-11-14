#include "control/ContactForceOptimizer.h"
#include "common/mathTools.h"
#include "thirdParty/quadProgpp/QuadProg++.hh"
#ifdef solve
#undef solve
#endif

bool ContactForceOptimizer::allocate(const Vec34 &feetFromCom, const VecInt4 &contact,
                                     const std::array<Mat3, 4> &forceToTorque,
                                     const Vec3 &effortLimits, const Vec6 &weights,
                                     const Vec6 &requested, Vec34 &ground) {
    ground.setZero();
    Eigen::Matrix<double, 6, 12> A;
    for (int i = 0; i < 4; ++i) {
        A.block<3, 3>(0, 3 * i) = Mat3::Identity();
        A.block<3, 3>(3, 3 * i) = skew(feetFromCom.col(i));
    }
    int n = contact.sum();
    if (n == 0) {
        _previous.setZero();
    } else {
        Mat12 H = A.transpose() * weights.asDiagonal() * A + .002 * Mat12::Identity();
        Vec12 g = -A.transpose() * weights.asDiagonal() * requested - .001 * _previous;
        quadprogpp::Matrix<double> G(12, 12), CE(12, 3 * (4 - n)), CI(12, 16 * n);
        quadprogpp::Vector<double> g0(12), ce(3 * (4 - n)), ci(16 * n), x(12);
        for (int i = 0; i < 12; ++i) {
            g0[i] = g(i);
            x[i] = 0;
            for (int j = 0; j < 12; ++j)
                G[i][j] = H(i, j);
            for (int j = 0; j < 3 * (4 - n); ++j)
                CE[i][j] = 0;
            for (int j = 0; j < 16 * n; ++j)
                CI[i][j] = 0;
        }
        for (int j = 0; j < 3 * (4 - n); ++j)
            ce[j] = 0;
        for (int j = 0; j < 16 * n; ++j)
            ci[j] = 0;
        int e = 0, k = 0;
        const double mu = .55;
        for (int leg = 0; leg < 4; ++leg) {
            if (!contact(leg)) {
                for (int a = 0; a < 3; ++a)
                    CE[3 * leg + a][e++] = 1;
                continue;
            }
            // Inscribed octagon: the tangential vector norm stays below mu*Fz,
            // unlike an axis-aligned square which allows sqrt(2)*mu at corners.
            for (int a = 0; a < 8; ++a) {
                double angle = a * M_PI / 4;
                CI[3 * leg][k] = -std::cos(angle);
                CI[3 * leg + 1][k] = -std::sin(angle);
                CI[3 * leg + 2][k] = mu * std::cos(M_PI / 8);
                ++k;
            }
            CI[3 * leg + 2][k] = 1;
            ci[k++] = -5;
            CI[3 * leg + 2][k] = -1;
            ci[k++] = 180;
            const Mat3 &JT = forceToTorque[leg];
            for (int a = 0; a < 3; ++a)
                for (int sign : {-1, 1}) {
                    for (int j = 0; j < 3; ++j)
                        CI[3 * leg + j][k] = sign * JT(a, j);
                    ci[k] = .8 * effortLimits(a);
                    ++k;
                }
        }
        const double result = quadprogpp::solve_quadprog(G, g0, CE, ce, CI, ci, x);
        if (!std::isfinite(result)) {
            return false;
        }
        for (int i = 0; i < 12; ++i)
            ground.data()[i] = x[i];
        _previous = vec34ToVec12(ground);
    }
    return ground.allFinite();
}
