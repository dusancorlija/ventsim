// rls.hpp - recursive least squares with exponential forgetting.
//
// Fixed size, no allocation, O(N^2) per update: suitable for a 1 kHz loop
// on a Cortex-M4F for N = 3.
#pragma once

#include <array>
#include <cstddef>

namespace ventsim::fw {

template <std::size_t N>
class Rls {
public:
    using Vec = std::array<double, N>;

    explicit Rls(double lambda, double p0 = 1e4) noexcept : lambda_(lambda) { reset(p0); }

    void reset(double p0 = 1e4) noexcept
    {
        theta_.fill(0.0);
        for (std::size_t i = 0; i < N; ++i)
            for (std::size_t j = 0; j < N; ++j)
                p_[i][j] = (i == j) ? p0 : 0.0;
    }

    // y = phi^T theta + noise
    void update(const Vec& phi, double y) noexcept
    {
        Vec pphi{};
        for (std::size_t i = 0; i < N; ++i)
            for (std::size_t j = 0; j < N; ++j)
                pphi[i] += p_[i][j] * phi[j];

        double denom = lambda_;
        for (std::size_t i = 0; i < N; ++i) denom += phi[i] * pphi[i];

        Vec k{};
        for (std::size_t i = 0; i < N; ++i) k[i] = pphi[i] / denom;

        double err = y;
        for (std::size_t i = 0; i < N; ++i) err -= phi[i] * theta_[i];
        for (std::size_t i = 0; i < N; ++i) theta_[i] += k[i] * err;

        // P = (P - k * phi^T P) / lambda, then symmetrise against rounding drift
        for (std::size_t i = 0; i < N; ++i)
            for (std::size_t j = 0; j < N; ++j)
                p_[i][j] = (p_[i][j] - k[i] * pphi[j]) / lambda_;
        for (std::size_t i = 0; i < N; ++i)
            for (std::size_t j = i + 1; j < N; ++j)
                p_[i][j] = p_[j][i] = 0.5 * (p_[i][j] + p_[j][i]);
    }

    const Vec& theta() const noexcept { return theta_; }

private:
    double lambda_;
    Vec theta_{};
    std::array<std::array<double, N>, N> p_{};
};

}  // namespace ventsim::fw
