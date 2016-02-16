/*
 * Copyright 2008 Kyoto University and Honda Motor Co.,Ltd.
 * All rights reserved.
 * HARK was developed by researchers in Okuno Laboratory
 * at the Kyoto University and Honda Research Institute Japan Co.,Ltd.
 */
#ifndef ___SPECTRAL_MEAN_HPP___
#define ___SPECTRAL_MEAN_HPP___

#include <vector>
#include <list>

using namespace std;

#include <boost/shared_ptr.hpp>

using namespace boost;

class SpectralMean {
    size_t m_dimension;
    size_t m_spectrum_count;
    vector<float> m_mean;
    vector<float> m_spectrum_sum;
public:
    SpectralMean() {}
    SpectralMean(size_t d)
        : m_dimension(d), m_spectrum_count(0), m_spectrum_sum(d, 0.0f), m_mean(d, 0.0f) {}

    void Initialize(size_t d) {
        m_dimension = d;
        m_spectrum_count = 0;
        m_spectrum_sum.resize(d);
        m_mean.resize(d);
        for (int i = 0; i < d; i++) {
            m_spectrum_sum[i] = 0.0f;
            m_mean[i] = 0.0f;
        }
    }
    
    void AddSpectrum(const vector<float>& spec)
    {
        for (int i = 0; i < m_dimension; i++) {
            m_spectrum_sum[i] += spec[i];
        }
        m_spectrum_count++;
    }

    void CalculateMean()
    {
        for (int i = 0; i < m_dimension; i++) {
            m_mean[i] = m_spectrum_sum[i] / m_spectrum_count;
        }
    }

    void Clear()
    {
        for (int i = 0; i < m_dimension; i++) {
            m_spectrum_sum[i] = 0.0f;
        }
        m_spectrum_count = 0;
    }

    const vector<float>& GetSpectralMean() const
    {
        return m_mean;
    }
};

#endif
