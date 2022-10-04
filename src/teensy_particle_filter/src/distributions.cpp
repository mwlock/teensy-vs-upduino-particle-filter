#include "distributions.hpp"

double Distributions::sampleNormal(double mean, double std){
    
    /**
     * Ititalise pose of a partile
     * https://cplusplus.com/reference/random/normal_distribution/
     * https://cplusplus.com/reference/random/uniform_real_distribution/
     *
     * @param mean mean point around which to sample
     * @param std standard deviation of normal distribution
     */

    std::default_random_engine generator;
    std::normal_distribution<double> distribution = std::normal_distribution<double>(mean,std);
    double sample = distribution(generator);
    return sample;

    }