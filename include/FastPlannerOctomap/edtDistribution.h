// optimize trajectory using distribution on EDT

#include"Map.h"
#include"utils.h"

#include<cmath>
#include<random> // for adding gaussian random no.

namespace edtDistribution
{
    class edtDistribution
    {
        public:
            Map3D::OctoMapEDT costMap3D; // to use the map parameters
            std::vector<std::vector<float> > edtNoise;
            Eigen::Vector3d trajectory; // current trajectory obtained from fast planner
            std::vector<std::vector<Eigen::Vector3d> > noiseParams; // each entry in this vector is Vector3d(mean, variance, weight). Each entry for each range
            edtDistribution();
            std::vector<float> genEdtDistribution(float edtVal, float distToCam); // this generates noise sampled from a mixture of gaussians and adds it to the current edt value
    };
}

edtDistribution::edtDistribution::edtDistribution()
{
    std::cout<<"Trajectory optimizer initialized "<<std::endl;

    std::cout<<"Enter the noise parameters and generate random samples for each range "<<std::endl;


    for(int i = 0; i<8; i++)
    {
        std::vector<Eigen::Vector3d> localNoiseParams;
        
        Eigen::Vector3d weights, means, variances;
        
        std::cout<<"Enter weights for range ("<<i<<","<<i+1<<") "<<std::endl;
        std::cin>>weights(0)>>weights(1)>>weights(2);
        std::cout<<weights.transpose()<<std::endl;

        localNoiseParams.push_back(weights);

        std::cout<<"Enter means for range ("<<i<<","<<i+1<<") "<<std::endl;
        std::cin>>means(0)>>means(1)>>means(2);
        std::cout<<means.transpose()<<std::endl;

        localNoiseParams.push_back(means);


        std::cout<<"Enter variances for range ("<<i<<","<<i+1<<") "<<std::endl;
        std::cin>>variances(0)>>variances(1)>>variances(2);
        std::cout<<variances.transpose()<<std::endl;

        localNoiseParams.push_back(variances);

        noiseParams.push_back(localNoiseParams);

        // generate the random samples based on these weights, means and variances
        std::cout<<"--------- Generating Noise now for a given GMM --------------"<<std::endl;

        std::default_random_engine de(time(0));
        
        std::vector<float> noise(50,0.0);

        for(int j = 0; j<3; j++)
        {
            std::normal_distribution<float> nd(means(j), sqrt(variances(j)));

            for(int k = 0; k<50; k++)
            {
                noise.at(i) += weights(j)*nd(de);
            }

        }
        
        edtNoise.push_back(noise);

        std::cout<<"GMM random numbers created with the given parameters "<<std::endl;
        std::cout<<"*************************************************************************************"<<std::endl;
    }


}

std::vector<float> edtDistribution::edtDistribution::genEdtDistribution(float edtVal, float distToCam)
{
    // apply greatest integer function to the edtVal to get the index for noise vector
    float edtIndex = floor(distToCam);

    std::vector<float> edt_samples(50,0.0);

    if(edtIndex > 0)
    {
        std::vector<float> noiseVector = edtNoise.at(edtIndex);

        for(int i = 0; i<edt_samples.size(); i++)
        {
            edt_samples.at(i) = edtVal + noiseVector.at(i);
        }
 
    }
    else
    {
        std::cout<<"Occupied space ... "<<std::endl;
    }

    return edt_samples;
    
}