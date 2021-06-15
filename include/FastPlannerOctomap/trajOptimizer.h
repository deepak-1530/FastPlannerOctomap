// optimize trajectory using distribution on EDT

#include"Map.h"
#include"utils.h"

#include<random> // for adding gaussian random no.


// pass the EDT pointer and the trajectory here
// at each value of the EDT add certain samples of mixture of gaussian random noise
// optimize trajectory then using bernstein polynomial

namespace trajOptimizer
{
    class trajOptimizer
    {
        public:
            Map3D::OctoMapEDT costMap3D; // to use the map parameters
            std::vector<std::vector<float> > edtNoise;
            Eigen::Vector3d trajectory; // current trajectory obtained from fast planner
            std::vector<std::vector<Eigen::Vector3d> > noiseParams; // each entry in this vector is Vector3d(mean, variance, weight). Each entry for each range
            trajOptimizer();
            void addNoiseToEdt(float edtVal, float distToCam); // this generates noise sampled from a mixture of gaussians and adds it to the current edt value

    };
}

trajOptimizer::trajOptimizer::trajOptimizer()
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
        std::cout<<"----------------------############# Generating Noise now for a given GMM #####################-------------------"<<std::endl;

        std::default_random_engine de(time(0));
        
        for(int j = 0; j<3; j++)
        {
            std::vector<float> noise;

        }

    }


}

void trajOptimizer::trajOptimizer::addNoise(float edtVal, float distToCam)