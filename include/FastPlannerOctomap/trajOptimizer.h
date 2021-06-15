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
            std::vector<std::vector<Eigen::Vector3d> > edtNoise;
            Eigen::Vector3d trajectory; // current trajectory obtained from fast planner
            std::vector<std::vector<Eigen::Vector3d> > noiseParams; // each entry in this vector is Vector3d(mean, variance, weight). Each entry for each range
            trajOptimizer();
            void addNoiseToEdt(float edtVal, float distToCam); // this generates noise sampled from a mixture of gaussians and adds it to the current edt value

    };
}

trajOptimizer::trajOptimizer::trajOptimizer()
{

}