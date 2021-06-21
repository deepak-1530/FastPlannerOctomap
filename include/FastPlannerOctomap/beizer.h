/****
 * Generate Beizer curves given a set of waypoints
****/

#include"utils.h"

namespace Beizer
{
    class Beizer
        {
            public:
                int order;
                Beizer(int order_);
                std::vector<Eigen::Vector3d> genBeizerCurve(std::vector<Eigen::Vector3d> controlPoints);
            
            private:
                double binomialCoefficient(int n, int i);
                float factorial(int x);

        };
}

/*** Constructor ***/
Beizer::Beizer::Beizer(int order_)
{
    std::cout<<"Beizer curve trajectory class initialized "<<std::endl;
    order = order_;
}

/*** function to generate beizer curve ***/
std::vector<Eigen::Vector3d> Beizer::Beizer::genBeizerCurve(std::vector<Eigen::Vector3d> controlPoints)
{
    std::vector<Eigen::Vector3d> beizerTraj;

    if(order<controlPoints.size())
        order = controlPoints.size();

   for(float t = 0; t<=1; t=t+0.1)
   {
    Eigen::Vector3d pt;
    pt(0) = 0;
    pt(1) = 0;
    pt(2) = 0;

    for(int n = 0; n<order; n++)
    {
        Eigen::Vector3d cp = controlPoints.at(n);

        double binCoeff    = binomialCoefficient(order, n);
        
        // generate beizer curve for each set of waypoints
        for(int i = 0; i<3; i++)
        {
            pt(i) += binCoeff*pow((1.0-t), (order-n))*pow(t,n)*cp(i);
        }

        
    }

    beizerTraj.push_back(pt);
   }

   return beizerTraj;

}

double Beizer::Beizer::binomialCoefficient(int n, int i)
{
    return double(factorial(n))/(double(factorial(n-i))*double(factorial(i)));
}

float Beizer::Beizer::factorial(int x)
{
    if(x==1 || x==0)
    {
        return 1;
    }

    else
    {
        return float(x*factorial(x-1));
    }
}