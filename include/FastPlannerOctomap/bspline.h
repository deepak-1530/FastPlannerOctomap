// bspline header file

#include<iostream>
#include<math.h>
#include<cmath>
#include<vector>
#include<Eigen/Dense>

#include<fstream>
#include<cstdlib>

namespace BSpline
{
class BSpline
{
    public:
        int order=3;
        int numCtrlPoints, knotSize, numSegments;
        std::vector<float> knotVector;
        std::vector<Eigen::Vector3d> ctrlPoints;
        std::vector<Eigen::Vector3d> splineTrajectory;             // this contains all the points in the spline for all the control points
        std::vector<std::vector<Eigen::Vector3d> > splineSegments; // spline trajectory of each segment
        BSpline(float interval_);                                                 // constructor
        void setOrder(int _order_);
        void getBSplineTrajectory();
        void setControlPoints(std::vector<Eigen::Vector3d> _ctrlPoints_);
        float interval;

    private:
        double coxDeBoorBasis(int i, int k, float u); // this is the spline basis function --> using recursive cox-deboor equation to get bspline basis function
        void setKnotVector();
        void setNumSegments();

};  
} // namesapce

/////////////////////////////////////////////////////////////////////////
/** constructor **/
BSpline::BSpline::BSpline(float interval_)
{
    order = 3;
    std::cout<<"Spline initialized \n";
    interval = interval_;
}

//////////////////////////////////////////////////////////////////////////
/** set control points for which spline is to be calculated **/
void BSpline::BSpline::setControlPoints(std::vector<Eigen::Vector3d> _ctrlPoints_)
{
    ctrlPoints = _ctrlPoints_;
    setKnotVector();
    setNumSegments();
}

//////////////////////////////////////////////////////////////////////////
/** set order of spline **/
void BSpline::BSpline::setOrder(int _order_)
{
    order = _order_;
}

//////////////////////////////////////////////////////////////////////////
/** get the knot vectors **/
void BSpline::BSpline::setKnotVector()
{
    knotSize = order + numCtrlPoints + 1;  // m = n + p + 1

    /**
     *  use the bspline theory
     *  t[i] = 0 if i<k
     *  t[i] = i-k+1 if k<=i<n
     *  t[i] = n-k+2 if i >= n
    **/
    float count  = 0;
    for(int i = 0; i<knotSize; i++)
    {
        int m = i;

        if(i<order)
        {
            knotVector.push_back(0);
        }

        else if(i>=order && i<=numCtrlPoints)
        {
            count += interval;
            knotVector.push_back(count);
        }

        else  if(i > numCtrlPoints)
        {
            //count += 0.25;
            knotVector.push_back(count+interval);//numCtrlPoints - order + 2.0);
        }
        
    }

}

///////////////////////////////////////////////////////////////////////////
/** set the number of segments in the spline **/
void BSpline::BSpline::setNumSegments()
{
    numSegments = numCtrlPoints - order + 2;

    std::cout<<"Number of segments are "<<numSegments<<std::endl;
}

///////////////////////////////////////////////////////////////////////////
/** calculate the bsplines given the control points and the knot vectors **/
void BSpline::BSpline::getBSplineTrajectory()
{
    /** spline for each segment **/
    std::vector<Eigen::Vector3d> splineSegment;

    /** create segment-wise splines **/
    for(int i = 0; i<numSegments; i++)
    {    
        float t_s = knotVector[int(order) - 1 + i];
        
        /** now for each segment take the control points (each segment is affected by 
         * "order" no. of points i.e. here each segment is affected by 4 points) **/
        for(float t=t_s; t<t_s + interval; t=t+0.01) 
        {
            Eigen::Vector3d pt(0,0,0);

            for(int j = i; j<i+order; j++)
                {
                    pt += coxDeBoorBasis(j,order,t)*ctrlPoints[j];
                }

                splineSegment.push_back(pt);
        }   

        splineSegments.push_back(splineSegment);
        splineSegment.clear();
    }

}

//////////////////////////////////////////////////////////////////////////////
/** calculate the spline coefficients using cox-deBoor recursive equations **/
double BSpline::BSpline::coxDeBoorBasis(int i, int k, float t)
{   
    if(k==1)
    {   
        if (t==knotVector[i] && t==knotVector[i + 1] && i>=order-1 && i<=knotSize-order)
            {
                return 1;
            } 
        else if (t >= knotVector[i] && t < knotVector[i + 1] && i>=order-1 && i<=knotSize-order)
            {
                return 1;
            }
        else
            {
                return 0;
            }
    }

    float delT1 = knotVector[i+k-1] - knotVector[i];
    float delT2 = knotVector[i+k] - knotVector[i+1];

    float C1, C2;

    if(delT1==0)
    {
        C1 = 0;
    }
    else
    {
        C1 = ((t - knotVector[i])*coxDeBoorBasis(i,k-1,t))/delT1;
    }

    if (delT2 == 0)
    {
        C2 = 0;
    }
    else
    {
        C2 = ((knotVector[i+k] - t)*coxDeBoorBasis(i+1, k-1, t))/delT2;
    }
    
    return C1 + C2;
}   