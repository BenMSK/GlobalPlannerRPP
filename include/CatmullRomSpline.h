#ifndef Q_MOC_RUN
#pragma once
#include <vector>
#include <math.h>
#include <eigen3/Eigen/Core>
//#include <Eigen/Core>
//#include <usr/include/eigen3/Eigen/Core>

#endif

using namespace std;
using namespace Eigen;

class CatmullRomSpline {
private:
    float M11; float M12; float M13; float M14;
    float M21; float M22; float M23; float M24;
    float M31; float M32; float M33; float M34;
    float M41; float M42; float M43; float M44;

public:
    CatmullRomSpline() {
        M11 = 0.0; M12 = 1.0; M13 = 0.0; M14 = 0.0;
        M21 =-0.5; M22 = 0.0; M23 = 0.5; M24 = 0.0;
        M31 = 1.0; M32 =-2.5; M33 = 2.0; M34 =-0.5;
        M41 =-0.5; M42 = 1.5; M43 =-1.5; M44 = 0.5;
    }

    Vector2d CalCurveInt(double t, Vector2d p1, Vector2d p2, Vector2d p3, Vector2d p4 ) {
        double c1,c2,c3,c4;
        Vector2d ret;
        c1 = M12*p2[0];
        c2 = M21*p1[0] + M23*p3[0];
        c3 = M31*p1[0] + M32*p2[0] + M33*p3[0] + M34*p4[0];
        c4 = M41*p1[0] + M42*p2[0] + M43*p3[0] + M44*p4[0];

        ret[0] = (((c4*t + c3)*t +c2)*t + c1);

        c1 = M12*p2[1];
        c2 = M21*p1[1] + M23*p3[1];
        c3 = M31*p1[1] + M32*p2[1] + M33*p3[1] + M34*p4[1];
        c4 = M41*p1[1] + M42*p2[1] + M43*p3[1] + M44*p4[1];

        ret[1] = (((c4*t + c3)*t +c2)*t + c1);

        return ret;
    };


    vector< Vector2d > PathToCurve(vector< Vector2d > PaPath, int step, int section) {
        int i;
        vector< Vector2d > curvePath;

        if(PaPath.size() == 0)
            return curvePath;

        Vector2d p;
        int v1,v2,v3,v4;

        for(int i=0; i<PaPath.size(); i=i+step) {
            v1 = i-step;
            v2 = i;
            v3 = i+step;
            v4 = i+step*2;

            if( i-step < 0 )
                v1=0;
            if( PaPath.size() <= i+step )
                v3=PaPath.size()-1;
            if( PaPath.size() <= i+step*2 )
                v4=PaPath.size()-1;

            //printf("[%d] - %d %d %d %d\n",PaPath.size(), v1, v2, v3, v4);
            float V = (PaPath[v2][0]-PaPath[v3][0])*(PaPath[v2][0]-PaPath[v3][0]) +(PaPath[v2][1]-PaPath[v3][1])*(PaPath[v2][1]-PaPath[v3][1]);
            double dist = sqrt( V );
            double eT = dist/section;
            //printf("dist = %lf, step = %lf \n", dist, eT);

            int iteration;
            for(double t=0; t<dist; t+=eT) {
                p = CalCurveInt( t/dist, PaPath[v1], PaPath[v2], PaPath[v3], PaPath[v4]);
                curvePath.push_back(p);
            }
        }
        curvePath.push_back( PaPath[PaPath.size()-1] );

        return curvePath;
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
