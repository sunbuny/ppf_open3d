//
//  PPF.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 02.08.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#include "PPF.h"
//#include "Constants.h"
#include <eigen3/Eigen/Geometry>
#include <math.h>

//PPF2 PPF::makePPF2(const vector<Vector3f> &pts, const vector<Vector3f> &nor, int i, int j){
//    PPF2 ppf;
//    ppf.i=i;
//    //ppf.j=j;

//    Vector3f m1=pts[i];
//    Vector3f n1=nor[i];
//    Vector3f m2=pts[j];
//    Vector3f n2=nor[j];

//    Vector3f dist = m1-m2;
//    Vector3f dn=dist.normalized();

//    //discretise
//    uint8_t d,n1d,n2d,n1n2;  //all of these are <255
//    d = dist.norm()/Params::ddist; //Euclidean distance
//    n1d = acos(n1.dot(dn))/Params::dangle;
//    n2d = acos(n2.dot(dn))/Params::dangle;
//    n1n2 = acos(n1.dot(n2))/Params::dangle;

//    //hashkey
//    //return p(0)*ndist*ndist*ndist + p(1)*nangle*nangle + p(2)*nangle + p(3);  //TODO does it still work if nangle != ndist?
//    //return k.d + k.n1d*nangle + k.n2d*nangle*nangle + k.n1n2*nangle*nangle*nangle;  //ndist=20 must be smaller than nangle=30
//    ppf.k = (d | (n1d<<8) | (n2d<<16) | (n1n2<<24));

//    //calculate angle
//    ppf.alpha = planarRotAngle(m1,n1,m2);

//    return ppf;

//}

PPF::PPF(){
    //cout<<"default const"<<endl;
}

//Constructor
PPF::PPF(const vector<Vector3f> &pts, const vector<Vector3f> &nor, int i, int j){
    this->i=i;
    //this->j=j;

    //TODO
    //Vector<uint8_t,4> fd;

    Vector3f m1=pts[i];
    Vector3f n1=nor[i];
    Vector3f m2=pts[j];
    Vector3f n2=nor[j];

    Vector4f f = computePPF(m1,n1,m2,n2);

    //discretise
    uint8_t d,n1d,n2d,n1n2;  //all of these are <255
    d=f.x()/Params::getInstance()->ddist;
    n1d=f.y()/Params::getInstance()->dangle;
    n2d=f.z()/Params::getInstance()->dangle;
    n1n2=f.w()/Params::getInstance()->dangle;

    //hashkey
    //return p(0)*ndist*ndist*ndist + p(1)*nangle*nangle + p(2)*nangle + p(3);  //TODO does it still work if nangle != ndist?
    //return k.d + k.n1d*nangle + k.n2d*nangle*nangle + k.n1n2*nangle*nangle*nangle;  //ndist=20 must be smaller than nangle=30
    k = (d | (n1d<<8) | (n2d<<16) | (n1n2<<24));

    //calculate angle
    alpha = planarRotAngle(m1,n1,m2);
}

//F(m1, m2) = (∥d∥2, ∠(n1, d), ∠(n2, d), ∠(n1, n2)),
Vector4f PPF::computePPF(Vector3f m1, Vector3f n1, Vector3f m2, Vector3f n2){
    Vector4f f;
    Vector3f dist = m1-m2;
    Vector3f dn=dist.normalized();

    f.x() = dist.norm(); //Euclidean distance
    f.y() = acos(n1.dot(dn));
    f.z() = acos(n2.dot(dn));
    f.w() = acos(n1.dot(n2));

    return f;
}

void PPF::print(){
    //cout <<"{"<< i <<","<< j<<"} ";
    cout<<k<<endl;
    //cout<<d<<" "<<(180*n1d/M_PI)<<" "<<(180*n2d/M_PI)<<" "<<(180*n1n2/M_PI)<<endl;
}


bool PPF::operator==(const PPF &o) const{
    return k==o.k;//d==o.d && n1d==o.n1d && n2d==o.n2d && n1n2 == o.n1n2;
}

bool PPF::operator<(const PPF &o) const{
    return k < o.k;
}

//hashCode so we can insert this class in unordered_map as key directly
//int PPF::operator()(const PPF& k) const
unsigned int PPF::hashKey()
{
    return k;
}


