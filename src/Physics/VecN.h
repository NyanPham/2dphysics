#ifndef VECN_H
#define VECN_H

struct VecN {
    int N;
    float* data;

    VecN();
    VecN(int N);
    VecN(const VecN& v);
    ~VecN(); 
   
    void Zero();
    float Dot(const VecN& v) const;                 // v1.Dot(v2)
   
    VecN& operator = (const VecN& v);               // v1 = v2
    VecN operator + (const VecN& v) const;          // v1 + v2
    VecN operator - (const VecN& v) const;          // v1 - v2
    VecN operator * (const float n) const;          // v * n
    const VecN& operator += (const VecN& v);        // v1 += v2 
    const VecN& operator -= (const VecN& v);        // v2 -= v2
    const VecN& operator *= (const float n);        // v *= n 
    float operator [] (const int index) const;      // v[index]
    float& operator [] (const int index);           // v[index]

};

#endif

