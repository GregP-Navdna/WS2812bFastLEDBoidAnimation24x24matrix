#ifndef VEC2_H
#define VEC2_H

#include <Arduino.h>
#include "lookup_tables.h" // Include our lookup tables

//vec2 template class for vector math operations
template <class T>
class vec2 {
  public:
    T x, y;
    
    vec2() :x(0), y(0) {}
    vec2(T x, T y) : x(x), y(y) {}
    vec2(const vec2& v) : x(v.x), y(v.y) {}
    
    vec2& operator=(const vec2& v) {
      x = v.x;
      y = v.y;
      return *this;
    }
    bool operator==(vec2& v) {
        return x == v.x && y == v.y;
    }

    bool operator!=(vec2& v) {
        return x != v.x || y != v.y; 
    }
    
    vec2 operator+(vec2& v) {
      return vec2(x + v.x, y + v.y);
    }
    vec2 operator-(vec2& v) {
      return vec2(x - v.x, y - v.y);
    }	
    vec2& operator+=(vec2& v) {
      x += v.x;
      y += v.y;
      return *this;
    }
    vec2& operator-=(vec2& v) {
      x -= v.x;
      y -= v.y;
      return *this;
    }	
    vec2 operator+(double s) {
      return vec2(x + s, y + s);
    }
    vec2 operator-(double s) {
      return vec2(x - s, y - s);
    }
    vec2 operator*(double s) {
      return vec2(x * s, y * s);
    }
    vec2 operator/(double s) {
      return vec2(x / s, y / s);
    }
    vec2& operator+=(double s) {
      x += s;
      y += s;
      return *this;
    }
    vec2& operator-=(double s) {
      x -= s;
      y -= s;
      return *this;
    }
    vec2& operator*=(double s) {
      x *= s;
      y *= s;
      return *this;
    }
    vec2& operator/=(double s) {
      x /= s;
      y /= s;
      return *this;
    }

    void set(T x, T y) {
      this->x = x;
      this->y = y;
    }
    void rotateAroundPoint(T cx, T cy, double deg) {
        double theta = deg / 180.0 * M_PI;  
        double c = fastCos(fastRadToDegNorm(theta));
        double s = fastSin(fastRadToDegNorm(theta));

        // Translate point back to the origin:
        x -= cx;
        y -= cy;

        // Rotate point
        double tx = x * c - y * s;
        double ty = x * s + y * c;

        // Translate point back to the original location
        x = tx + cx;
        y = ty + cy;
    }
    
    void rotate(double deg) {
      double theta = deg / 180.0 * M_PI;
      double c = fastCos(fastRadToDegNorm(theta));
      double s = fastSin(fastRadToDegNorm(theta));
      double tx = x * c - y * s;
      double ty = x * s + y * c;
      x = tx;
      y = ty;
    }
    
    // Fast inverse square root
    float Q_rsqrt(float number) {
      long i;
      float x2, y;
      const float threehalfs = 1.5F;
  
      x2 = number * 0.5F;
      y  = number;
      i  = * ( long * ) &y;                       // evil floating point bit level hacking
      i  = 0x5f3759df - ( i >> 1 );               // what the fuck?
      y  = * ( float * ) &i;
      y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
  
      return y;
    }
    
    float length() const {
      return fastSqrt(x * x + y * y);
    }
    
    vec2& normalize() {
      if (length() == 0) return *this;
      *this *= (1.0 / length());
      return *this;
    }
    
    float dist(vec2 v) const {
      vec2 d(v.x - x, v.y - y);
      return d.length();
    }

    void truncate(double length) {
      double angle = fastAtan2(y, x);
      x = length * fastCos(fastRadToDegNorm(angle));
      y = length * fastSin(fastRadToDegNorm(angle));
    }
    
    vec2 ortho() const {
      return vec2(y, -x);
    }
    
    static float dot(vec2 v1, vec2 v2) {
      return v1.x * v2.x + v1.y * v2.y;
    }
    
    static float cross(vec2 v1, vec2 v2) {
      return (v1.x * v2.y) - (v1.y * v2.x);
    }
    
    void makePositive() {
        if (x < 0) x = -x; 
        if (y < 0) y = -y; 
    }

    float mag() const {
        return length();
    }

    float magSq() {
        return (x * x + y * y);
    }

    void limit(float max) {
      if (magSq() > max*max) {
          normalize();
          *this *= max;
      }
    }
};

// Common typedefs
typedef vec2<float> PVector;
typedef vec2<double> vec2d;

#endif // VEC2_H
