#pragma once
#include <iostream>
#include <tf2/LinearMath/Vector3.h>
#include <math.h>
using namespace std;
using namespace tf2;


class Line{
    private:
        float m;
        float b;
        float theta;
    public:
        Line();
        Line(Vector3 pos, Vector3 obj);
        void Refresh(Vector3 pos, Vector3 obj);
        float Dist(Vector3 pos);
        float Evaluate(float x);
        float getTheta();
};
