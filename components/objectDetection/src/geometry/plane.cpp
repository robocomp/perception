#include "plane.h"

float Plane::normAngle(float theta)
{
    while (theta > 2.*M_PIl)
        theta -= 2.*M_PIl;
    while (theta < -2.*M_PIl)
        theta += 2.*M_PIl;
    return theta;
}

QVec Plane::coefficientsToRotation(const QVec &planeVector, const QVec &initialRots)
{
    QVec v = planeVector;

    // Compute R_x
    float RX = normAngle(atan2(v(1), -v(2)));
		if (abs(RX-initialRots(0)) >= M_PIl)
    {
        if (RX > 0)
            RX -= M_PIl;
        else
            RX += M_PIl;
    }

    // Compute R_y
    QMat m = Rot3D(-RX, 0, 0);
    v = m * v;
    const float RY = normAngle(-atan2(v(0), -v(2)));

    return QVec::vec3(RX, RY, initialRots(2));
}

QVec Plane::coefficientsToTranslation(const QVec coefficients, const QVec &initalTrans)
{
	float A = coefficients(0);
	float B = coefficients(1);
	float C = coefficients(2);
	float D = coefficients(3);
	
	float p1 = initalTrans(0);
	float p2 = initalTrans(1);
	float p3 = initalTrans(2);
	
	float y = (B*C+B*D-A*A*p2+A*B*p1)/(-B*B-A*A);
	float x = (-B*y-C-D)/A;
	float z = (-C*y-B*p3+C*p2)/-B;
	
	return QVec::vec3(x, y, z);
}
