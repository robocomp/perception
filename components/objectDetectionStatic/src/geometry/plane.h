/**
 * @file plane.h
 * @brief Object containing general functions to interact with a plane
 * 
 */
#pragma once 

#include <qmat/QMatAll>

/**
 * @class Plane
 */

class Plane 
{
	
public:
		Plane();
		~Plane();
		
		static float normAngle(float theta);
		static QVec coefficientsToRotation(const QVec &planeVector, const QVec &initialRots);
		static QVec coefficientsToTranslation(const QVec coefficients, const QVec &initalTrans);
		
};