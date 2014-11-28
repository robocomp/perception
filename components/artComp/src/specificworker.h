/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <string.h>
#include <AR/param.h>
#include <AR/ar.h>
#include <AR/arMulti.h>

/**
       \brief
       @author authorname
*/

class SpecificWorker : public GenericWorker
{
Q_OBJECT

private:
  
	float probability;
	bool isSingle;
	int size;
	int id_marker;
	
	///Multimarcas
	ARMultiMarkerInfoT  *mMarker;

	///Imagen obtenida de cameraComp
	RoboCompCamera::imgType imgRGB;
	///Parametros de la camara
	RoboCompCamera::TCamParams camParams;
	RoboCompDifferentialRobot::TBaseState bState;
	RoboCompCommonHead::THeadState hState;
	
public:
  
	virtual bool getPosition(Ice::Double& tx, Ice::Double& ty, Ice::Double& tz, Ice::Double& rx, Ice::Double& ry, Ice::Double& rz);
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	void setParams(RoboCompCommonBehavior::ParameterList params);
	  

public slots:
  
 	void compute();
};

#endif