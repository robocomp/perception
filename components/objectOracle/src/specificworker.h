/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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

/**
       \brief
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>

#ifdef __cplusplus
extern "C"{
#endif 
    
#include "ccv/ccv.h"

#ifdef __cplusplus
}

#endif

//#include "t.hpp"

class SpecificWorker : public GenericWorker
{  
    ccv_convnet_t* convnet;
    fstream file;
    bool first;
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void getLabelsFromImage(const ColorSeq &image, ResultList &result);
public slots:
	void compute(); 	

private:
	
};

#endif

