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

#ifdef __cplusplus
extern "C"{
#endif 
    
#include <ccv.h>
//int ccv_read(const void* data, ccv_dense_matrix_t** x, int type, int rows, int cols, int scanline);
int ccv_read_impl(void const* in, ccv_dense_matrix_t** x, int type, int rows, int cols, int scanline);

#ifdef __cplusplus
}
#endif

//#include "t.hpp"

class SpecificWorker : public GenericWorker
{
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

