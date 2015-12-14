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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
    first = true;
    
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	timer.start(Period);
        
	return true;
}

void SpecificWorker::compute()
{
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
        if(first)
        {
            first =false;
            
            ColorSeq rgbMatrix;
            rgbMatrix.resize(640*480);
            cv::Mat rgb = cv::imread("/home/robocomp/robocomp/files/images/test/perro.png");
            for (int row = 0; row<480; row++)
            {
                for (int column = 0; column<640; column ++)
                {     
                    
                    rgbMatrix[(640*row)+column].blue = rgb.at<cv::Vec3b>(row,column)[0];
                    rgbMatrix[(640*row)+column].green = rgb.at<cv::Vec3b>(row,column)[1];
                    rgbMatrix[(640*row)+column].red = rgb.at<cv::Vec3b>(row,column)[2];
                }
            }
            ResultList result;
            objectoracle_proxy->getLabelsFromImage(rgbMatrix, result);      
            
            for(int i=0; i < result.size();i++ )
            {
                cout<<result[i].name<<" "<<result[i].believe<<endl;
            }
	
        }
}







