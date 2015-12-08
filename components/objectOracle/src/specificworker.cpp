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
    ColorSeq i;
    ResultList r;
    getLabelsFromImage(i, r);
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
    
}

static unsigned int get_current_time(void)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

void SpecificWorker::getLabelsFromImage(const ColorSeq &image, ResultList &result)
{
//     cv::Mat rgb = cv::imread("img.png");
    //cv::Mat M(480,640,CV_8UC1, cv::Scalar::all(0));
    
    ccv_dense_matrix_t* ccv_image = 0;
    
   // ccv_read(rgb.data, &ccv_image, CCV_IO_ANY_RAW | CCV_IO_RGB_COLOR rgb.rows, rgb.cols, rgb.step[0]);
    ccv_read("/home/marcog/robocomp/components/perception/components/objectOracle/bin/img.png", &ccv_image, CCV_IO_ANY_FILE | CCV_IO_RGB_COLOR);

    ccv_convnet_t* convnet = ccv_convnet_read(0, "/home/marcog/ccv/samples/image-net-2012-vgg-d.sqlite3");
    
    ccv_dense_matrix_t* input = 0;
    ccv_convnet_input_formation(convnet->input, ccv_image, &input);
    ccv_matrix_free(ccv_image);
    unsigned int elapsed_time = get_current_time();
    ccv_array_t* rank = 0;
    ccv_convnet_classify(convnet, &input, 1, &rank, 5, 1);
    elapsed_time = get_current_time() - elapsed_time;
    int i;
    for (i = 0; i < rank->rnum - 1; i++)
    {
            ccv_classification_t* classification = (ccv_classification_t*)ccv_array_get(rank, i);
            printf("%d %f ", classification->id + 1, classification->confidence);
    }
    ccv_classification_t* classification = (ccv_classification_t*)ccv_array_get(rank, rank->rnum - 1);
    printf("%d %f\n", classification->id + 1, classification->confidence);
    printf("elapsed time %dms\n", elapsed_time);
    ccv_array_free(rank);
    ccv_matrix_free(input);
    ccv_convnet_free(convnet);
    
    
    
}





