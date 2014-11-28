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
 
#include "specificworker.h"
#include <iostream>
#include <sys/stat.h>
#include <QtGui/qimage.h>

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)	
{
  //Inicializamos ARToolKit
  size = 0;
  id_marker = -1;
  
  //Cargamos la marca
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}
void SpecificWorker::compute( )
{
  
}

void SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	struct stat buf;
	string path,cameraData;
	
	path = params["markerPath"].value;
		// Llamas a staf con ese buffer
	if(stat(path.c_str(), &buf)==-1){
	    std::cout<<"El fichero "<<path<<" no existe"<<std::endl;
	    exit(-1);
	}
	
	cameraData = params["cameraData"].value;
	
	if(stat(cameraData.c_str(), &buf)==-1){
	    std::cout<<"El fichero "<<cameraData<<" no existe"<<std::endl;
	    exit(-1);
	}
	
	if(params["isSingleMarker"].value.compare("false")==0)
	  isSingle=false;
	else{
	  if(params["isSingleMarker"].value.compare("true")==0){
	    isSingle=true;
	    try{
	      size = atoi(params["markerSize"].value.c_str());
	      if(size<1){
		std::cout<<"El tamaño de la marca tiene que ser >=1"<<std::endl;
		exit(-1);
	      }
	    }
	    catch(...){
	      std::cout<<"Error al obtener el tamaño de la marca"<<std::endl;
	      exit(-1);
	    }
	  }
	  else{
	    std::cout<<"Valor erroneo. isSingleMarker debe ser \"true\" o \"false\""<<std::endl;
	    exit(-1);
	  }
	}
	
	try{
	  probability = atof(params["probability"].value.c_str());
	  if(probability<0 || probability > 1){
	    std::cout<<"El valor de probabilidad tiene que estar comprendido ente 0 y 1"<<std::endl;
	    exit(-1);
	  }
	}
	catch(...){
	  std::cout<<"Error al obtener la probabilidad de busqueda"<<std::endl;
	  exit(-1);
	}
	
	//Cargando los parametros intrínsecos de la camara
	ARParam  wparam,cparam;
	if(arParamLoad(cameraData.c_str(), 1, &wparam)< 0){
	  std::cout<<"Error al cargar los parametros intrínsecos de la camara"<<std::endl;
	  exit(-1);
	}
	arParamChangeSize(&wparam, 640, 480, &cparam);
	arInitCparam(&cparam);   // Inicializamos la camara con "cparam"
	
	
	//Cargando la marca de ART
	if(isSingle){
	  id_marker = arLoadPatt(path.c_str()); 
	  if(id_marker < 0){
	    std::cout<<"Error al cargar la marca"<<std::endl;
	    exit(-1);
	  }
	  std::cout<<"Marca cargada correctamente"<<std::endl;
	}
	else{
	  mMarker = arMultiReadConfigFile(path.c_str());
	  if(mMarker == NULL){
	    std::cout<<"Error al cargar la marca"<<std::endl;
	    exit(-1);
	  }
	  std::cout<<"Multimarca cargado correctamente"<<std::endl;
	}
	
	timer.start(Period);
	
};

bool SpecificWorker::getPosition(Ice::Double& tx, Ice::Double& ty, Ice::Double& tz, Ice::Double& rx, Ice::Double& ry, Ice::Double& rz)
{
  ///ARToolKit variables
  ARUint8 *dataPtr 
  ARMarkerInfo *marker_info;
  int marker_num=0,i,j=-1;
  //uchar* miImg;
  double matrix[3][4]; //matriz de transformacion de la marca
  double p_center[2] = {0.0,0.0}; //Centro de la marca
  
  //std::cout<<"Iteracion"<<std::endl;
  camera->getRGBPackedImage(0,imgRGB,hState,bState);
  
  dataPtr = new ARUint8[640*480*3];
  memcpy(dataPtr,&imgRGB[0],640*480*3);
  
  
    
  if(arDetectMarker(dataPtr, 100, &marker_info, &marker_num) >= 0) {
    if(marker_num!=0){
      
      if(!isSingle && arMultiGetTransMat(marker_info, marker_num, mMarker) <= 0){
	return false;
      }
      
      //cout<<"Encontrada"<<endl;
      for(i=0;i<marker_num;i++){
	if(id_marker==marker_info[i].id){
	  if(j==-1)
	      j=i;
	  else{
	    if(marker_info[j].cf<marker_info[i].cf)
	      j=i;
	  }
	}
      }
      
      if(j!=-1 && marker_info[j].cf>=probability){
	//cout<<"********  OK, Supera la probabilidad "<<probability<<endl;
	arGetTransMat(&marker_info[j], p_center, (double)size, matrix);
	
	tx = matrix[0][3];
	ty = matrix[1][3];
	tz = matrix[2][3];
	
	rx = matrix[0][0];
	ry = matrix[1][1];
	rz = matrix[2][2];
	
      }
      else{
	return false;
      }
      
    }
    else{
      //cout<<"No encontrada"<<endl;
      return false;
    }
  }
  else{
    //std::cout<<"Error en la deteccion de la marca"<<std::endl;  
    return false;
  }
  
  //Copiar los valores de la matriz de transformacion
  
  delete [] dataPtr;
  
  return true;
}
