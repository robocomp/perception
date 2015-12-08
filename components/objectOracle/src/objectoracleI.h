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
#ifndef OBJECTORACLE_H
#define OBJECTORACLE_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <ObjectOracle.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompObjectOracle;

class ObjectOracleI : public QObject , public virtual RoboCompObjectOracle::ObjectOracle
{
Q_OBJECT
public:
	ObjectOracleI( GenericWorker *_worker, QObject *parent = 0 );
	~ObjectOracleI();
	
	void getLabelsFromImage(const ColorSeq  &image,  ResultList  &result, const Ice::Current&);

	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:


};

#endif
