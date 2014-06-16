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
#include "agmcommonbehaviorI.h"

AGMCommonBehaviorI::AGMCommonBehaviorI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
	// Component initialization...
}


AGMCommonBehaviorI::~AGMCommonBehaviorI()
{
	// Free component resources here
}

// Component functions, implementation
bool AGMCommonBehaviorI::activateAgent(const ParameterMap& prs, const Ice::Current&)
{
	return worker->activateAgent(prs);
}

bool AGMCommonBehaviorI::deactivateAgent(const Ice::Current&)
{
	return worker->deactivateAgent();
}

StateStruct AGMCommonBehaviorI::getAgentState(const Ice::Current&)
{
	return worker->getAgentState();
}

ParameterMap AGMCommonBehaviorI::getAgentParameters(const Ice::Current&)
{
	return worker->getAgentParameters();
}

bool AGMCommonBehaviorI::setAgentParameters(const ParameterMap& prs, const Ice::Current&)
{
	printf("setAgentParameters\n");
	return worker->setAgentParameters(prs);
}

void AGMCommonBehaviorI::killAgent(const Ice::Current&)
{
	worker->killAgent();
}

int AGMCommonBehaviorI::uptimeAgent(const Ice::Current&)
{
	return worker->uptimeAgent();
}

bool AGMCommonBehaviorI::reloadConfigAgent(const Ice::Current&)
{
	return worker->reloadConfigAgent();
}


