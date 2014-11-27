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

#include <QMutexLocker>

#include <genericworker.h>

#include <boost/thread/thread.hpp>

#include <roah_rsbb.h>

using namespace std;
using namespace roah_rsbb;

const uint16_t port = 6666;
const string host = "localhost";
const string TEAM_NAME = "UrsusTeam";
const string ROBOT_NAME = "Ursus";
const string CRYPTO_KEY = "randomkey";
const string CRYPTO_CIPHER = "aes-128-cbc";

/**
       \brief
       @author authorname
*/

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
public slots:
 	void compute(); 
	void doTheGuess();
	
private:
	shared_ptr<PublicChannel<>> public_channel_;
	QMutex private_channel_mutex_;
	shared_ptr<PrivateChannel<>> private_channel_;

	static void receive_benchmark_state (boost::asio::ip::udp::endpoint& endpoint, uint16_t comp_id, uint16_t msg_type, shared_ptr<const roah_rsbb_msgs::BenchmarkState> msg);
	static void receive_robot_state (boost::asio::ip::udp::endpoint& endpoint, uint16_t comp_id, uint16_t msg_type, shared_ptr<const roah_rsbb_msgs::RobotState> msg);
	void receive_rsbb_beacon (boost::asio::ip::udp::endpoint& endpoint, uint16_t comp_id, uint16_t msg_type, shared_ptr<const roah_rsbb_msgs::RoahRsbbBeacon> rsbb_beacon);
	static void receive_robot_beacon (boost::asio::ip::udp::endpoint& endpoint, uint16_t comp_id, uint16_t msg_type, shared_ptr<const roah_rsbb_msgs::RobotBeacon> msg);
	static void receive_tablet_beacon (boost::asio::ip::udp::endpoint& endpoint, uint16_t comp_id, uint16_t msg_type, shared_ptr<const roah_rsbb_msgs::TabletBeacon> msg);

};

#endif