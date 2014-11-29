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

/**
* \brief Default constructor
*/


SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
// 	public_channel_ = shared_ptr<PublicChannel<>>(new PublicChannel<>(host, port));
// 	private_channel_ = shared_ptr<PrivateChannel<>>();
// 	
// // 	cout << "Connected public channel to " << host << ":" << port << endl << flush;
// 	public_channel_->signal_rsbb_beacon_received().connect (boost::bind (&SpecificWorker::receive_rsbb_beacon, this, _1, _2, _3, _4));
// 	public_channel_->signal_robot_beacon_received().connect (&SpecificWorker::receive_robot_beacon);
// 	public_channel_->signal_tablet_beacon_received().connect (&SpecificWorker::receive_tablet_beacon);

	connect(doTheGuessButton, SIGNAL(clicked()), this, SLOT(doTheGuess()));
	objectdetection_proxy->reloadVFH();
	objectdetection_proxy->loadTrainedVFH();
	runs = 0;
	
	lineEdit->setText("1417199521.png");
	lineEdit_2->setText("1417199518.pcd");

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

void SpecificWorker::compute( )
{
// 	static boost::posix_time::ptime loop_time = boost::posix_time::microsec_clock::universal_time();
// 	shared_ptr<PrivateChannel<CerrErrorHandler>> private_channel;
// 	{
// 		QMutexLocker lock(&private_channel_mutex_);
// 		private_channel = private_channel_;
// 	}
// 	if (private_channel)
// 	{
// 		roah_rsbb_msgs::RobotState msg;
// 		roah_rsbb::now (msg.mutable_time());
// 		msg.set_messages_saved (0);
// // 		cout << "Sending RobotState" << endl << flush;
// 		private_channel->send (msg);
// 	}
// 	else
// 	{
// 		roah_rsbb_msgs::RobotBeacon msg;
// 		msg.set_team_name (TEAM_NAME);
// 		msg.set_robot_name (ROBOT_NAME);
// 		roah_rsbb::now (msg.mutable_time());
// // 		cout << "Sending RobotBeacon" << endl << flush;
// 		public_channel_->send (msg);
// 	}
// 
// 	loop_time += boost::posix_time::milliseconds(1000);
// 	boost::this_thread::sleep(loop_time);
}

void SpecificWorker::doTheGuess()
{
	QString image=lineEdit->text();
	QString pcd=lineEdit_2->text();
	cout << objectdetection_proxy->getResult(image.toStdString(), pcd.toStdString()) <<endl;

//   objectdetection_proxy->grabTheAR();
// 	objectdetection_proxy->aprilFitModel("table");
// 	objectdetection_proxy->passThrough();
// 	objectdetection_proxy->ransac("table");
// 	objectdetection_proxy->getInliers("table");
// 	objectdetection_proxy->projectInliers("table");
// 	objectdetection_proxy->convexHull("table");
// 	objectdetection_proxy->extractPolygon("table");
	//vfh guess
// 	std::vector<string> guesses;
// 	objectdetection_proxy->vfh(guesses);
// 	float x, y, theta;
// 	objectdetection_proxy->centroidBasedPose(x, y, theta);
// 	objectdetection_proxy->passThrough();
// 	objectdetection_proxy->ransac("table");
// 	objectdetection_proxy->getInliers("table");
// 	objectdetection_proxy->projectInliers("table");
// 	objectdetection_proxy->convexHull("table");
// 	objectdetection_proxy->extractPolygon("table");
// 	
// 	//show vfh results
// 	std::vector<string> guesses;
// 	objectdetection_proxy->vfh(guesses);
// 	QStringList pieces;
// 	QString path_to_pcd, name_of_object;
// 	path_to_pcd = QString::fromStdString(guesses[0]);
// 	string instance_code;
// 	pieces = path_to_pcd.split( "/" );
// 	name_of_object = pieces.at( pieces.length() - 2 );
// 
// 	
/*	
	QStringList pieces;
	QString path_to_pcd, name_of_object;
	path_to_pcd = QString::fromStdString(guesses[0]);
	string instance_code;
	pieces = path_to_pcd.split( "/" );
	name_of_object = pieces.at( pieces.length() - 2 );
	string instance = name_of_object.toStdString();
	
	std::cout<<"object_class: a"<<std::endl;
	std::cout<<"object_name: "<<instance <<std::endl;
	std::cout<<"  object_pose: "<<std::endl;
	std::cout<<"    x: "<<x<<std::endl;
	std::cout<<"    y: "<<y<<std::endl;
	std::cout<<"    theta: "<<theta<<std::endl;
	
	timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	string filename = "/home/spyke/robocomp/components/perception/components/objectDetectionStatic/datalogged/" + QString::number(ts.tv_sec).toStdString()+ ".txt";
	
	std::ofstream myfile;
  myfile.open (filename);
  myfile << "object_class: a \n";
	myfile<<"object_name: "<< instance <<"\n";
	myfile<<"  object_pose: \n";
	myfile<<"    x: "<<x<<" \n";
	myfile<<"    y: "<<y<<"\n";
	myfile<<"    theta: "<<theta<<"\n";
  myfile.close();
	
	runs++;
	if(runs == 10)
		system(" cp /home/spyke/robocomp/components/perception/components/objectDetectionStatic/datalogged/* /media/spyke/0C1E-1130/Ursus/FBM1/Round2/");*/
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
}

void SpecificWorker::receive_benchmark_state(boost::asio::ip::udp::endpoint& endpoint, uint16_t comp_id, uint16_t msg_type, shared_ptr<const roah_rsbb_msgs::BenchmarkState> msg)
{
// 	cout << "Received BenchmarkState from " << endpoint.address().to_string() << ":" << endpoint.port() << ", COMP_ID " << comp_id;
// 	cout << ", MSG_TYPE " << msg_type << endl;
// 	cout << "  benchmark_type: " << msg->benchmark_type() << endl;
// 	cout << "  benchmark_state: " << msg->benchmark_state() << endl;
// 	cout << flush;
}

void SpecificWorker::receive_robot_state(boost::asio::ip::udp::endpoint& endpoint, uint16_t comp_id, uint16_t msg_type, shared_ptr<const roah_rsbb_msgs::RobotState> msg) 
{
// 	cout << "Received RobotState from " << endpoint.address().to_string() << ":" << endpoint.port() << ", COMP_ID " << comp_id;
// 	cout << ", MSG_TYPE " << msg_type << endl;
// 	cout << "  time: " << msg->time().sec() << "." << msg->time().nsec() << endl;
// 	cout << "  messages_saved: " << msg->messages_saved() << endl;
// 	cout << flush;
}

void SpecificWorker::receive_rsbb_beacon(boost::asio::ip::udp::endpoint& endpoint, uint16_t comp_id, uint16_t msg_type, shared_ptr<const roah_rsbb_msgs::RoahRsbbBeacon> rsbb_beacon)
{
// 	cout << "Received RoahRsbbBeacon from " << endpoint.address().to_string() << ":" << endpoint.port() << ", COMP_ID " << comp_id << ", MSG_TYPE " << msg_type << endl;
// 
// 	unsigned short connect_port = 0;
// 	for (auto const& bt : rsbb_beacon->benchmarking_teams())
// 	{
// 		cout << "  team_name: " << bt.team_name() << ", robot_name: " << bt.robot_name() << ", rsbb_port: " << bt.rsbb_port() << endl;
// 		if ( (bt.team_name() == TEAM_NAME) && (bt.robot_name() == ROBOT_NAME))
// 		{
// 			connect_port = bt.rsbb_port();
// 			// break; // Commented to show all entries
// 		}
// 	}
// 
// 	QMutexLocker lock(&private_channel_mutex_);
// 	if (connect_port != (private_channel_ ? private_channel_->port() : 0))
// 	{
// 		if (private_channel_)
// 		{
// 			cout << "Disconnecting private channel" << endl;
// 			private_channel_.reset();
// 		}
// 		if (connect_port)
// 		{
// 			cout << "Connecting private channel to " << endpoint.address().to_string() << ":" << connect_port << endl;
// 			private_channel_ = make_shared<PrivateChannel<CerrErrorHandler>> (endpoint.address().to_string(), connect_port, CRYPTO_KEY, CRYPTO_CIPHER);
// 			private_channel_->signal_benchmark_state_received().connect (&SpecificWorker::receive_benchmark_state);
// 			private_channel_->signal_robot_state_received().connect (&SpecificWorker::receive_robot_state);
// 		}
// 	}
// 	cout << flush;
}

void SpecificWorker::receive_robot_beacon(boost::asio::ip::udp::endpoint& endpoint, uint16_t comp_id, uint16_t msg_type, shared_ptr<const roah_rsbb_msgs::RobotBeacon> msg)
{
// 	cout << "Received RobotBeacon from " << endpoint.address().to_string() << ":" << endpoint.port() << ", COMP_ID " << comp_id;
// 	cout << ", MSG_TYPE " << msg_type << endl;
// 	cout << "  team_name: " << msg->team_name() << endl;
// 	cout << "  robot_name: " << msg->robot_name() << endl;
// 	cout << "  time: " << msg->time().sec() << "." << msg->time().nsec() << endl;
// 	cout << flush;
}

void SpecificWorker::receive_tablet_beacon(boost::asio::ip::udp::endpoint& endpoint, uint16_t comp_id, uint16_t msg_type, shared_ptr<const roah_rsbb_msgs::TabletBeacon> msg)
{
// 	cout << "Received TabletBeacon from " << endpoint.address().to_string() << ":" << endpoint.port() << ", COMP_ID " << comp_id;
// 	cout << ", MSG_TYPE " << msg_type << endl;
// 	cout << "  last_call: " << msg->last_call().sec() << "." << msg->last_call().nsec() << endl;
// 	cout << "  last_pos: " << msg->last_pos().sec() << "." << msg->last_pos().nsec() << endl;
// 	cout << "  x: " << msg->x() << endl;
// 	cout << "  y: " << msg->y() << endl;
// 	cout << flush;
}


