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
/** \mainpage RoboComp::genericComp
 *
 * \section intro_sec Introduction
 *
 * The genericComp component...
 *
 * \section interface_sec Interface
 *
 * genericComp interface...
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * genericComp ...
 *
 * \subsection install2_ssec Compile and install
 * cd genericComp
 * <br>
 * cmake . && make
 * <br>
 * To install:
 * <br>
 * sudo make install
 *
 * \section guide_sec User guide
 *
 * \subsection config_ssec Configuration file
 *
 * <p>
 * The configuration file genericComp/etc/specific_config and genericComp/etc/generic_config...
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/genericComp --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */
// QT includes
#include <QtCore>
#include <QtGui>

// ICE includes
#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>
#include <Ice/Application.h>

#include <rapplication/rapplication.h>
#include <qlog/qlog.h>
// View the config.h file for config options like
// QtGui, etc...
#include "config.h"
#include "genericmonitor.h"
#include "genericworker.h"
#include "specificworker.h"
#include "specificmonitor.h"
#include "commonbehaviorI.h"
#include <agmcommonbehaviorI.h>
#include <agmexecutivetopicI.h>
#include <apriltagsI.h>

// Includes for remote proxy example
// #include <Remote.h>
#include <ui_guiDlg.h>
#include <AGMAgent.h>


// User includes here

// Namespaces
using namespace std;
using namespace RoboCompCommonBehavior;
using namespace RoboCompAGMCommonBehavior;
using namespace RoboCompAGMExecutive;
using namespace RoboCompAprilTags;
using namespace RoboCompAGMAgent;


class objectdetectorAgentComp : public RoboComp::Application
{
private:
	// User private data here

	void initialize();
	MapPrx mprx;

public:
	virtual int run(int, char*[]);
};

void objectdetectorAgentComp::initialize()
{
	// Config file properties read example
	// configGetString( PROPERTY_NAME_1, property1_holder, PROPERTY_1_DEFAULT_VALUE );
	// configGetInt( PROPERTY_NAME_2, property1_holder, PROPERTY_2_DEFAULT_VALUE );
}
int objectdetectorAgentComp::run(int argc, char* argv[])
{
#ifdef USE_QTGUI
	QApplication a(argc, argv);  // GUI application
#else
	QCoreApplication a(argc, argv);  // NON-GUI application
#endif
	int status=EXIT_SUCCESS;

	// Remote server proxy access example
	// RemoteComponentPrx remotecomponent_proxy;
	AGMAgentTopicPrx agmagenttopic_proxy;


	string proxy;

	// User variables


	initialize();

	// Remote server proxy creation example
	// try
	// {
	// 	// Load the remote server proxy
	//	proxy = getProxyString("RemoteProxy");
	//	remotecomponent_proxy = RemotePrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
	//	if( !remotecomponent_proxy )
	//	{
	//		rInfo(QString("Error loading proxy!"));
	//		return EXIT_FAILURE;
	//	}
	//catch(const Ice::Exception& ex)
	//{
	//	cout << "[" << PROGRAM_NAME << "]: Exception: " << ex << endl;
	//	return EXIT_FAILURE;
	//}
	//rInfo("RemoteProxy initialized Ok!");
	// 	// Now you can use remote server proxy (remotecomponent_proxy) as local object
	
	IceStorm::TopicManagerPrx topicManager = IceStorm::TopicManagerPrx::checkedCast(communicator()->propertyToProxy("TopicManager.Proxy"));
	
	IceStorm::TopicPrx agmagenttopic_topic;
    while(!agmagenttopic_topic){
		try {
			agmagenttopic_topic = topicManager->retrieve("AGMAgentTopic");
		}catch (const IceStorm::NoSuchTopic&){
			try{
				agmagenttopic_topic = topicManager->create("AGMAgentTopic");
			}catch (const IceStorm::TopicExists&){
				// Another client created the topic.
			}
		}
	}
	Ice::ObjectPrx agmagenttopic_pub = agmagenttopic_topic->getPublisher()->ice_oneway();
	AGMAgentTopicPrx agmagenttopic = AGMAgentTopicPrx::uncheckedCast(agmagenttopic_pub);
	mprx["AGMAgentTopicPub"] = (::IceProxy::Ice::Object*)(&agmagenttopic);
	
	
	GenericWorker *worker = new SpecificWorker(mprx);
	//Monitor thread
	GenericMonitor *monitor = new SpecificMonitor(worker,communicator());
	QObject::connect(monitor,SIGNAL(kill()),&a,SLOT(quit()));
	QObject::connect(worker,SIGNAL(kill()),&a,SLOT(quit()));
	monitor->start();
	
	if ( !monitor->isRunning() )
		return status;
	try
	{
		// Server adapter creation and publication
		Ice::ObjectAdapterPtr adapterCommonBehavior = communicator()->createObjectAdapter("CommonBehavior");
		CommonBehaviorI *commonbehaviorI = new CommonBehaviorI(monitor );
		adapterCommonBehavior->add(commonbehaviorI, communicator()->stringToIdentity("commonbehavior"));
		adapterCommonBehavior->activate();
		// Server adapter creation and publication
    	Ice::ObjectAdapterPtr AGMExecutiveTopic_adapter = communicator()->createObjectAdapter("AGMExecutiveTopicTopic");
    	AGMExecutiveTopicPtr agmexecutivetopicI_ = new AGMExecutiveTopicI(worker);
    	Ice::ObjectPrx agmexecutivetopic_proxy = AGMExecutiveTopic_adapter->addWithUUID(agmexecutivetopicI_)->ice_oneway();
    	IceStorm::TopicPrx agmexecutivetopic_topic;
    	while(!agmexecutivetopic_topic){
	    	try {
	    		agmexecutivetopic_topic = topicManager->retrieve("AGMExecutiveTopic");
	    	 	IceStorm::QoS qos;
	      		agmexecutivetopic_topic->subscribeAndGetPublisher(qos, agmexecutivetopic_proxy);
	    	}
	    	catch (const IceStorm::NoSuchTopic&) {
	       		// Error! No topic found!
	    	}
    	}
    	AGMExecutiveTopic_adapter->activate();
    	// Server adapter creation and publication
    	Ice::ObjectAdapterPtr AprilTags_adapter = communicator()->createObjectAdapter("AprilTagsTopic");
    	AprilTagsPtr apriltagsI_ = new AprilTagsI(worker);
    	Ice::ObjectPrx apriltags_proxy = AprilTags_adapter->addWithUUID(apriltagsI_)->ice_oneway();
    	IceStorm::TopicPrx apriltags_topic;
    	while(!apriltags_topic){
	    	try {
	    		apriltags_topic = topicManager->retrieve("AprilTags");
	    	 	IceStorm::QoS qos;
	      		apriltags_topic->subscribeAndGetPublisher(qos, apriltags_proxy);
	    	}
	    	catch (const IceStorm::NoSuchTopic&) {
	       		// Error! No topic found!
	    	}
    	}
    	AprilTags_adapter->activate();
    	// Server adapter creation and publication
		Ice::ObjectAdapterPtr adapterAGMCommonBehavior = communicator()->createObjectAdapter("AGMCommonBehaviorComp");
		AGMCommonBehaviorI *agmcommonbehavior = new AGMCommonBehaviorI(worker);
		adapterAGMCommonBehavior->add(agmcommonbehavior, communicator()->stringToIdentity("agmcommonbehavior"));

		adapterAGMCommonBehavior->activate();
		cout << SERVER_FULL_NAME " started" << endl;

		// User defined QtGui elements ( main window, dialogs, etc )

#ifdef USE_QTGUI
		//ignoreInterrupt(); // Uncomment if you want the component to ignore console SIGINT signal (ctrl+c).
		a.setQuitOnLastWindowClosed( true );
#endif
		// Run QT Application Event Loop
		a.exec();
		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;

		cout << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << endl;
		cout << ex;

#ifdef USE_QTGUI
		a.quit();
#endif
		monitor->exit(0);
}

	return status;
}

int main(int argc, char* argv[])
{
	bool hasConfig = false;
	string arg;
	objectdetectorAgentComp app;

	// Search in argument list for --Ice.Config= argument
	for (int i = 1; i < argc; ++i)
	{
		arg = argv[i];
		if ( arg.find ( "--Ice.Config=", 0 ) != string::npos )
			hasConfig = true;
	}

	if ( hasConfig )
		return app.main( argc, argv );
	else
		return app.main(argc, argv, "../etc/generic_config"); // "config" is the default config file name
}
