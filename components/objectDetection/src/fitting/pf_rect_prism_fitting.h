#ifndef RECTPRISMFITTING_H
#define RECTPRISMFITTING_H
#include "rect_prism_cloud_particle.h"
#include "fitting.h"

typedef pcl::PointXYZRGB PointT;

class PfRectPrismFitting: public fitting
{
	// Define callback signature typedefs
	typedef void (sig_cb_fitting_addapt) (const boost::shared_ptr<RectPrism>&);

	boost::thread captured_thread;
	mutable boost::mutex capture_mutex;
	boost::signals2::signal<sig_cb_fitting_addapt>* fitting_signal;

public:
	PfRectPrismFitting( int numparticles, pcl::PointCloud<PointT>::Ptr cloudToFit);
	~PfRectPrismFitting();
	void sig_term();
	void captureThreadFunction();
	void setPointCloud(  pcl::PointCloud<PointT>::Ptr cloud) { cloud2Fit=*cloud; }
	float getRandom(float var);
	inline double getRandom() { return (double(rand())/RAND_MAX); }
	inline  boost::shared_ptr<RectPrism> getBest() { boost::shared_ptr<RectPrism> rp(new RectPrism());
		*rp=bestParticle.getRectPrism(); 
		return rp; }
		
	inline RectPrismCloudParticle getBestParticle() { return bestParticle; }

private:
	RectPrismCloudPFInputData input;
	pcl::PointCloud<PointT> cloud2Fit;
	RCParticleFilter<RectPrismCloudPFInputData, int, RectPrismCloudParticle, RCParticleFilter_Config> *pf;

	RCParticleFilter_Config c;
	pcl::PointCloud<PointT>::Ptr cloud;
	RectPrismCloudParticle bestParticle;
};

#endif