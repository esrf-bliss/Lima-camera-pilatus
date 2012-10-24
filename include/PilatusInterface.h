#ifndef PILATUSINTERFACE_H
#define PILATUSINTERFACE_H

#include "HwInterface.h"
#include "HwFileEventMgr.h"
#include "Debug.h"
#include "PilatusCamera.h"

namespace lima
{
namespace Pilatus
{
class Interface;

/*******************************************************************
 * \class DetInfoCtrlObj
 * \brief Control object providing Pilatus detector info interface
 *******************************************************************/

class DetInfoCtrlObj: public HwDetInfoCtrlObj
{
DEB_CLASS_NAMESPC(DebModCamera, "DetInfoCtrlObj", "Pilatus");

public:
	struct Info
	{
	  Size		m_det_size;
	  std::string 	m_det_model;
	};
	DetInfoCtrlObj(const Info* = NULL);
	virtual ~DetInfoCtrlObj();

	virtual void getMaxImageSize(Size& max_image_size);
	virtual void getDetectorImageSize(Size& det_image_size);

	virtual void getDefImageType(ImageType& def_image_type);
	virtual void getCurrImageType(ImageType& curr_image_type);
	virtual void setCurrImageType(ImageType curr_image_type);

	virtual void getPixelSize(double& x_size,double &y_size);
	virtual void getDetectorType(std::string& det_type);
	virtual void getDetectorModel(std::string& det_model);

	virtual void registerMaxImageSizeCallback(HwMaxImageSizeCallback&)
	{
		;
	}
	;
	virtual void unregisterMaxImageSizeCallback(HwMaxImageSizeCallback&)
	{
		;
	}
	;

	double getMinLatTime() const {return 3e-3;}
private:
	Info	m_info;
};
/*******************************************************************
 * \class SyncCtrlObj
 * \brief Control object providing Pilatus synchronization interface
 *******************************************************************/

class SyncCtrlObj: public HwSyncCtrlObj
{
DEB_CLASS_NAMESPC(DebModCamera, "SyncCtrlObj", "Pilatus");

public:

 SyncCtrlObj(Camera& cam,DetInfoCtrlObj&);
	virtual ~SyncCtrlObj();

	virtual bool checkTrigMode(TrigMode trig_mode);
	virtual void setTrigMode(TrigMode trig_mode);
	virtual void getTrigMode(TrigMode& trig_mode);

	virtual void setExpTime(double exp_time);
	virtual void getExpTime(double& exp_time);

	virtual void setLatTime(double lat_time);
	virtual void getLatTime(double& lat_time);

	virtual void setNbHwFrames(int nb_frames);
	virtual void getNbHwFrames(int& nb_frames);

	virtual void getValidRanges(ValidRangesType& valid_ranges);

	void prepareAcq();
	
private:
	Camera& m_cam;
	int m_nb_frames;
	double m_exposure_requested;
	double m_latency;
};

/*******************************************************************
 * \class Interface
 * \brief Pilatus hardware interface
 *******************************************************************/

class Interface: public HwInterface
{
DEB_CLASS_NAMESPC(DebModCamera, "PilatusInterface", "Pilatus");

public:
	Interface(Camera& cam,const DetInfoCtrlObj::Info* = NULL);
	virtual ~Interface();

	//- From HwInterface
	virtual void getCapList(CapList&) const;
	virtual void reset(ResetLevel reset_level);
	virtual void prepareAcq();
	virtual void startAcq();
	virtual void stopAcq();
	virtual void getStatus(StatusType& status);
	virtual int getNbHwAcquiredFrames();

	void setEnergy(double energy);
	double getEnergy(void);
	void setMxSettings(const std::string& str);
	void setThresholdGain(int threshold, Camera::Gain gain);
	int getThreshold(void);
	Camera::Gain getGain(void);
	void sendAnyCommand(const std::string& str);

private:
	class _BufferCallback;
	friend class _BufferCallback;

	Camera& m_cam;
	CapList m_cap_list;
	DetInfoCtrlObj m_det_info;
	_BufferCallback* m_buffer_cbk;
	HwTmpfsBufferMgr m_buffer;
	SyncCtrlObj m_sync;
	double m_latency;
};

} // namespace Pilatus
} // namespace lima

#endif // PILATUSINTERFACE_H
