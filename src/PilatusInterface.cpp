//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2011
// European Synchrotron Radiation Facility
// BP 220, Grenoble 38043
// FRANCE
//
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//###########################################################################
#include <algorithm>
#include <fcntl.h>
#include <pwd.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <unistd.h>
#include <errno.h>
#include "lima/Debug.h"
#include "PilatusInterface.h"

using namespace lima;
using namespace lima::Pilatus;

static const char CAMERA_NAME_TOKEN[] = "camera_name";
static const char CAMERA_WIDE_TOKEN[] = "camera_wide";
static const char CAMERA_HIGH_TOKEN[] = "camera_high";
static const char CAMERA_S_SERIE_TOKEN[] = "camera_s_serie";

static const char CAMERA_PILATUS3_TOKEN[] = "PILATUS3";

static const char FILE_PATTERN[] = "tmp_img_%.7d.edf";
static const int  DECTRIS_EDF_OFFSET = 1024;

static const float P2_6M_MAX_FREQUENCY[3] = {12, 40, 100}; //full, c18, c2
static const float P3_6M_MAX_FREQUENCY[3] = {100, 200, 500}; //full, c18, c2




/*******************************************************************
 * \brief DetInfoCtrlObj constructor
 * \param info if info is NULL look for ~det/p2_det/config/cam_data/camera.def file
 *******************************************************************/
DetInfoCtrlObj::DetInfoCtrlObj(Camera& cam,const DetInfoCtrlObj::Info* info):
  m_cam(cam),
  m_is_s_serie(false)
{
    DEB_CONSTRUCTOR();
    if(info)
      m_info = *info;
    else			// look for local file
      {
	char aBuffer[2048];
	struct passwd aPwd;
	struct passwd *aResultPwd;

	const char *config_file = m_cam.configFile();
	
	FILE* aConfFile = fopen(config_file,"r");
	if(!aConfFile)
	  THROW_HW_ERROR(Error) << "Can't open config file :"
				<< config_file;
	char aReadBuffer[1024];
	int aWidth = -1,aHeight = -1;
	while(fgets(aReadBuffer,sizeof(aReadBuffer),aConfFile))
	  {
	    if(!strncmp(aReadBuffer,
			CAMERA_NAME_TOKEN,sizeof(CAMERA_NAME_TOKEN) - 1))
	      {
		char *aBeginPt = strchr(aReadBuffer,(unsigned int)'"');
		++aBeginPt;
		char *aEndPt = strrchr(aBeginPt,(unsigned int)'"');
		*aEndPt = '\0';	// remove last "
		m_info.m_det_model = aBeginPt;
		//Check if pilatus2 or 3
		m_is_pilatus3 = !strncmp(aBeginPt,CAMERA_PILATUS3_TOKEN,sizeof(CAMERA_PILATUS3_TOKEN) - 1);
		m_is_pilatus2 = !m_is_pilatus3;
                DEB_PARAM() << DEB_VAR1(m_is_pilatus2);
                DEB_PARAM() << DEB_VAR1(m_is_pilatus3);

	      }
	    else if(!strncmp(aReadBuffer,
			     CAMERA_HIGH_TOKEN,sizeof(CAMERA_HIGH_TOKEN) - 1))
	      {
		char *aPt = aReadBuffer;
		while(*aPt && (*aPt < '1' || *aPt > '9')) ++aPt;
		aHeight = atoi(aPt);
	      }
	    else if(!strncmp(aReadBuffer,
			     CAMERA_WIDE_TOKEN,sizeof(CAMERA_WIDE_TOKEN) - 1))
	      {
		char *aPt = aReadBuffer;
		while(*aPt && (*aPt < '1' || *aPt > '9')) ++aPt;
		aWidth = atoi(aPt);
	      }
	    else if(!strncmp(aReadBuffer,
			     CAMERA_S_SERIE_TOKEN,sizeof(CAMERA_S_SERIE_TOKEN) - 1))
	      {
		m_is_s_serie = true;
	      }
	    
	  }
	if(aWidth <= 0 || aHeight <= 0)
	  {
	    fclose(aConfFile);
	    THROW_HW_ERROR(Error) << "Can't get detector info";
	  }
	m_info.m_det_size = Size(aWidth,aHeight);
	if (m_is_s_serie )
	  m_info.m_det_model += ", S serie";
	else
	  m_info.m_det_model += ", X serie";

	fclose(aConfFile);
      }
}

//-----------------------------------------------------
//
//-----------------------------------------------------
DetInfoCtrlObj::~DetInfoCtrlObj()
{
    DEB_DESTRUCTOR();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void DetInfoCtrlObj::getMaxImageSize(Size& size)
{
    DEB_MEMBER_FUNCT();
    // get the max image size
    getDetectorImageSize(size);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void DetInfoCtrlObj::getDetectorImageSize(Size& size)
{
    DEB_MEMBER_FUNCT();
    // get the max image size of the detector
    size = m_info.m_det_size;
}


//-----------------------------------------------------
//
//-----------------------------------------------------
void DetInfoCtrlObj::getDefImageType(ImageType& image_type)
{
    DEB_MEMBER_FUNCT();
    getCurrImageType(image_type);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void DetInfoCtrlObj::getCurrImageType(ImageType& image_type)
{
    DEB_MEMBER_FUNCT();
    image_type= Bpp32S;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void DetInfoCtrlObj::setCurrImageType(ImageType image_type)
{
    DEB_MEMBER_FUNCT();
    ImageType valid_image_type;
    getDefImageType(valid_image_type);
    if (image_type != valid_image_type)
        throw LIMA_HW_EXC(InvalidValue, "Invalid Pixel depth value");
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void DetInfoCtrlObj::getPixelSize(double& x_size,double& y_size)
{
    DEB_MEMBER_FUNCT();
    x_size = y_size = 172.0e-6;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void DetInfoCtrlObj::getDetectorType(std::string& type)
{
    DEB_MEMBER_FUNCT();
    type  = "Pilatus";

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void DetInfoCtrlObj::getDetectorModel(std::string& model)
{
    DEB_MEMBER_FUNCT();
    model = m_info.m_det_model;
}
//-----------------------------------------------------
//
//-----------------------------------------------------
double DetInfoCtrlObj::getMinLatTime() const
{
  return (m_is_pilatus3 && !m_is_s_serie) ? 950e-6 : 3e-3;
}

/*******************************************************************
 * \brief SyncCtrlObj constructor
 *******************************************************************/

SyncCtrlObj::SyncCtrlObj(Camera& cam,
			 DetInfoCtrlObj &det_info,
			 RoiCtrlObj &roi) :
  m_cam(cam),
  m_det_info(det_info),
  m_roi(roi),
  m_latency(det_info.getMinLatTime())
{
}

//-----------------------------------------------------
//
//-----------------------------------------------------
SyncCtrlObj::~SyncCtrlObj()
{
}

//-----------------------------------------------------
//
//-----------------------------------------------------
bool SyncCtrlObj::checkTrigMode(TrigMode trig_mode)
{
    bool valid_mode = false;
    switch (trig_mode)
    {
    case IntTrig:
    case IntTrigMult:
    case ExtTrigSingle:
    case ExtTrigMult:
    case ExtGate:
        valid_mode = true;
        break;

    default:
        valid_mode = false;
        break;
    }
    return valid_mode;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::setTrigMode(TrigMode trig_mode)
{
    DEB_MEMBER_FUNCT();
    if (!checkTrigMode(trig_mode))
        throw LIMA_HW_EXC(InvalidValue, "Invalid trigger mode");
    Camera::TriggerMode trig;
    switch(trig_mode)
    {
        case IntTrig        : trig = Camera::INTERNAL_SINGLE;
        break;
        case IntTrigMult    : trig = Camera::INTERNAL_MULTI;
        break;
        case ExtTrigSingle  : trig = Camera::EXTERNAL_SINGLE;
        break;
        case ExtTrigMult    : trig = Camera::EXTERNAL_MULTI;
        break;
        case ExtGate        : trig = Camera::EXTERNAL_GATE;
        break;
	default: break;
    }

    m_cam.setTriggerMode(trig);

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::getTrigMode(TrigMode& trig_mode)
{
    Camera::TriggerMode trig = m_cam.triggerMode();
    switch(trig)
    {
        case Camera::INTERNAL_SINGLE    :   trig_mode = IntTrig;
        break;
        case Camera::INTERNAL_MULTI     :   trig_mode = IntTrigMult;
        break;
        case Camera::EXTERNAL_SINGLE    :   trig_mode = ExtTrigSingle;
        break;
        case Camera::EXTERNAL_MULTI     :   trig_mode = ExtTrigMult;
        break;
        case Camera::EXTERNAL_GATE      :   trig_mode = ExtGate;
        break;
    }
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::setExpTime(double exp_time)
{
    m_exposure_requested = exp_time;
    m_cam.setExposure(exp_time);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::getExpTime(double& exp_time)
{
    exp_time = m_cam.exposure();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::setLatTime(double lat_time)
{
   m_latency = lat_time;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::getLatTime(double& lat_time)
{
    lat_time = m_latency;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::setNbHwFrames(int nb_frames)
{
    DEB_MEMBER_FUNCT();
    m_nb_frames = nb_frames;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::getNbHwFrames(int& nb_frames)
{
    nb_frames =  m_nb_frames;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::getValidRanges(ValidRangesType& valid_ranges)
{
    double min_time = 1e-9;
    double max_time = 1e6;
    valid_ranges.min_exp_time = min_time;
    valid_ranges.max_exp_time = max_time;
    valid_ranges.min_lat_time = m_latency;
    valid_ranges.max_lat_time = max_time;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void SyncCtrlObj::prepareAcq()
{
    DEB_MEMBER_FUNCT();
    
    TrigMode trig_mode;
    getTrigMode(trig_mode);

    double exposure =  m_exposure_requested;
    double exposure_period = exposure + m_latency;

    if(m_det_info.isPilatus3() && 
       (trig_mode == ExtGate || trig_mode == ExtTrigMult))
      {
	int max_frequency = m_roi.getMaxFrequency();
	if(max_frequency > 0)
	  {
	    double min_exposure_period = 1./max_frequency;
	    if(exposure_period < min_exposure_period)
	      exposure_period = min_exposure_period;
	  }
      }
    m_cam.setExposurePeriod(exposure_period);
	
    int nb_frames;
    // For IntTrigMult only one frame must be set and startAcq() will retrig a single frame
    // This is the only trigger mode which can run the for ever, For the other modes we must
    // workaround the number of frames to the maximum value than camserver can accept, this
    // is risky !!

    if (trig_mode == IntTrig && m_nb_frames > 65535)
      THROW_HW_ERROR(InvalidValue) << "In IntTrig trigger mode, maximum number of frames is 65535";

    if (trig_mode == IntTrigMult)
      nb_frames = 1;
    else if (m_nb_frames == 0)
      nb_frames = 65535;
    else
	nb_frames = m_nb_frames;
    
    m_cam.setNbImagesInSequence(nb_frames);

}
/*****************************************************************************
				 ROI
*****************************************************************************/
static const int MODULE_WIDTH = 487;
static const int MODULE_HEIGHT = 195;

static const int MODULE_WIDTH_SPACE = 7;
static const int MODULE_HEIGHT_SPACE = 17;

RoiCtrlObj::RoiCtrlObj(Camera& cam,DetInfoCtrlObj& det) :
  m_cam(cam),
  m_det(det),
  m_has_hardware_roi(true)
{
  DEB_CONSTRUCTOR();
  Size detImageSize;
  det.getDetectorImageSize(detImageSize);

  int fullframe_max_frequency = -1;
  int c18_max_frequency = -1;
  int c2_max_frequency = -1;

  // S (versus X) series do not have hardware ROI capability
  // need the camera.conf file patched with keyword "camera_s_serie"
  if (det.isSSerie())
    {
      m_has_hardware_roi = false;
    }
  else if(detImageSize == Size(2463,2527)) // Pilatus 6M
    {
      if(det.isPilatus2())
	{
	  fullframe_max_frequency = P2_6M_MAX_FREQUENCY[0];
	  c18_max_frequency = P2_6M_MAX_FREQUENCY[1];
	  c2_max_frequency = P2_6M_MAX_FREQUENCY[2];
	}
      else
	{
	  fullframe_max_frequency = P3_6M_MAX_FREQUENCY[0];
	  c18_max_frequency = P3_6M_MAX_FREQUENCY[1];
	  c2_max_frequency = P3_6M_MAX_FREQUENCY[2];
	}

      DEB_PARAM() << DEB_VAR1(fullframe_max_frequency);
      DEB_PARAM() << DEB_VAR1(c18_max_frequency);
      DEB_PARAM() << DEB_VAR1(c2_max_frequency);

      Roi c2(2 * MODULE_WIDTH + 2 * MODULE_WIDTH_SPACE,
	     5 * MODULE_HEIGHT + 5 * MODULE_HEIGHT_SPACE,
	     MODULE_WIDTH,
	     2 * MODULE_HEIGHT + MODULE_HEIGHT_SPACE);
      m_possible_rois.push_back(PATTERN2ROI(Pattern("C2",c2_max_frequency),c2));

      Roi c18(MODULE_WIDTH + MODULE_WIDTH_SPACE,
	      3 * (MODULE_HEIGHT + MODULE_HEIGHT_SPACE),
	      3 * MODULE_WIDTH + 2 * MODULE_WIDTH_SPACE,
	      6 * MODULE_HEIGHT + 5 * MODULE_HEIGHT_SPACE);
      m_possible_rois.push_back(PATTERN2ROI(Pattern("C18",c18_max_frequency),c18));
    }
  else if(detImageSize == Size(1475,1679) && det.isPilatus3()) // Pilatus 2M
    {

      fullframe_max_frequency = 250;

      Roi c2(MODULE_WIDTH + MODULE_WIDTH_SPACE,
	     3 * MODULE_HEIGHT + 3 * MODULE_HEIGHT_SPACE,
	     MODULE_WIDTH,
	     2 * MODULE_HEIGHT + MODULE_HEIGHT_SPACE);
      m_possible_rois.push_back(PATTERN2ROI(Pattern("C2",500),c2));

      Roi r8(MODULE_WIDTH + MODULE_WIDTH_SPACE,
	     2 * MODULE_HEIGHT + 2 * MODULE_HEIGHT_SPACE,
	     2 * MODULE_WIDTH + MODULE_WIDTH_SPACE,
	     4 * MODULE_HEIGHT + 3 * MODULE_HEIGHT_SPACE);
      m_possible_rois.push_back(PATTERN2ROI(Pattern("R8",500),r8));

      Roi l8(0,
	     2 * MODULE_HEIGHT + 2 * MODULE_HEIGHT_SPACE,
	     2 * MODULE_WIDTH + MODULE_WIDTH_SPACE,
	     4 * MODULE_HEIGHT + 3 * MODULE_HEIGHT_SPACE);
      m_possible_rois.push_back(PATTERN2ROI(Pattern("L8",500),l8));

      Roi c12(0,
	      2 * MODULE_HEIGHT + 2 * MODULE_HEIGHT_SPACE,
	      3 * MODULE_WIDTH + 2 * MODULE_WIDTH_SPACE,
	      4 * MODULE_HEIGHT + 3 * MODULE_HEIGHT_SPACE);
      m_possible_rois.push_back(PATTERN2ROI(Pattern("C12",250),c12));
    }
  else if(detImageSize == Size(981,1043) && det.isPilatus3()) // Pilatus 1M
    {

      fullframe_max_frequency = 500;

      Roi r1(MODULE_WIDTH + MODULE_WIDTH_SPACE,
	     2 * MODULE_HEIGHT + 2 * MODULE_HEIGHT_SPACE,
	     MODULE_WIDTH,
	     MODULE_HEIGHT);
      m_possible_rois.push_back(PATTERN2ROI(Pattern("R1",500),r1));

      Roi l1(0,
	     2 * MODULE_HEIGHT + 2 * MODULE_HEIGHT_SPACE,
	     MODULE_WIDTH,
	     MODULE_HEIGHT);
      m_possible_rois.push_back(PATTERN2ROI(Pattern("L1",500),l1));

      Roi r3(MODULE_WIDTH + MODULE_WIDTH_SPACE,
	     MODULE_HEIGHT + MODULE_HEIGHT_SPACE,
	     MODULE_WIDTH,
	     3 * MODULE_HEIGHT + 2 * MODULE_HEIGHT_SPACE);
      m_possible_rois.push_back(PATTERN2ROI(Pattern("R3",500),r3));

      Roi l3(0,
	     MODULE_HEIGHT + MODULE_HEIGHT_SPACE,
	     MODULE_WIDTH,
	     3 * MODULE_HEIGHT + 2 * MODULE_HEIGHT_SPACE);
      m_possible_rois.push_back(PATTERN2ROI(Pattern("L3",500),l3));
    }
  
  if(!m_has_hardware_roi)
    DEB_WARNING() << "Hardware Roi not managed for this detector";

  Roi full(Point(0,0),detImageSize);
  m_possible_rois.push_back(PATTERN2ROI(Pattern("0",fullframe_max_frequency),full));
  m_current_roi = full;
  m_current_max_frequency = fullframe_max_frequency;
}

void RoiCtrlObj::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
  DEB_MEMBER_FUNCT();

  ROIS::const_iterator i = _getRoi(set_roi);
  if(i == m_possible_rois.end())
    THROW_HW_ERROR(Error) << "Something weird happen";

  hw_roi = i->second;
}

void RoiCtrlObj::setRoi(const Roi& set_roi)
{
  DEB_MEMBER_FUNCT();
  
  ROIS::const_iterator i;
  if(set_roi.isActive())
    {
      i = _getRoi(set_roi);
      if(i == m_possible_rois.end())
	THROW_HW_ERROR(Error) << "Something weird happen";
    }
  else
    i = --m_possible_rois.end(); // full_frame

 
  if(m_has_hardware_roi)
    m_cam.setRoi(i->first.pattern);
  
  m_current_roi = i->second;
  m_current_max_frequency = i->first.max_frequency;
}

void RoiCtrlObj::getRoi(Roi& hw_roi)
{
  hw_roi = m_current_roi;
}

inline RoiCtrlObj::ROIS::const_iterator
RoiCtrlObj::_getRoi(const Roi& roi) const
{
  for(ROIS::const_iterator i = m_possible_rois.begin();
      i != m_possible_rois.end();++i)
    {
      if(i->second.containsRoi(roi))
	return i;
    }
  return m_possible_rois.end();
}
/*****************************************************************************
			  Memory map manager
*****************************************************************************/
class _MmapManager : public HwBufferCtrlObj::Callback
{
  DEB_CLASS(DebModCamera, "Pilatus::_MmapManager");
  typedef std::pair<void*,long> AddressNSize;
  typedef std::map<void*,AddressNSize> Data2BaseNSize;
  typedef std::multiset<void *> BufferList;
public:
  _MmapManager() : HwBufferCtrlObj::Callback() {}
  virtual void map(void* address)
  {
    DEB_MEMBER_FUNCT();

    AutoMutex lock(m_mutex);
    m_buffer_in_use.insert(address);
  }
  virtual void release(void* address)
  {
    DEB_MEMBER_FUNCT();

    AutoMutex lock(m_mutex);
    BufferList::iterator it = m_buffer_in_use.find(address);
    if(it == m_buffer_in_use.end())
      THROW_HW_ERROR(Error) << "Internal error: releasing buffer not in used list";

    m_buffer_in_use.erase(it++);
    if(it == m_buffer_in_use.end() || *it != address)
      {
	Data2BaseNSize::iterator mmap_info = m_data_2_base_n_size.find(address);
	munmap(mmap_info->second.first,mmap_info->second.second);
	m_data_2_base_n_size.erase(mmap_info);
      }
  }
  virtual void releaseAll()
  {
    DEB_MEMBER_FUNCT();

    AutoMutex lock(m_mutex);
    for(Data2BaseNSize::iterator mmap_info = m_data_2_base_n_size.begin();
	mmap_info != m_data_2_base_n_size.end();++mmap_info)
      munmap(mmap_info->second.first,mmap_info->second.second);
    m_data_2_base_n_size.clear();
    m_buffer_in_use.clear();
  }

  void register_new_mmap(void *mmap_mem_base,
			 void *aDataBuffer,long length)
  {
    AutoMutex lock(m_mutex);
    m_data_2_base_n_size[aDataBuffer] = AddressNSize(mmap_mem_base,length);
  }
  
private:
  Mutex			m_mutex;
  Data2BaseNSize	m_data_2_base_n_size;
  BufferList		m_buffer_in_use;
};

/*******************************************************************
 * \brief Interface::_BufferCallback
 *******************************************************************/
class Interface::_BufferCallback : public HwTmpfsBufferMgr::Callback
{
  DEB_CLASS_NAMESPC(DebModCamera, "_BufferCallback", "Pilatus");
public:
  _BufferCallback(Interface& hwInterface) : m_interface(hwInterface) {}
  virtual ~_BufferCallback() {}
  virtual void prepare(const DirectoryEvent::Parameters &params)
  {
    DEB_MEMBER_FUNCT();

    m_interface.m_cam.setImgpath(params.watch_path);
    m_interface.m_cam.setFileName(params.file_pattern);
    m_image_events.clear();
  }

  virtual bool getFrameInfo(int image_number,const char* full_path,
			    HwFileEventCallbackHelper::CallFrom from,
			    HwFrameInfoType &frame_info)
  {
    DEB_MEMBER_FUNCT();

    FrameDim anImageDim;
    getFrameDim(anImageDim);
    long memSize = anImageDim.getMemSize();

    int fd = open(full_path,O_RDONLY);
    if(fd < 0)
      {
	if(from == HwFileEventCallbackHelper::OnDemand)
	  THROW_HW_ERROR(Error) << "Image is no more available";
	else
	  {
	    m_interface.m_cam.errorStopAcquisition();
	    THROW_HW_ERROR(Error) << strerror(errno) << DEB_VAR1(full_path);
	  }
      }
    void* mmap_mem_base = mmap(NULL,DECTRIS_EDF_OFFSET + memSize,
			  PROT_READ,MAP_SHARED,fd,0);

    close(fd);

    if(mmap_mem_base == MAP_FAILED)
      {
	m_interface.m_cam.errorStopAcquisition();
	THROW_HW_ERROR(Error) << "Problem to read image:" << DEB_VAR1(full_path);
      }

    if(from != HwFileEventCallbackHelper::OnDemand)
      m_image_events.insert(image_number);

    void* aDataBuffer = (char*)mmap_mem_base + DECTRIS_EDF_OFFSET;
    frame_info = HwFrameInfoType(image_number,aDataBuffer,&anImageDim,
				 Timestamp(),0,
				 HwFrameInfoType::Managed);
    m_mmap_manager.register_new_mmap(mmap_mem_base,
				     aDataBuffer,DECTRIS_EDF_OFFSET + memSize);
    bool aReturnFlag = true;
    if(m_interface.m_buffer.getNbOfFramePending() > 32)
      {
	m_interface.m_cam.errorStopAcquisition();
	aReturnFlag = false;
      }
    else {
      int nb_frames;
      m_interface.m_sync.getNbHwFrames(nb_frames);
      aReturnFlag = int(m_image_events.size()) != nb_frames;
    }
    return aReturnFlag;
  }
  virtual void getFrameDim(FrameDim& frame_dim)
  {
    DEB_MEMBER_FUNCT();

    Roi hw_roi;
    m_interface.m_roi.getRoi(hw_roi);
    const Size &current_size = hw_roi.getSize();
    ImageType current_image_type;
    m_interface.m_det_info.getCurrImageType(current_image_type);
    
    frame_dim.setSize(current_size);
    frame_dim.setImageType(current_image_type);
  }
  virtual HwBufferCtrlObj::Callback* getBufferCallback()
  {
    return &m_mmap_manager;
  }
private:
  Interface&	m_interface;
  _MmapManager	m_mmap_manager;
  std::set<long>		m_image_events;
};

/*******************************************************************
 * \brief Hw Interface constructor
 *******************************************************************/

Interface::Interface(Camera& cam,const DetInfoCtrlObj::Info* info)
            :   m_cam(cam),
                m_det_info(cam, info),
		m_roi(cam, m_det_info),
		m_buffer_cbk(new Interface::_BufferCallback(*this)),
                m_sync(cam,m_det_info, m_roi),
		m_saving(cam),
		m_buffer(cam.tmpFsPath(), FILE_PATTERN, *m_buffer_cbk)
{
    DEB_CONSTRUCTOR();

    HwDetInfoCtrlObj *det_info = &m_det_info;
    m_cap_list.push_back(HwCap(det_info));

    // Usually ramdisk is use to store image files saved by camserver
    // default max memory size use in HwTmpfsBufferMgr is 50% of the provided ramdisk.
    // we increase here the size to 95% of the available ramdisk.
    // The buffer will calculate how many frames (edf files) in can store in
    // this mapped disk.  For instance if the ramdisk is about 16GB and we stay at 50%
    // only 8GB can be used to store the images, With a fast and long acquisition
    // one can get an overrun error.
    m_buffer.setMemoryPercent(0.95);
    
    HwBufferCtrlObj *buffer = &m_buffer;
    m_cap_list.push_back(HwCap(buffer));

    HwSyncCtrlObj *sync = &m_sync;
    m_cap_list.push_back(HwCap(sync));

    HwSavingCtrlObj *saving = &m_saving;
    m_cap_list.push_back(HwCap(saving));

    // S (versus X) series do not have hardware ROI capability
    // need the camera.conf file patched with keyword "camera_s_serie"
    if (!m_det_info.isSSerie())
      {
	HwRoiCtrlObj *roi = &m_roi;
	m_cap_list.push_back(HwCap(roi));
      }
    
    m_buffer.getDirectoryEvent().watch_moved_to();

    //Activate new pilatus3 threshold method
    if(m_det_info.isPilatus2()) cam._pilatus2model();
    if(m_det_info.isPilatus3()) cam._pilatus3model();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
Interface::~Interface()
{
    DEB_DESTRUCTOR();
    delete m_buffer_cbk;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::getCapList(HwInterface::CapList &cap_list) const
{
    DEB_MEMBER_FUNCT();
    cap_list = m_cap_list;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::reset(ResetLevel reset_level)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(reset_level);

    stopAcq();

    Size image_size;
    m_det_info.getMaxImageSize(image_size);
    ImageType image_type;
    m_det_info.getDefImageType(image_type);
    FrameDim frame_dim(image_size, image_type);
    m_buffer.setFrameDim(frame_dim);

    m_buffer.setNbConcatFrames(1);
    m_buffer.setNbBuffers(1);
    if(reset_level == HardReset)
        m_cam.hardReset();
    else
	m_cam.softReset();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::prepareAcq()
{
    DEB_MEMBER_FUNCT();

    if(m_saving.isActive())
      m_saving.prepare();
    else
      m_buffer.prepare();
    m_sync.prepareAcq();
    // counter only use for IntTrigMult to increase the file number
    m_image_number = 0;

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::startAcq()
{
    DEB_MEMBER_FUNCT();

    // multi start calls if trigger mode is IntTrigMult
    if (m_image_number == 0) {
      if(m_saving.isActive())
	m_saving.start();
      else
	m_buffer.start();
    }
    // in case of IntTrigMult trigger mode an image number
    // is passed to the start to increase the file number
    m_cam.startAcquisition(m_image_number);
    m_image_number++;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::stopAcq()
{
    DEB_MEMBER_FUNCT();

    if(m_saving.isActive())
      m_saving.stop();
    else
      m_buffer.stop();

    m_cam.stopAcquisition();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::getStatus(StatusType& status)
{

    DEB_MEMBER_FUNCT();
    Camera::Status cam_status = m_cam.status();

    if(cam_status == Camera::STANDBY)
    {
	status.det = DetIdle;
	if(!m_saving.isActive())
	  {
	    int nbFrames;
	    m_sync.getNbHwFrames(nbFrames);
	    if(m_buffer.isStopped())
	      status.acq = AcqReady;
	    else {
	      status.acq = getNbHwAcquiredFrames() >= nbFrames ? AcqReady : AcqRunning;
	    }
	  }
	else
	  status.acq = AcqReady;
    }
    else if(cam_status == Camera::DISCONNECTED ||
	    cam_status == Camera::ERROR)
    {
        status.det = DetFault;
        status.acq = AcqFault;
    }
    else
    {
        status.det = DetExposure;
        status.acq = AcqRunning;       
    }    
    status.det_mask = DetExposure;
    DEB_TRACE() << DEB_VAR2(cam_status,status);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
int Interface::getNbHwAcquiredFrames()
{
    DEB_MEMBER_FUNCT();
    int acq_frames = m_buffer.getLastAcquiredFrame()+1;
    return acq_frames;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::setThresholdGain(int threshold, Camera::Gain gain)
{
    m_cam.setThresholdGain(threshold, gain);
}
//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::setThreshold(int threshold,int energy)
{
  m_cam.setThreshold(threshold,energy);
}
//-----------------------------------------------------
//
//-----------------------------------------------------
int Interface::getThreshold(void)
{
    return m_cam.threshold();
}


//-----------------------------------------------------
//
//-----------------------------------------------------
Camera::Gain Interface::getGain(void)
{
    return m_cam.gain();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::sendAnyCommand(const std::string& str)
{
    m_cam.sendAnyCommand(str);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::setEnergy(double energy)
{
	m_cam.setEnergy(energy);
}
//-----------------------------------------------------
//
//-----------------------------------------------------
double Interface::getEnergy(void)
{
    return m_cam.energy();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
