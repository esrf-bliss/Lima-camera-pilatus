//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2023
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
#ifndef PILATUSCAMERA_H
#define PILATUSCAMERA_H

#include "lima/Debug.h"



namespace lima
{
namespace Pilatus
{
class Interface;
class Camera
{
    friend class Interface;
    DEB_CLASS_NAMESPC(DebModCameraCom,"Camera","Pilatus");

public:
    enum Status
    {
        ERROR,
        DISCONNECTED,
        STANDBY,
        SETTING_ENERGY,
        SETTING_THRESHOLD,
        SETTING_EXPOSURE,
        SETTING_NB_IMAGE_IN_SEQUENCE,
        SETTING_EXPOSURE_PERIOD,
        SETTING_HARDWARE_TRIGGER_DELAY,
        SETTING_EXPOSURE_PER_FRAME,
	SETTING_ROI,
        KILL_ACQUISITION,
        RUNNING,
	RESET_HIGH_VOLTAGE,
	ANYCMD
    };

    enum Gain
    {
        DEFAULT_GAIN,
        LOW,
        MID,
        HIGH,
        UHIGH
    };

    enum TriggerMode
    {
        INTERNAL_SINGLE,
        INTERNAL_MULTI,
        EXTERNAL_SINGLE,
        EXTERNAL_MULTI,
        EXTERNAL_GATE
    };

    Camera(const std::string& host_name = "localhost",
	   int host_port = 41234,
	   const std::string& config_file = "/home/det/p2_det/config/cam_data/camera.def",
	   const std::string& tmpfs_path = "/lima_data");
 ~Camera();
    
    void connect(const char* host,int port);
    
    const char* serverIP() const;
    int serverPort() const;

    void setImgpath(const std::string& path);
    const std::string& imgpath() const;
    
    void setFileName(const std::string& name);
    const std::string& fileName() const;
    
    Status status() const;

    double energy() const;
    void setEnergy(double val);

    int threshold() const;
    Gain gain() const;
    void setThresholdGain(int threshold,Gain gain = DEFAULT_GAIN); // backward compatibility
    void setThreshold(int threshold,int energy = -1);
  
    double exposure() const;
    void setExposure(double expo);

    double exposurePeriod() const;
    void setExposurePeriod(double expo_period);

    int nbImagesInSequence() const;
    void setNbImagesInSequence(int nb);

    double hardwareTriggerDelay() const;
    void setHardwareTriggerDelay(double);

    int nbExposurePerFrame() const;
    void setNbExposurePerFrame(int);

    TriggerMode triggerMode() const;
    void setTriggerMode(TriggerMode);

    void startAcquisition(int image_number = 0);
    void stopAcquisition();
    void errorStopAcquisition();

    bool gapfill() const;
    void setGapfill(bool onOff);
    
    void send(const std::string& message);
    
    void sendAnyCommand(const std::string& message);    
    std::string sendAnyCommandAndGetErrorMsg(const std::string& message);

    int nbAcquiredImages() const;
    void version(int& major,int& minor,int& patch) const;

    bool hasRoiCapability() const;
    void setRoi(const std::string&);

    bool hasHighVoltageReset();
    void resetHighVoltage(unsigned int sleeptime = 1);
    const char* configFile() const {return m_config_file.c_str();};
    const char* tmpFsPath() const {return m_tmpfs_path.c_str();};

  void getTemperatureHumidity(float& temperature, float& humidity);
  
private:
    static constexpr double             TIME_OUT = 10.;
    enum HIGH_VOLTAGE { NOT_INITIALIZED,
			HAS_HIGH_VOLTAGE,DONT_HAVE_HIGH_VOLTAGE };

    const        std::string& errorMessage() const;
    void         softReset();
    void         hardReset();
    void         quit();    
    void	 _connect(const char* host,int port);
    
    static void* _runFunc(void*);
    void         _run();    
    void         _initVariable();
    void         _resync();
    void         _reinit();
    void	 _pilatus2model();
    void	 _pilatus3model(); ///< set pilatus3 threshold extention
    void         _work_around_threshold_bug();

    std::map<std::string,Gain>    GAIN_SERVER_RESPONSE;
    std::map<Gain,std::string>    GAIN_VALUE2SERVER;


    //socket/synchronization with pilatus variables
    std::string             m_server_ip;
    int                     m_server_port;
    std::string             m_config_file;
    std::string             m_tmpfs_path;
    int                     m_socket;
    bool                    m_stop;
    pthread_t               m_thread_id;
    int                     m_pipes[2];
    Status                  m_state;
    mutable Cond            m_cond;

    //Cache variables
    std::string             m_error_message;
    int			    m_energy;
    double                  m_exposure;
    int                     m_exposure_per_frame;
    double                  m_exposure_period;
    Gain                    m_gain;
    bool                    m_gap_fill;
    double                  m_hardware_trigger_delay;
    int                     m_nimages;
    int                     m_threshold;
    TriggerMode             m_trigger_mode;
    std::string             m_imgpath;
    std::string             m_file_name;
    std::string             m_file_pattern;    
    int			    m_nb_acquired_images;
    bool		    m_has_cmd_setenergy;
    bool                    m_pilatus2_model;
    bool                    m_pilatus3_threshold_mode;
    bool		    m_has_cmd_roi;
    int			    m_major_version;
    int                     m_minor_version;
    int                     m_patch_version;
    HIGH_VOLTAGE            m_cmd_high_voltage_reset;
    float                   m_temperature;
    float                   m_humidity;
  short                     m_channel;
};
}
}
#endif//PILATUSCAMERA_H
