//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2023
// European Synchrotron Radiation Facility
// CS40220 38043 Grenoble Cedex 9
// FRANCE
//
// Contact: lima@esrf.fr
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

#include <pthread.h>

#include <stdlib.h>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <iostream>
#include <iomanip>

#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <poll.h>
#include <unistd.h>

#include "lima/Exceptions.h"

#include "PilatusCamera.h"

using namespace lima;
using namespace lima::Pilatus;

static const char  SOCKET_SEPARATOR = '\030';
static const char* SPLIT_SEPARATOR  = "\x18"; // '\x18' == '\030'

//---------------------------
//- utility function
//---------------------------
static inline const char* _get_ip_addresse(const char *name_ip)
{
  
  if(inet_addr(name_ip) != INADDR_NONE)
    return name_ip;
  else
    {
      struct hostent *host = gethostbyname(name_ip);
      if(!host)
	{
	  char buffer[256];
	  snprintf(buffer,sizeof(buffer),"Can not found ip for host %s ",name_ip);
	  throw LIMA_HW_EXC(Error,buffer);
	}
      return inet_ntoa(*((struct in_addr*)host->h_addr));
    }
}


#define RECONNECT_WAIT_UNTIL(testState,errmsg)			    \
  if(m_socket == -1)						    \
    _connect(m_server_ip.c_str(),m_server_port);			    \
								    \
  while(m_state != testState &&					    \
	m_state != Camera::ERROR &&				    \
	m_state != Camera::DISCONNECTED)			    \
{                                                                   \
  if(!m_cond.wait(TIME_OUT))                                        \
    THROW_HW_ERROR(lima::Error) << errmsg;                          \
}

//-----------------------------------------------------
//
//-----------------------------------------------------
inline void _split(const std::string inString,
		   const std::string &separator,
		   std::vector<std::string> &returnVector)
{
    std::string::size_type start = 0;
    std::string::size_type end = 0;

    while ((end=inString.find (separator, start)) != std::string::npos)
    {
        returnVector.push_back (inString.substr (start, end-start));
        start = end+separator.size();
    }

    returnVector.push_back (inString.substr (start));
}

//-----------------------------------------------------
// split the thread command returned message and extract
// the channel, temperature and humidity
// for instance a 6M model returns:
// msg=215 OK Channel 0: Temperature = 32.1C, Rel. Humidity = 28.6%;
// Channel 1: Temperature = 27.0C, Rel. Humidity = 38.1%;
// Channel 2: Temperature = 29.5C, Rel. Humidity = 0.0%

//-----------------------------------------------------
inline void _split_THread(const std::string inString,
			  std::vector<float> &returnVector)
{
    std::string item;
    std::stringstream ss (inString.substr(6));
    
    float channel, temperature, humidity;

    while (getline(ss, item, ';'))
    {
        channel = stof(item.substr(9,1));
	temperature = stof(item.substr(item.find("Temperature")+ 14,4));
	humidity = stof(item.substr(item.find("Humidity")+ 11,4));
	returnVector.push_back (channel);
	returnVector.push_back (temperature);
	returnVector.push_back (humidity);
    }
}

//-----------------------------------------------------
//
//-----------------------------------------------------
Camera::Camera(const std::string& host_name,
	       int host_port,
	       const std::string& config_file,
	       const std::string& tmpfs_path)
                :   m_socket(-1),
                    m_stop(false),
                    m_thread_id(0),
                    m_state(DISCONNECTED),
                    m_nb_acquired_images(0),
		    m_has_cmd_setenergy(true),
		    m_pilatus2_model(false),
		    m_pilatus3_threshold_mode(false),
		    m_has_cmd_roi(true),
		    m_major_version(-1),
		    m_minor_version(-1),
		    m_patch_version(-1),
		    m_cmd_high_voltage_reset(NOT_INITIALIZED),
		    m_server_ip(host_name),
		    m_server_port(host_port),
		    m_config_file(config_file),
		    m_tmpfs_path(tmpfs_path),
		    m_acq_killed(false)

{
    DEB_CONSTRUCTOR();

    _initVariable();

    if(pipe(m_pipes))
        THROW_HW_ERROR(Error) << "Can't open pipe";

    pthread_create(&m_thread_id,NULL,_runFunc,this);

    try
      {
	connect(host_name.c_str(),host_port);
      }
    catch(Exception &e)		// Not an error in that case
      {
      }
}

//-----------------------------------------------------
//
//-----------------------------------------------------
Camera::~Camera()
{
    DEB_MEMBER_FUNCT();
    AutoMutex aLock(m_cond.mutex());
    m_nb_acquired_images = 0;
    m_stop = true;
    if(m_socket >=0)
    {
      if(write(m_pipes[1],"|",1) == -1)
	DEB_ERROR() << "Something wrong happen!!!";

      close(m_socket);
      m_socket = -1;
    }
    else
    {
      if(write(m_pipes[1],"|",1) == -1)
	DEB_ERROR() << "Something wrong happen!!!";
    }
    aLock.unlock();

    void *returnPt;
    if(m_thread_id > 0)
    {
        DEB_TRACE()<<"[pthread_join - BEGIN]";
        if(pthread_join(m_thread_id,&returnPt)!=0)
        {
            DEB_TRACE()<<"UNknown Error";
        }
        DEB_TRACE()<<"[pthread_join - END]";
    }
}

//-----------------------------------------------------
//
//-----------------------------------------------------
const char* Camera::serverIP() const
{
  return m_server_ip.c_str();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
int Camera::serverPort() const
{
    return m_server_port;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::_initVariable()
{
    m_imgpath                           = "/ramdisk/images/";
    m_file_name                         = "image_%.5d.cbf";
    m_file_pattern                      = m_file_name;
    m_threshold                         = -1;
    m_energy                            = -1;
    m_gain                              = DEFAULT_GAIN;
    m_exposure                          = -1.;
    m_nimages                           = 1;
    m_exposure_period                   = -1.;
    m_hardware_trigger_delay            = -1.;
    m_exposure_per_frame                = 1;
    m_nb_acquired_images 		= 0;
    m_trigger_mode 			= INTERNAL_SINGLE;

    GAIN_SERVER_RESPONSE["low"]         = LOW;
    GAIN_SERVER_RESPONSE["mid"]         = MID;
    GAIN_SERVER_RESPONSE["high"]        = HIGH;
    GAIN_SERVER_RESPONSE["ultra"]       = UHIGH;
    GAIN_SERVER_RESPONSE["autog"]       = AUTOG;

    GAIN_VALUE2SERVER[LOW]              = "lowG";
    GAIN_VALUE2SERVER[MID]              = "midG";
    GAIN_VALUE2SERVER[HIGH]             = "highG";
    GAIN_VALUE2SERVER[UHIGH]            = "uhighG";
    GAIN_VALUE2SERVER[AUTOG]            = "autoG";
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::connect(const char *host,int port)
{
  DEB_MEMBER_FUNCT();
  AutoMutex aLock(m_cond.mutex());
  _initVariable();
  _connect(host,port);
}

void Camera::_connect(const char *host,int port)
{
    DEB_MEMBER_FUNCT();

    if(host && port)
    {
        if(m_socket >= 0)
        {
            close(m_socket);
            m_socket = -1;
        }
        else
        {
            m_socket = socket(PF_INET, SOCK_STREAM,IPPROTO_TCP);
            if(m_socket >= 0)
            {
                int flag = 1;
                setsockopt(m_socket,IPPROTO_TCP,TCP_NODELAY, (void*)&flag,sizeof(flag));
                struct sockaddr_in add;
                add.sin_family = AF_INET;
                add.sin_port = htons((unsigned short)port);
                add.sin_addr.s_addr = inet_addr(_get_ip_addresse(host));
                if(::connect(m_socket,reinterpret_cast<sockaddr*>(&add),sizeof(add)))
                {
                    close(m_socket);
                    m_socket = -1;
                    THROW_HW_ERROR(Error) << "Can't open connection";
                }
                if(write(m_pipes[1],"|",1) == -1)
		  DEB_ERROR() << "Something wrong happen to pipe ???";

                m_state = Camera::STANDBY;
                _resync();
            }
            else
                THROW_HW_ERROR(Error) << "Can't create socket";
        }
    }
    //Workaround to avoid bug in camserver
    send("exposure warmup.edf");
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::_resync()
{
    if(m_has_cmd_setenergy)
      send("setenergy");
    else
      send("setthreshold");
    if(m_has_cmd_roi)
      send("setroi 0");
    send("exptime");
    send("expperiod");
    send("nimages 1");
    std::stringstream cmd;
    cmd<<"imgpath "<<m_imgpath;
    send(cmd.str());
    send("delay");
    send("nexpframe");
    send("setackint 0");
    send("dbglvl 1");
    send("version");
    // read temperature&humidity without channel number camserver will return the first available channel
    send("thread");
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::_reinit()
{
    _resync();
    send("nimages");
}
//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::_pilatus2model()
{
  m_pilatus2_model = true;
}
//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::_pilatus3model()
{
  m_pilatus3_threshold_mode = true;
}
//-----------------------------------------------------
// This method will reset threshold if it's a pilatus3 
// and bugged version
//-----------------------------------------------------
void Camera::_work_around_threshold_bug()
{
  if(m_pilatus3_threshold_mode &&
     m_major_version <= 7 &&
     m_minor_version <= 4 &&
     m_patch_version <= 3)
    send("setthreshold 0");
}
//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::send(const std::string& message)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << DEB_VAR1(message);
    std::string msg = message;
    msg+= SOCKET_SEPARATOR;
    if(write(m_socket,msg.c_str(),msg.size()) == -1)
      THROW_HW_ERROR(Error) << "Could not send message to camserver";

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void* Camera::_runFunc(void *commPt)
{
    ((Camera*)commPt)->_run();
    return NULL;
}


/******************************************************
--> SUCCESS CASE (set number of images)
ni 10
15 OK N images set to: 10

--> SUCCESS CASE (get path where images are saved)
imgpath
10 OK /home/det/p2_det/images/

--> SUCCESS CASE (get path where images must be saved)
imgpath /ramdisk/tmp/Arafat
10 OK /ramdisk/tmp/Arafat/

--> SUCCESS CASE (set exposure period)
expp 0.2
15 OK Exposure period set to: 0.2000000 sec

--> SUCCESS CASE (set exposure time)
expt 0.997
15 OK Exposure time set to: 0.9970000 sec

--> SUCCESS CASE (start acquisition)
exposure toto_0001.cbf
15 OK  Starting 0.1970000 second background: 2011-Aug-04T15:27:22.593
7 OK /ramdisk/tmp/Arafat/toto_0300.cbf

--> SUCCESS CASE (abort acquisition)
exposure toto_0001.cbf
15 OK  Starting 0.1970000 second background: 2011-Aug-04T15:33:03.178
k
13 ERR kill
7 OK /ramdisk/tmp/Arafat/toto_0019.cbf

--> SUCCESS CASE (get threshold)
setthr
15 OK  Settings: mid gain; threshold: 6300 eV; vcmp: 0.654 V
 Trim file:
  /home/det/p2_det/config/calibration/p6m0106_E12600_T6300_vrf_m0p20.bin
 
--> SUCCESS CASE (set threshold to 6000)
setthr 6000
15 OK /tmp/setthreshold.cmd

--> ERROR CASE (send a bad command)
threshold
1 ERR *** Unrecognized command: threshold

--> ERROR CASE (bad parameters)
setthr low 5000
15 ERR ERROR: unknown gain setting: low 5000

TO DO :
1 - manage state for threshold : DONE -> OK
2 - resync() after threshold -> need manage state : DONE -> OK
3 - manage the stop() command like an abort :DONE -> OK
******************************************************/
void Camera::_run()
{
    DEB_MEMBER_FUNCT();
    struct pollfd fds[2];
    fds[0].fd = m_pipes[0];
    fds[0].events = POLLIN;

    fds[1].events = POLLIN;
    AutoMutex aLock(m_cond.mutex());
    while(!m_stop)
    {
        int nb_poll_fd = 1;
        if(m_socket >= 0)
        {
            fds[1].fd = m_socket;
            ++nb_poll_fd;
        }
        aLock.unlock();
        poll(fds,nb_poll_fd,-1);
        aLock.lock();
        if(fds[0].revents)
        {
            char buffer[1024];
            if(read(m_pipes[0],buffer,sizeof(buffer)) == -1)
	      DEB_WARNING() << "Something strange happen!";
        }
        if(nb_poll_fd > 1 && fds[1].revents)
        {
            char messages[16384];
            int aMessageSize = recv(m_socket,messages,sizeof(messages),0);                    
            if(aMessageSize<=0)
            {
                DEB_TRACE() <<"-- no message received";
                close(m_socket);
                m_socket = -1;
                m_state = Camera::DISCONNECTED;                
            }
            else
            {                
                if(m_state == Camera::ERROR)//nothing to do, keep last error until a new explicit user command (start, stop, setenergy, ...)
                  continue;
                std::string strMessages(messages,aMessageSize );                    
                DEB_TRACE() << DEB_VAR1(strMessages);
                std::vector<std::string> msg_vector;
                _split(strMessages,SPLIT_SEPARATOR,msg_vector);
                for(std::vector<std::string>::iterator msg_iterator = msg_vector.begin();
		    msg_iterator != msg_vector.end();++msg_iterator)
                {
                    std::string &msg = *msg_iterator;
                    if(msg.substr(0,2) == "15") // generic response
                    {
                        if(msg.substr(3,2) == "OK") // will check what the message is about
                        {
                            std::string real_message = msg.substr(6);
			    size_t position;
                            if(real_message.find("Energy") != std::string::npos)
			    {
			      size_t columnPos = real_message.find(":");
			      if(columnPos == std::string::npos)
				{
				  m_threshold = m_energy = -1;
				  m_gain = DEFAULT_GAIN;
				}
			      else
				m_energy = atoi(real_message.substr(columnPos + 1).c_str());
			    }
                            if((position = real_message.find("Settings:")) !=
			       std::string::npos) // Threshold and gain is already set,read them
                            {
			        std::string submsg = real_message.substr(position);
                                std::vector<std::string> threshold_vector;
                                _split(submsg.substr(10),";",threshold_vector);
                                std::string &gain_string = threshold_vector[0];
                                std::string &threshold_string = threshold_vector[1];
                                std::vector<std::string> thr_val;
                                _split(threshold_string," ",thr_val);
                                std::string &threshold_value = thr_val[2];
                                m_threshold = atoi(threshold_value.c_str());

                                std::vector<std::string> gain_split;
                                _split(gain_string," ",gain_split);
                                std::string &gain_value = gain_split[0];
                                std::map<std::string,Gain>::iterator gFind = GAIN_SERVER_RESPONSE.find(gain_value);
                                m_gain = gFind->second;
                                m_state = Camera::STANDBY;                               
                            }
                            else if(real_message.find("/tmp/setthreshold")!=std::string::npos)
                            {
                                if(m_state == Camera::SETTING_THRESHOLD)
                                {
                                  DEB_TRACE() << "-- Threshold process succeeded";
                                }
                                if(m_state == Camera::SETTING_ENERGY)
                                {
                                  DEB_TRACE() << "-- SetEnergy process succeeded";
                                }
                                _reinit(); // resync with server
                            }
                            else if(real_message.find("Exposure")!=std::string::npos)
                            {
                                int columnPos = real_message.find(":");
                                int lastSpace = real_message.rfind(" ");
                                if(real_message.substr(9,4) == "time")
                                {
                                    m_exposure = atof(real_message.substr(columnPos + 1,lastSpace).c_str());
                                }
                                else if(real_message.substr(9,6) == "period")
                                {
                                    m_exposure_period = atof(real_message.substr(columnPos + 1,lastSpace).c_str());
                                }
                                else // Exposures per frame
                                {
                                    m_exposure_per_frame = atoi(real_message.substr(columnPos + 1).c_str());\
                                }
                            }
                            else if(real_message.find("Delay")!=std::string::npos)
                            {
                                int columnPos = real_message.find(":");
                                int lastSpace = real_message.rfind(" ");
                                m_hardware_trigger_delay = atof(real_message.substr(columnPos + 1,lastSpace).c_str());
                            }
                            else if(real_message.find("N images")!=std::string::npos)
                            {
                                int columnPos = real_message.find(":");
                                m_nimages = atoi(real_message.substr(columnPos+1).c_str());
                            }
			    if(m_state != Camera::RUNNING)
			      m_state = Camera::STANDBY;
                        }// Fixes message from camserver tvx-7.3.13-121212 version
                        else if(msg.substr(0,10) == "15 ERR ROI" && m_pilatus2_model)
			  {
			      DEB_TRACE() << "-- setROI process succeeded";
			      m_state = Camera::STANDBY;
			  }
                        else  // ERROR MESSAGE
                        {
			  m_error_message = msg.substr(7);
                            if(m_state == Camera::SETTING_THRESHOLD)
			      DEB_TRACE() << "-- Threshold process failed";
                            if(m_state == Camera::SETTING_ENERGY)
			      DEB_TRACE() << "-- SetEnergy process failed";
                            else if(m_state == Camera::RUNNING)
			      DEB_TRACE() << "-- Exposure process failed";
                            else
			      DEB_TRACE() << "-- ERROR " << m_error_message;

			    m_state = Camera::ERROR;
			}
                        m_cond.broadcast();
                    }
                    else if(msg.substr(0,2) == "13") //Acquisition Killed
                    {
                        DEB_TRACE() << "-- Acquisition Killed"; 
                        m_state = Camera::STANDBY;
			m_acq_killed = true;
                    }
                    else if(msg.substr(0,2) == "7 ")
                    {
                        if(msg.substr(2,2) == "OK")
                        {
                            DEB_TRACE() << "-- Exposure succeeded";
                            m_state = Camera::STANDBY;
                            m_nb_acquired_images = m_nimages;
                        }
                        else
                        {
			    // camserver sends a 7 ERR just after the "13 Acquisition Killed" msg
			    // this is not an real error, so trap it here to not set the status to fault
			    if (m_acq_killed)
			        m_acq_killed = false;
			    else
			    {
                                DEB_TRACE() << "-- ERROR";                      
                                m_state = Camera::ERROR;
                                msg = msg.substr(2);
                                m_error_message = msg.substr(msg.find(" "));
			        DEB_TRACE() << m_error_message;
			    }
                        }
                    }
                    else if(msg.substr(0,2) == "1 ")
                    {
                        if(msg.substr(2,3) == "ERR")
                        {
			  // Not an error just old version of camserver
			  if(msg.find("Unrecognized command: setenergy") != 
			     std::string::npos)
			    {
			      m_has_cmd_setenergy = false;
			      _resync();
			    }
			  else if(msg.find("Unrecognized command: version") != 
				  std::string::npos)
			    {
			      DEB_TRACE() << "Can't retrieved camserver version";
			    }
			  else if(msg.find("Unrecognized command: setroi") !=
				  std::string::npos)
			    {
			      m_has_cmd_roi = false;
			      _resync();
			    }
			  else if(msg.find("Unrecognized command: resetmodulepower") != 
				  std::string::npos)
			    {
			      m_cmd_high_voltage_reset = DONT_HAVE_HIGH_VOLTAGE;
			      m_state = Camera::STANDBY;
			      m_cond.broadcast();
			    }
			  else
			    {
                            DEB_TRACE() << "-- ERROR";
                            m_error_message = msg.substr(6);
                            DEB_TRACE() << m_error_message;
                            m_state = Camera::ERROR;
			    }
                        }
                    }
                    else if(msg.substr(0,2) == "10")
                    {
                        if(msg.substr(3,2) == "OK")
                        {
                            DEB_TRACE() << "-- imgpath setting succeeded";
                            ////m_imgpath = msg.substr(6);////@@@@
                            m_state = Camera::STANDBY;
                        }
                        else
                        {
                            DEB_TRACE() << "-- ERROR";
                            m_state = Camera::ERROR;
                            msg = msg.substr(2);
                            m_error_message = msg.substr(msg.find(" "));
                            DEB_TRACE() << m_error_message;
                        }                        
                    }
		    else if(msg.substr(0,2) == "24")
		      {
			if(msg.substr(3,2) == "OK" &&
			   msg.substr(6,12) == "Code release")
			  {
			    DEB_TRACE() << msg.substr(6);
			    std::vector<std::string> version_vector;
			    _split(msg.substr(20),".",version_vector);
			    if(version_vector.size() == 3)
			      {
				m_major_version = atoi(version_vector[0].c_str());
				m_minor_version = atoi(version_vector[1].c_str());
				m_patch_version = atoi(version_vector[2].c_str());
			      }
			  }
		      }
		    else if (msg.substr(0,3) == "215")
		      {
			if(msg.substr(4,2) == "OK" &&
			   msg.substr(7,7) == "Channel")
			  {
			    std::vector<float> th;
			    _split_THread(msg, th);
			    m_channel_temperature_humidity_list = th;
			  }
		      }
                }
            }
        }
    }
}
//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setImgpath(const std::string& path)
{
    DEB_MEMBER_FUNCT();
    AutoMutex aLock(m_cond.mutex());
    RECONNECT_WAIT_UNTIL(Camera::STANDBY,
			 "Could not set imgpath, server not idle");
    m_imgpath = path;    
    std::stringstream cmd;
    cmd<<"imgpath "<<m_imgpath;
    send(cmd.str());
}

//-----------------------------------------------------
//
//-----------------------------------------------------
const std::string& Camera::imgpath() const
{
    AutoMutex aLock(m_cond.mutex());
    return m_imgpath;
}

//-----------------------------------------------------
//
//-----------------------------------------------------   
void Camera::setFileName(const std::string& name)
{
    DEB_MEMBER_FUNCT();
    AutoMutex aLock(m_cond.mutex());
    m_file_name = name;
    m_file_pattern = m_file_name;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
const std::string& Camera::fileName() const
{
    AutoMutex aLock(m_cond.mutex());
    return m_file_name;
}
    
    
//-----------------------------------------------------
//
//-----------------------------------------------------
Camera::Status Camera::status() const
{
    AutoMutex aLock(m_cond.mutex());
    return m_state;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
const std::string& Camera::errorMessage() const
{
    AutoMutex aLock(m_cond.mutex());
    return m_error_message;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::softReset()
{
    AutoMutex aLock(m_cond.mutex());
    m_error_message.clear();
    m_state = Camera::STANDBY;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::hardReset()
{
    AutoMutex aLock(m_cond.mutex());
    send("resetcam");
}

//-----------------------------------------------------
// Return energy in keV
//-----------------------------------------------------
double Camera::energy() const
{
    AutoMutex aLock(m_cond.mutex());
    if(m_has_cmd_setenergy)
       return (double)m_energy/1000;
    else
       return (double)m_threshold/600;
}

//-----------------------------------------------------
// Set energy in keV
//-----------------------------------------------------
void Camera::setEnergy(double val)
{
    DEB_MEMBER_FUNCT();

    AutoMutex aLock(m_cond.mutex());
    RECONNECT_WAIT_UNTIL(Camera::STANDBY,
			 "Could not set energy, server is not idle");
    if(m_has_cmd_setenergy)
      {
	_work_around_threshold_bug();

	m_state = Camera::SETTING_ENERGY;
	std::stringstream msg;
	msg << "setenergy " << val*1000;
	send(msg.str());
      }
    else
      {
        Camera::Gain gain;
        int threshold;
        // In old version of camserver,  setenergy is not implemented, emulate it instead
        // with threshold and gain, according to rules of ranges given by Dectris support
        threshold = (int)(val * 600); //60% of the energy in eV
        if (val > 12) gain = LOW;
        else if (val > 8 && val <= 12) gain = MID;
        else if (val >= 6 && val <= 8) gain = HIGH;
        else gain = UHIGH;
        aLock.unlock();
        setThresholdGain(threshold, gain);
      }

}

//-----------------------------------------------------
// Return the threshold in eV
//-----------------------------------------------------
int Camera::threshold() const
{
    AutoMutex aLock(m_cond.mutex());
    return m_threshold;
}

//-----------------------------------------------------
// Return the gain
//-----------------------------------------------------
Camera::Gain Camera::gain() const
{
    AutoMutex aLock(m_cond.mutex());
    return m_gain;
}

//-----------------------------------------------------
// Set Threshold (ev) and gain pair
//-----------------------------------------------------
void Camera::setThresholdGain(int value,Camera::Gain gain)
{
    DEB_MEMBER_FUNCT();

    AutoMutex aLock(m_cond.mutex());
    RECONNECT_WAIT_UNTIL(Camera::STANDBY,
			 "Could not set threshold, server is not idle");
    m_state = Camera::SETTING_THRESHOLD;    
    if(gain == DEFAULT_GAIN)
    {
        char buffer[128];
        snprintf(buffer,sizeof(buffer),"setthreshold %d",value);
        send(buffer);
    }
    else
    {
        std::map<Gain,std::string>::iterator i = GAIN_VALUE2SERVER.find(gain);
        std::string &gainStr = i->second;

        char buffer[128];
        snprintf(buffer,sizeof(buffer),"setthreshold %s %d",gainStr.c_str(),value);

        send(buffer);
    }


    if (m_gap_fill)
        send("gapfill -1");
}
//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setThreshold(int threshold,int energy)
{
  DEB_MEMBER_FUNCT();

  if(!m_pilatus3_threshold_mode)
    THROW_HW_ERROR(NotSupported) << "Could not use pilatus threshold flavor";

  AutoMutex aLock(m_cond.mutex());
  RECONNECT_WAIT_UNTIL(Camera::STANDBY,
		       "Could not set threshold,server is not idle");

  _work_around_threshold_bug();

  m_state = Camera::SETTING_THRESHOLD;
  if(energy < 0)
    {
      char buffer[128];
      snprintf(buffer,sizeof(buffer),"setthreshold %d",threshold);
      send(buffer);
    }
  else
    {
      char buffer[128];
      snprintf(buffer,sizeof(buffer),"setthreshold energy %d %d",energy,threshold);
      send(buffer);
    }
}
//-----------------------------------------------------
//
//-----------------------------------------------------
double Camera::exposure() const
{
    AutoMutex aLock(m_cond.mutex());
    return m_exposure;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setExposure(double val)
{
    DEB_MEMBER_FUNCT();
    AutoMutex aLock(m_cond.mutex());
    // yet an other border-effect with the SPEC CCD interface
    // to reach the GATE mode SPEC programs extgate + expotime = 0
    if(m_trigger_mode == Camera::EXTERNAL_GATE and val <= 0)
        return;

    RECONNECT_WAIT_UNTIL(Camera::STANDBY,
			 "Could not set exposure, server is not idle");
    m_state = Camera::SETTING_EXPOSURE;
    std::stringstream msg;
    msg << "exptime " << val;
    send(msg.str());
}

//-----------------------------------------------------
//
//-----------------------------------------------------
double Camera::exposurePeriod() const
{
    AutoMutex aLock(m_cond.mutex());
    return m_exposure_period;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setExposurePeriod(double val)
{
    DEB_MEMBER_FUNCT();
    AutoMutex aLock(m_cond.mutex());
    RECONNECT_WAIT_UNTIL(Camera::STANDBY,
			 "Could not set exposure period, server not idle");
    m_state = Camera::SETTING_EXPOSURE_PERIOD;
    std::stringstream msg;
    msg << std::setprecision(9) << "expperiod " << val;
    send(msg.str());
    // Exposure period can failed if it's two fast
    while(m_state == Camera::SETTING_EXPOSURE_PERIOD)
      m_cond.wait(TIME_OUT);
    if(m_state == Camera::ERROR)
      {
	m_state = Camera::STANDBY;
	THROW_HW_ERROR(Error) << "Could not set exposure period to:"
			      << DEB_VAR2(val,m_error_message);
      }
}

//-----------------------------------------------------
//
//-----------------------------------------------------
int Camera::nbImagesInSequence() const
{
    AutoMutex aLock(m_cond.mutex());
    return m_nimages;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setNbImagesInSequence(int nb)
{
    DEB_MEMBER_FUNCT();
    AutoMutex aLock(m_cond.mutex());
    RECONNECT_WAIT_UNTIL(Camera::STANDBY,
			 "Could not set number image in sequence, server not idle");
    m_state = Camera::SETTING_NB_IMAGE_IN_SEQUENCE;
    std::stringstream msg;
    msg << "nimages " << nb;
    send(msg.str());
}

//-----------------------------------------------------
//
//-----------------------------------------------------
double Camera::hardwareTriggerDelay() const
{
    AutoMutex aLock(m_cond.mutex());
    return m_hardware_trigger_delay;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setHardwareTriggerDelay(double value)
{
    DEB_MEMBER_FUNCT();
    AutoMutex aLock(m_cond.mutex());
    RECONNECT_WAIT_UNTIL(Camera::STANDBY,
			 "Could not set hardware trigger delay, server not idle");
    m_state = Camera::SETTING_HARDWARE_TRIGGER_DELAY;
    std::stringstream msg;
    msg << "delay " << value;
    send(msg.str());
}

//-----------------------------------------------------
//
//-----------------------------------------------------
int Camera::nbExposurePerFrame() const
{
    AutoMutex aLock(m_cond.mutex());
    return m_exposure_per_frame;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setNbExposurePerFrame(int val)
{
    DEB_MEMBER_FUNCT();
    AutoMutex aLock(m_cond.mutex());
    RECONNECT_WAIT_UNTIL(Camera::STANDBY,
			 "Could not set exposure per frame, server not idle");
    m_state = Camera::SETTING_EXPOSURE_PER_FRAME;
    std::stringstream msg;
    msg << "nexpframe " << val;
    send(msg.str());
}

//-----------------------------------------------------
//
//-----------------------------------------------------
Camera::TriggerMode Camera::triggerMode() const
{
    AutoMutex aLock(m_cond.mutex());
    return m_trigger_mode;
}

//-----------------------------------------------------
//@brief set the trigger mode
//
//Trigger can be:
// - Internal (Software) == INTERNAL_SINGLE
// - External start == EXTERNAL_SINGLE
// - External multi start == EXTERNAL_MULTI
// - External gate == EXTERNAL_GATE
//-----------------------------------------------------
void Camera::setTriggerMode(Camera::TriggerMode triggerMode)
{
    AutoMutex aLock(m_cond.mutex());
    m_trigger_mode = triggerMode;
}


//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::startAcquisition(int image_number)
{
    DEB_MEMBER_FUNCT();
    AutoMutex aLock(m_cond.mutex());
    m_nb_acquired_images = 0;
    if(m_state == Camera::RUNNING)
        THROW_HW_ERROR(Error) << "Could not start acquisition, you have to wait the end of the previous one";

    char filename[256];
    snprintf(filename,sizeof(filename),m_file_pattern.c_str(),image_number);

    //Start Acquisition
 
    RECONNECT_WAIT_UNTIL(Camera::STANDBY,
			 "Could not start Acquisition, server not idle");    
    m_state = Camera::RUNNING;
    std::stringstream msg;

    if(m_trigger_mode == Camera::EXTERNAL_SINGLE)
      msg << "exttrigger " << filename;
    else if(m_trigger_mode == Camera::EXTERNAL_MULTI)
      msg << "extmtrigger " << filename;
    else if(m_trigger_mode == Camera::EXTERNAL_GATE)
      msg << "extenable " << filename;
    else
      msg << "exposure " << filename;

    send(msg.str());
    m_cond.wait(TIME_OUT);
    if(m_state == Camera::ERROR)
    {
      m_state = Camera::STANDBY;
      THROW_HW_ERROR(Error) << "Could not start the acquisition: "
			    << m_error_message;
    }
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::stopAcquisition()
{
    AutoMutex aLock(m_cond.mutex());
    if(m_state == Camera::RUNNING)
    {
        m_state = Camera::KILL_ACQUISITION;
	if (m_pilatus2_model)
	  send("k");
	else
	  // interrupts a single exposure and an exposure series
	  // only avvailable for Pilatus 3 models (camserver 7)
	  send("camcmd k");
    }
}

void Camera::errorStopAcquisition()
{
  stopAcquisition();
  m_state = Camera::ERROR;
}
//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setGapfill(bool val)
{
    DEB_MEMBER_FUNCT();
    AutoMutex aLock(m_cond.mutex());
    RECONNECT_WAIT_UNTIL(Camera::STANDBY,
			 "Could not set gap, server not idle");
    m_gap_fill = val;
    std::stringstream msg;
    msg << "gapfill " << (m_gap_fill ? -1 : 0);
    send(msg.str());
}

//-----------------------------------------------------
//
//-----------------------------------------------------
bool Camera::gapfill() const
{
    AutoMutex aLock(m_cond.mutex());
    return m_gap_fill;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::sendAnyCommand(const std::string& message)
{
    DEB_MEMBER_FUNCT();

    AutoMutex aLock(m_cond.mutex());
    RECONNECT_WAIT_UNTIL(Camera::STANDBY,
			 "Could not send the Command, server is not idle");

    send(message);
}

std::string Camera::sendAnyCommandAndGetErrorMsg(const std::string& message)
{
  DEB_MEMBER_FUNCT();

  AutoMutex aLock(m_cond.mutex());
  RECONNECT_WAIT_UNTIL(Camera::STANDBY,
		       "Could not send the Command, server is not idle");

  m_state = Camera::ANYCMD;
  send(message);

  while(m_state != Camera::STANDBY &&
	m_state != Camera::ERROR &&
	m_state != Camera::DISCONNECTED)
    {
      if(!m_cond.wait(TIME_OUT))
	return "Timeout";
    }

  if(m_state == Camera::ERROR)
    return m_error_message;
  else if(m_state == Camera::DISCONNECTED)
    return "Disconnected";
  else
    return "";
}
//-----------------------------------------------------
//
//-----------------------------------------------------
int Camera::nbAcquiredImages() const
{
    DEB_MEMBER_FUNCT();

    AutoMutex aLock(m_cond.mutex());
    return m_nb_acquired_images;
}
//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::version(int& major,int& minor,int& patch) const
{
  major = m_major_version;
  minor = m_minor_version;
  patch = m_patch_version;
}
//-----------------------------------------------------
//  ROI(S)
//-----------------------------------------------------
bool Camera::hasRoiCapability() const
{
  AutoMutex lock(m_cond.mutex());
  return m_has_cmd_roi;
}
void Camera::setRoi(const std::string& roi_pattern)
{
  DEB_MEMBER_FUNCT();
  if(!hasRoiCapability())
    THROW_HW_ERROR(Error) << "This detector doesn't manage hardware roi";

  AutoMutex aLock(m_cond.mutex());
  RECONNECT_WAIT_UNTIL(Camera::STANDBY,
		       "Could not set exposure period, server not idle");
  m_state = Camera::SETTING_ROI;
  std::stringstream msg;
  msg << "setroi " << roi_pattern;
  send(msg.str());
  // ROI can failed if it's not manage in this detector
  while(m_state == Camera::SETTING_ROI)
    m_cond.wait(TIME_OUT);
  if(m_state == Camera::ERROR)
    {
      m_state = Camera::STANDBY;
      THROW_HW_ERROR(Error) << "Could not set roi to:"
			    << DEB_VAR2(roi_pattern,m_error_message);
    }
}
/*-----------------------------------------------------
		  High voltage reset command (Cdte)
------------------------------------------------------*/
bool Camera::hasHighVoltageReset()
{
  DEB_MEMBER_FUNCT();

  AutoMutex lock(m_cond.mutex());
  if(m_pilatus3_threshold_mode)
    {
      if(m_cmd_high_voltage_reset == NOT_INITIALIZED)
	{
          lock.unlock();
	  try
	    {
	      resetHighVoltage();
	    }
	  catch(...)
	    {}
	  lock.lock();
	}
      return m_cmd_high_voltage_reset == HAS_HIGH_VOLTAGE;
    }
  else
    return false;
}

void Camera::resetHighVoltage(unsigned int sleep_time)
{
  DEB_MEMBER_FUNCT();
  AutoMutex lock(m_cond.mutex());
  if(m_cmd_high_voltage_reset == DONT_HAVE_HIGH_VOLTAGE)
    THROW_HW_ERROR(Error) << "This detector doesn't have a "
			  << "command to reset high voltage";
  std::stringstream msg;
  msg << "resetmodulepower";
  if(sleep_time > 0.) msg << " " << sleep_time;
  m_state = Camera::RESET_HIGH_VOLTAGE;
  send(msg.str());

  m_cond.wait(sleep_time);
  while(m_state == Camera::RESET_HIGH_VOLTAGE)
    {
      m_cond.wait(TIME_OUT);
    }

  if(m_cmd_high_voltage_reset == DONT_HAVE_HIGH_VOLTAGE)
    {
      THROW_HW_ERROR(Error) << "This detector doesn't have a "
			    << "command to reset high voltage";
    }
  else
    {
      m_cmd_high_voltage_reset = HAS_HIGH_VOLTAGE;
    }
  if(m_state == Camera::ERROR)
    {
      m_state = Camera::STANDBY;
      THROW_HW_ERROR(Error) << "Could not reset high voltage";
    }
  else
    {
      /** Refresh the energy settings.
	  Has the chip supply voltage and HV are power cycle,
	  it lost the energy threshold.
       */
      if(m_has_cmd_setenergy)
	send("setenergy");
      else
	send("setthreshold");
    }
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getTemperatureHumidity(std::vector<float>& values)
{
    AutoMutex aLock(m_cond.mutex());
    std::stringstream msg;
    msg << "thread";
    send(msg.str());
    values = m_channel_temperature_humidity_list;
}
