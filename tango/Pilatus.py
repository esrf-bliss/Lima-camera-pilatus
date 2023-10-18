############################################################################
# This file is part of LImA, a Library for Image Acquisition
#
# Copyright (C) : 2009-2011
# European Synchrotron Radiation Facility
# BP 220, Grenoble 38043
# FRANCE
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.
############################################################################
#=============================================================================
#
# file :        Pilatus.py
#
# description : Python source for the Pilatus and its commands. 
#                The class is derived from Device. It represents the
#                CORBA servant object which will be accessed from the
#                network. All commands which can be executed on the
#                Pilatus are implemented in this file.
#
# project :     TANGO Device Server
#
# copyleft :    European Synchrotron Radiation Facility
#               BP 220, Grenoble 38043
#               FRANCE
#
#=============================================================================
#          This file is generated by POGO
#    (Program Obviously used to Generate tango Object)
#
#         (c) - Software Engineering Group - ESRF
#=============================================================================
#


import PyTango
import sys

from Lima import Core
from Lima.Server import AttrHelper

#==================================================================
#   Pilatus Class Description:
#
#
#==================================================================


class Pilatus(PyTango.Device_4Impl):

#--------- Add you global variables here --------------------------
    Core.DEB_CLASS(Core.DebModApplication, 'LimaCCDs')

#------------------------------------------------------------------
#    Device constructor
#------------------------------------------------------------------
    def __init__(self,cl, name):
        PyTango.Device_4Impl.__init__(self,cl,name)
        self.init_device()

        self.__FillMode = {'ON':True,
                           'OFF':False}
        self.__ThresholdGain = {'DEFAULT' : 0,
                                'LOW' : 1,
                                'MID' : 2,
                                'HIGH' : 3,
                                'ULTRA HIGH' : 4}

        self.__CamStatus = {'ERROR' : 0,
                            'DISCONNECTED' : 1,
                            'STANDBY' : 2,
                            'SETTING_ENERGY' :3,
                            'SETTING_THRESHOLD' : 4,
                            'SETTING_EXPOSURE' : 5,
                            'SETTING_NB_IMAGE_IN_SEQUENCE' :6,
                            'SETTING_EXPOSURE_PERIOD' :7,
                            'SETTING_HARDWARE_TRIGGER_DELAY' : 8,
                            'SETTING_EXPOSURE_PER_FRAME' : 9,
                            'SETTING_ROI' : 10,
                            'KILL_ACQUISITION' : 11,
                            'RUNNING' : 12,
                            'ANYCMD' : 13
                            }

        self.__ReadoutRoi = {'C60': Core.Roi(0,0,0,0),    #Full frame for 6M (0,0,2463,2527)
                             'C2' : Core.Roi(988,1060,487,407), #C2 for 6M
                             'C18': Core.Roi(494,636,1475,1255) #C18 for 6M
                             }


#------------------------------------------------------------------
#    Device destructor
#------------------------------------------------------------------
    def delete_device(self):
        pass

#------------------------------------------------------------------
#    Device initialization
#------------------------------------------------------------------
    def init_device(self):
        self.set_state(PyTango.DevState.ON)
        self.get_device_properties(self.get_device_class())

#------------------------------------------------------------------
#    getAttrStringValueList command:
#
#    Description: return a list of authorized values if any
#    argout: DevVarStringArray   
#------------------------------------------------------------------
    @Core.DEB_MEMBER_FUNCT
    def getAttrStringValueList(self, attr_name):
        return AttrHelper.get_attr_string_value_list(self, attr_name)

#------------------------------------------------------------------
#    resetHighVoltage command:
#
#    Description: resets high voltage
#    argin: sleep time (s) (-1 to not sleep)
#------------------------------------------------------------------
    @Core.DEB_MEMBER_FUNCT
    def resetHighVoltage(self, sleep_time):
        _PilatusCamera.resetHighVoltage(sleep_time)

#==================================================================
#
#    Pilatus read/write attribute methods
#
#==================================================================

#------------------------------------------------------------------
#    Read threshold_gain attribute
#------------------------------------------------------------------
    def read_threshold_gain(self, attr):
        gain = _PilatusCamera.gain()
        if gain is None:
            gain = "not set"
        else:
            gain = AttrHelper.getDictKey(self.__ThresholdGain,gain)
        attr.set_value(gain)

#------------------------------------------------------------------
#    Write threshold_gain attribute
#------------------------------------------------------------------
    def write_threshold_gain(self, attr):
        data = attr.get_write_value()
        gain = AttrHelper.getDictValue(self.__ThresholdGain,data)
        threshold = _PilatusCamera.threshold()
        _PilatusCamera.setThresholdGain(threshold,gain)

#------------------------------------------------------------------
#    Read threshold attribute
#------------------------------------------------------------------
    def read_threshold(self, attr):
        threshold = _PilatusCamera.threshold()
        attr.set_value(threshold)

#------------------------------------------------------------------
#    Write threshold attribute
#------------------------------------------------------------------
    def write_threshold(self, attr):
        data = attr.get_write_value()
        _PilatusCamera.setThresholdGain(data)

#------------------------------------------------------------------
#    Read energy_threshold attribute
#------------------------------------------------------------------
    def read_energy_threshold(self, attr) :
        energy = _PilatusCamera.energy()
        attr.set_value(energy)

#------------------------------------------------------------------
#    Write energy_threshold attribute
#------------------------------------------------------------------
    def write_energy_threshold(self, attr) :
        energy = attr.get_write_value()

        _PilatusCamera.setEnergy(energy)

#----------------------------------------------------------------------------
#     Read delay attribute
#----------------------------------------------------------------------------
    def read_trigger_delay(self,attr) :
        delay = _PilatusCamera.hardwareTriggerDelay()
        attr.set_value(delay)

#----------------------------------------------------------------------------
#     Write delay attribute
#----------------------------------------------------------------------------
    def write_trigger_delay(self,attr) :
        data = attr.get_write_value()
        delay = data
        
        _PilatusCamera.setHardwareTriggerDelay(delay)

#----------------------------------------------------------------------------
#     Read nb exposure per frame attribute
#----------------------------------------------------------------------------
    def read_nb_exposure_per_frame(self,attr) :
        nb_frames = _PilatusCamera.nbExposurePerFrame()
        attr.set_value(nb_frames)

#----------------------------------------------------------------------------
#     Write nb exposure per frame attribute
#----------------------------------------------------------------------------
    def write_nb_exposure_per_frame(self,attr) :
        data = attr.get_write_value()
        nb_frames = data
        
        _PilatusCamera.setNbExposurePerFrame(nb_frames)

#------------------------------------------------------------------
#    Read gapfill attribute
#------------------------------------------------------------------
    def read_fill_mode(self, attr):
        gapfill = _PilatusCamera.gapfill()
        gapfill = AttrHelper.getDictKey(self.__FillMode,gapfill)
        attr.set_value(gapfill)

#------------------------------------------------------------------
#    Write gapfill attribute
#------------------------------------------------------------------
    def write_fill_mode(self, attr):
        data = attr.get_write_value()
        gapfill = AttrHelper.getDictValue(self.__FillMode,data)
        _PilatusCamera.setGapfill(gapfill)

#------------------------------------------------------------------
#    Read cam_state attribute
#------------------------------------------------------------------
    def read_cam_state(self, attr):
        status = _PilatusCamera.status()
        status = AttrHelper.getDictKey(self.__CamStatus, status)
        attr.set_value(status)

#------------------------------------------------------------------
#    Read readout_geometry attribute
#------------------------------------------------------------------
    def read_readout_geometry(self, attr):
        if _PilatusCamera.hasRoiCapability():
            image = _CtControl.image()
            hw_image = image.getHard()
            roi = hw_image.getRealRoi()
            rmode = AttrHelper.getDictKey(self.__ReadoutRoi, roi)
            attr.set_value(rmode)
        else:
            attr.set_value('UNKNOWN')

#------------------------------------------------------------------
#    Write readout_geometry attribute
#------------------------------------------------------------------
    def write_readout_geometry(self, attr):
        if _PilatusCamera.hasRoiCapability():
            data = attr.get_write_value()
            image = _CtControl.image()
            roi = AttrHelper.getDictValue(self.__ReadoutRoi, data)
            image.setRoi(roi)
#------------------------------------------------------------------
#    Read temperature_humidity attribute
#------------------------------------------------------------------
    def read_temperature_humidity(self, attr):
        temperature, humidity = _PilatusCamera.getTemperatureHumidity()
        attr.set_value([temperature, humidity])
            
#==================================================================
#
#    Pilatus command methods
#
#==================================================================

#------------------------------------------------------------------
#    sendCamserverCommand  command
#------------------------------------------------------------------
    @Core.DEB_MEMBER_FUNCT
    def sendCamserverCmd(self, cmd):
        _PilatusCamera.sendAnyCommand(cmd)

#==================================================================
#
#    PilatusClass class definition
#
#==================================================================
class PilatusClass(PyTango.DeviceClass):

    #    Class Properties
    class_property_list = {
        }

    #    Device Properties
    device_property_list = {
        'host_port' :
        [PyTango.DevInt,
         "detector computer host name",4123],

        'host_name' :
        [PyTango.DevString,
         "detector computer host name","localhost"],

        'config_file' :
        [PyTango.DevString,
         "detector config path to get frame dimensions","/home/det/p2_det/config/cam_data/camera.def"],

        'tmpfs_path' :
        [PyTango.DevString,
         "temporary file-system path to let camserver stores images","/lima_data"],
        }

    #    Command definitions
    cmd_list = {
        'getAttrStringValueList':
        [[PyTango.DevString, "Attribute name"],
         [PyTango.DevVarStringArray, "Authorized String value list"]],
        'sendCamserverCmd':
        [[PyTango.DevString, "Camserver command to send"],
         [PyTango.DevVoid, "None"]],
        'resetHighVoltage':
        [[PyTango.DevLong, "sleep_time (s)"],
         [PyTango.DevVoid, ""]],
        }

    #    Attribute definitions
    attr_list = {
        'threshold_gain':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ_WRITE]],
        'threshold':
            [[PyTango.DevLong,
            PyTango.SCALAR,
            PyTango.READ_WRITE]],
        'energy_threshold':
            [[PyTango.DevFloat,
            PyTango.SCALAR,
            PyTango.READ_WRITE]],
        'fill_mode':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ_WRITE]],
        'trigger_delay':
            [[PyTango.DevDouble,
            PyTango.SCALAR,
            PyTango.READ_WRITE]],
        'nb_exposure_per_frame':
            [[PyTango.DevLong,
            PyTango.SCALAR,
            PyTango.READ_WRITE]],
        'cam_state':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ]],
        'readout_geometry':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ_WRITE]],
        'temperature_humidity':
            [[PyTango.DevFloat,
              PyTango.SPECTRUM,
              PyTango.READ,
              2]]
        }

#------------------------------------------------------------------
#    PilatusClass Constructor
#------------------------------------------------------------------
    def __init__(self, name):
        PyTango.DeviceClass.__init__(self, name)
        self.set_type(name)

#----------------------------------------------------------------------------
# Plugins
#----------------------------------------------------------------------------
from Lima import Pilatus as PilatusAcq

_PilatusInterface = None
_PilatusCamera = None
_CtControl = None

def get_control(**keys) :
    global _PilatusInterface
    global _PilatusCamera
    global _CtControl
    if _PilatusInterface is None:
        host_name = keys.pop('host_name', 'localhost')
        host_port = int(keys.pop('host_port', 41234))
        config_file = keys.pop('config_file', '/home/det/p2_det/config/cam_data/camera.def')
        tmpfs_path = keys.pop('tmpfs_path', '/lima_data')
        _PilatusCamera = PilatusAcq.Camera(host_name = host_name,
                                           host_port = host_port,
                                           config_file = config_file,
                                           tmpfs_path = tmpfs_path)
        _PilatusInterface = PilatusAcq.Interface(_PilatusCamera)
        _CtControl = Core.CtControl(_PilatusInterface)
    return _CtControl 

def get_tango_specific_class_n_device() :
    return PilatusClass,Pilatus
