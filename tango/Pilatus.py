############################################################################
# This file is part of LImA, a Library for Image Acquisition
#
# Copyright (C) : 2009-2026
# European Synchrotron Radiation Facility
# CS40220 38043 Grenoble Cedex 9
# FRANCE
#
# Contact: lima@esrf.fr
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
#         (c) - Bliss - ESRF
#=============================================================================
#
from tango import AttrWriteType, DevState
from tango.server import Device, attribute, command, device_property

from lima import core
from lima.server import AttrHelper

from Lima import Pilatus as PilatusAcq


class Pilatus(Device):

    core.DEB_CLASS(core.DebModule.DebModApplication, 'LimaCCDs')

    # ------------------------------------------------------------------
    #    Static properties
    # ------------------------------------------------------------------
    _PilatusCamera = None
    """Reference to Lima pilatus camera binding"""
    _PilatusInterface = None
    """Reference to Lima pilatus interface binding"""
    _CtControl = None
    """Reference to the CtControl created for this camera"""

    _FillMode = {'ON': True, 'OFF': False}
    _ThresholdGain = {
        'DEFAULT': 0,
        'LOW': 1,
        'MID': 2,
        'HIGH': 3,
        'ULTRA_HIGH': 4,
        'AUTO': 5,
        'Not supported': None,
    }
    _CamStatus = {
        'ERROR': 0,
        'DISCONNECTED': 1,
        'STANDBY': 2,
        'SETTING_ENERGY': 3,
        'SETTING_THRESHOLD': 4,
        'SETTING_EXPOSURE': 5,
        'SETTING_NB_IMAGE_IN_SEQUENCE': 6,
        'SETTING_EXPOSURE_PERIOD': 7,
        'SETTING_HARDWARE_TRIGGER_DELAY': 8,
        'SETTING_EXPOSURE_PER_FRAME': 9,
        'SETTING_ROI': 10,
        'KILL_ACQUISITION': 11,
        'RUNNING': 12,
        'ANYCMD': 13,
    }
    _ReadoutRoi = {
        'C60': core.Roi(0, 0, 0, 0),  # Full frame for 6M (0,0,2463,2527)
        'C2': core.Roi(988, 1060, 487, 407),  # C2 for 6M
        'C18': core.Roi(494, 636, 1475, 1255),  # C18 for 6M
    }

    # ------------------------------------------------------------------
    #    Device properties
    # ------------------------------------------------------------------
    host_port = device_property(dtype=int, default_value=4123)
    host_name = device_property(dtype=str, default_value="localhost")
    config_file = device_property(
        dtype=str, default_value="/home/det/p2_det/config/cam_data/camera.def"
    )
    tmpfs_path = device_property(dtype=str, default_value="/lima_data")

    # ------------------------------------------------------------------
    #    Device initialization
    # ------------------------------------------------------------------
    @core.DEB_MEMBER_FUNCT
    def init_device(self):
        Device.init_device(self)

        # Leagcy code used by AttrHelper.get_attr_string_value_list
        self.__FillMode = self._FillMode
        self.__ThresholdGain = self._ThresholdGain
        self.__CamStatus = self._CamStatus
        self.__ReadoutRoi = self._ReadoutRoi

        # read only once, will never change
        self._sensor_channels = None

        self.set_state(DevState.ON)

    # ------------------------------------------------------------------
    #    Commands
    # ------------------------------------------------------------------
    @command(dtype_in=str, dtype_out=(str,))
    @core.DEB_MEMBER_FUNCT
    def getAttrStringValueList(self, attr_name):
        return AttrHelper.get_attr_string_value_list(self, attr_name)

    @command(dtype_in=str, doc_in="Camserver command to send")
    @core.DEB_MEMBER_FUNCT
    def sendCamserverCmd(self, cmd):
        self._PilatusCamera.sendAnyCommand(cmd)

    @command(dtype_in=int, doc_in="sleep_time (s)")
    @core.DEB_MEMBER_FUNCT
    def resetHighVoltage(self, sleep_time):
        self._PilatusCamera.resetHighVoltage(sleep_time)

    # ------------------------------------------------------------------
    #    Attributes with custom read/write logic
    # ------------------------------------------------------------------
    def _threshold_gain_fget(self):
        gain = self._PilatusCamera.gain()
        if gain is None:
            return "not set"
        gain = AttrHelper.getDictKey(self._ThresholdGain, gain)
        if gain is None:
            gain = "not supported"
        return gain

    def _threshold_gain_fset(self, data):
        gain = AttrHelper.getDictValue(self._ThresholdGain, data)
        threshold = self._PilatusCamera.threshold()
        self._PilatusCamera.setThresholdGain(threshold, gain)

    threshold_gain = attribute(
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        doc="The detector threshold gain (LOW, MID, HIGH, ULTRA_HIGH, AUTO)",
        fget=_threshold_gain_fget,
        fset=_threshold_gain_fset,
    )

    def _threshold_fget(self):
        return self._PilatusCamera.threshold()

    def _threshold_fset(self, data):
        self._PilatusCamera.setThresholdGain(data)

    threshold = attribute(
        dtype=int,
        access=AttrWriteType.READ_WRITE,
        doc="The threshold level of the detector in eV",
        fget=_threshold_fget,
        fset=_threshold_fset,
    )

    def _energy_threshold_fget(self):
        return self._PilatusCamera.energy()

    def _energy_threshold_fset(self, energy):
        self._PilatusCamera.setEnergy(energy)

    energy_threshold = attribute(
        dtype=float,
        access=AttrWriteType.READ_WRITE,
        doc="The energy threshold in keV (sets both the gain and the threshold)",
        fget=_energy_threshold_fget,
        fset=_energy_threshold_fset,
    )

    def _trigger_delay_fget(self):
        return self._PilatusCamera.hardwareTriggerDelay()

    def _trigger_delay_fset(self, delay):
        self._PilatusCamera.setHardwareTriggerDelay(delay)

    trigger_delay = attribute(
        dtype=float,
        access=AttrWriteType.READ_WRITE,
        doc="The start exposure delay after the hardware trigger",
        fget=_trigger_delay_fget,
        fset=_trigger_delay_fset,
    )

    def _nb_exposure_per_frame_fget(self):
        return self._PilatusCamera.nbExposurePerFrame()

    def _nb_exposure_per_frame_fset(self, nb_frames):
        self._PilatusCamera.setNbExposurePerFrame(nb_frames)

    nb_exposure_per_frame = attribute(
        dtype=int,
        access=AttrWriteType.READ_WRITE,
        doc="The number of exposures per frame, to set an accumulation of frames",
        fget=_nb_exposure_per_frame_fget,
        fset=_nb_exposure_per_frame_fset,
    )

    def _fill_mode_fget(self):
        gapfill = self._PilatusCamera.gapfill()
        return AttrHelper.getDictKey(self._FillMode, gapfill)

    def _fill_mode_fset(self, data):
        gapfill = AttrHelper.getDictValue(self._FillMode, data)
        self._PilatusCamera.setGapfill(gapfill)

    fill_mode = attribute(
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        doc="The gap fill mode (ON, OFF)",
        fget=_fill_mode_fget,
        fset=_fill_mode_fset,
    )

    def _cam_state_fget(self):
        status = self._PilatusCamera.status()
        return AttrHelper.getDictKey(self._CamStatus, status)

    cam_state = attribute(
        dtype=str,
        access=AttrWriteType.READ,
        doc="Camserver status (ERROR, DISCONNECTED, STANDBY, SETTING_*, RUNNING, ...)",
        fget=_cam_state_fget,
    )

    def _readout_geometry_fget(self):
        if not self._PilatusCamera.hasRoiCapability():
            return "Not supported"
        image = self._CtControl.image()
        hw_image = image.getHard()
        roi = hw_image.getRealRoi()
        return AttrHelper.getDictKey(self._ReadoutRoi, roi)

    def _readout_geometry_fset(self, data):
        if self._PilatusCamera.hasRoiCapability():
            image = self._CtControl.image()
            roi = AttrHelper.getDictValue(self._ReadoutRoi, data)
            image.setRoi(roi)

    readout_geometry = attribute(
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        doc="Detector readout ROI selection (C60, C2, C18), when the HW ROI capability is available",
        fget=_readout_geometry_fget,
        fset=_readout_geometry_fset,
    )

    def _temperature_humidity_fget(self):
        values = self._PilatusCamera.getTemperatureHumidity()
        del values[0::3]
        return values

    temperature_humidity = attribute(
        dtype=(float,),
        max_dim_x=256,
        access=AttrWriteType.READ,
        label="temperature (C) and humidity (%) for each channel",
        doc="temperature, humidity, temperature, humidity ....",
        fget=_temperature_humidity_fget,
    )

    def _sensor_channels_fget(self):
        if self._sensor_channels is None:
            self._sensor_channels = [
                int(x) for x in self._PilatusCamera.getTemperatureHumidity()[0::3]
            ]
        return self._sensor_channels

    sensor_channels = attribute(
        dtype=(int,),
        max_dim_x=256,
        access=AttrWriteType.READ,
        label="Temperature/humidiy sensor channel numbers",
        doc="<sensor X>, <sensor Y>, ...",
        fget=_sensor_channels_fget,
    )

    # ------------------------------------------------------------------
    #    Attributes fully dispatched by naming convention
    # ------------------------------------------------------------------
    _max_frame_rate_fget, _ = AttrHelper.make_fget_fset(
        "max_frame_rate", lambda self: self._PilatusInterface
    )
    max_frame_rate = attribute(
        dtype=int,
        access=AttrWriteType.READ,
        label="Maximum Frame Rate",
        doc="In Hz",
        fget=_max_frame_rate_fget,
    )

    _readout_time_fget, _ = AttrHelper.make_fget_fset(
        "readout_time", lambda self: self._PilatusInterface
    )
    readout_time = attribute(
        dtype=float,
        access=AttrWriteType.READ,
        label="Readout Time",
        doc="The detector readout time in second",
        fget=_readout_time_fget,
    )


#----------------------------------------------------------------------------
# Plugins
#----------------------------------------------------------------------------

def get_control(_Pilatus=Pilatus, _Camera=PilatusAcq.Camera, _Interface=PilatusAcq.Interface,
                 **keys):
    camera = _Pilatus._PilatusCamera
    interface = _Pilatus._PilatusInterface

    if camera is None:
        host_name = keys.pop('host_name', 'localhost')
        host_port = int(keys.pop('host_port', 41234))
        config_file = keys.pop('config_file', '/home/det/p2_det/config/cam_data/camera.def')
        tmpfs_path = keys.pop('tmpfs_path', '/lima_data')
        camera = _Camera(
            host_name=host_name,
            host_port=host_port,
            config_file=config_file,
            tmpfs_path=tmpfs_path,
        )
        interface = _Interface(camera)

    control = core.CtControl(interface)

    _Pilatus._PilatusCamera = camera
    _Pilatus._PilatusInterface = interface
    _Pilatus._CtControl = control

    return control

def get_tango_specific_class_n_device():
    return Pilatus.TangoClassClass, Pilatus
