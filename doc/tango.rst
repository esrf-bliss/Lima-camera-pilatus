Pilatus Tango device
====================

This is the reference documentation of the Pilatus Tango device.

you can also find some useful information about the camera models/prerequisite/installation/configuration/compilation in the :ref:`Pilatus camera plugin <camera-pilatus>` section.

Properties
----------

This camera device has no property.

=============== =============== =============== ==============================================================
Property name	Mandatory	Default value	Description
=============== =============== =============== ==============================================================
host_name       No              localhost       Pilatus computer hostname
host_port       No              41234           Pilatus camserver port number
config_file     No              /home/det/      Configuration file path, read to get pilatus version (2 or 3)
                                p2_det/config/  and the camera size (height and width)
                                cam_data/
                                camera.def
tmpfs_path      No              /lima_data      Path to the temporary file-system where camserver will store
                                                the images				
=============== =============== =============== ==============================================================

Attributes
----------
======================= ======= ======================= ============================================================
Attribute name		RW	Type			Description
======================= ======= ======================= ============================================================
threshold_gain		rw	DevString		The detector threshold gain (**LOW,MID,HIGH,ULTRA HIGH,AUTO**)
fill_mode		rw	DevString		The gap fill mode (**ON,OFF**)
threshold		rw	DevLong			The threshold level of detector in eV
energy_threshold	rw	DevFloat		The energy threshold in keV (set the gain and the threshold)
trigger_delay		rw	DevDouble		The start exposure delay after the hard trigger
nb_exposure_per_frame   rw      DevLong                 The number of exposure/frame to set an accumulation of
                                                        frames
temperature_humidity    ro      DevFloat array          List of both constants, a detector can have multiple sensors
sensor_channels         ro      DevShort array          List of sens channels (refer to Dectris documentation)
max_frame_rate          ro      DevLong                 The maximum frame rate the detector can achieve, it can
                                                        can according to the suppored HW ROI
readout_time            ro      DevDouble               The detector readout time in second
======================= ======= ======================= ============================================================

Commands
--------

=======================	=============== =======================	======================================
Command name		Arg. in		Arg. out		Description
=======================	=============== =======================	======================================
Init			DevVoid 	DevVoid			Do not use
State			DevVoid		DevLong			Return the device state
Status			DevVoid		DevString		Return the device state as a string
getAttrStringValueList	DevString:	DevVarStringArray:	Return the authorized string value list for
			Attribute name	String value list	a given attribute name
=======================	=============== =======================	======================================


