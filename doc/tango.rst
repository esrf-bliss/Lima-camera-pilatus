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
TmpfsSize       No              0               OBSOLETE
=============== =============== =============== ==============================================================

Attributes
----------
======================= ======= ======================= ============================================================
Attribute name		RW	Type			Description
======================= ======= ======================= ============================================================
threshold_gain		rw	DevString		The detector threshold gain (**LOW,MID,HIGH,ULTRA HIGH**)
fill_mode		rw	DevString		The gap fill mode (**ON,OFF**)
threshold		rw	DevLong			The threshold level of detector in eV
energy_threshold	rw	DevFloat		The energy threshold in keV (set the gain and the threshold)
trigger_delay		rw	DevDouble		The start exposure delay after the hard trigger
nb_exposure_per_frame   rw      DevLong                 The number of exposure/frame to set an accumulation of frames                                                                                                Very useful to not saturate the pixel counters.
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


