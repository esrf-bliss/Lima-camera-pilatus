.. _camera-pilatus:

Dectris Pilatus
---------------

.. image:: Pilatus6M.jpg

Intoduction
```````````
The PILATUS detector (pixel apparatus for the SLS) is a novel type of a x-ray detector, which has been developed at the Paul Scherrer Institut (PSI) for the Swiss Light Source (SLS). PILATUS detectors are two-dimensional hybrid pixel array detectors, which operate in single-photon counting mode. A hybrid pixel that features single photon counting, comprises a preamplifier, a comparator and a counter. The preamplifier enforces the charge generated in the sensor by the incoming x-ray; the comparator produces a digital signal if the incoming charge exceeds a predefined threshold and thus, together with the counter, one obtains a complete digital storage and read-out of the number of detected x-rays per pixel without any read-out noise or dark current!

PILATUS detectors feature several advantages compared to current state-of-the-art CCD and imaging plate detectors. The main features include: no readout noise, superior signal-to-noise ratio, read-out time of 5 ms, a dynamic range of 20bit, high detective quantum efficiency and the possibility to suppress fluorescence by a energy threshold that is set individually for each pixel. A more complete comparison is given in Table 1. The short readout and fast framing time allow to take diffraction data in continuous mode without opening and closing the shutter for each frame (see Fig. 1). For a comparison on the response to x-rays of integrating and single photon counting detectors see Fig. 2.

Because of the specified properties, PILATUS detectors are superiour to state-of-the-art CCD and imaging plate detectors for various x-ray detection experiments. Major improvements can be expected for time-resolved experiments, for the study of weak diffraction phenomena (e.g. diffuse scattering), for accurate measurements of Bragg intensities, for resonant scattering experiments,...

Module configuration
````````````````````

Follow the generic instructions in :ref:`build_installation`. If using CMake directly, add the following flag:

.. code-block:: sh

 -DLIMACAMERA_PILATUS=true

For the Tango server installation, refers to :ref:`tango_installation`.

Installation
````````````

On Pilatus PC, create **as root** a ramdisk of 8GB which will be used by Lima dserver as temporary buffer:

    * edit file ``/etc/fstab`` and add the following line:

    .. code-block:: sh

      none                 /lima_data tmpfs    size=8g,mode=0777      0 0

    * make the directory:

    .. code-block:: sh

      mkdir /lima_data

    * and finally mount the ramdisk:

    .. code-block:: sh

      mount -a

- For Pilatus3, edit file ``~det/p2_det/config/cam_data/camera.def`` and add thoses two lines:

    * camera_wide = WIDTH_OF_THE_DETECTOR
    * camera_high = HEIGHT_OF_THE_DETECTOR

Start the system
````````````````

- Log on the detector pc as *det* user start tvx/camserver:

  .. code-block:: sh

    cd p2_det
    ./runtvx

- when tvx has finished initializing camserver just type *quit* in tvx window

- Log on the detector pc as *an other user* or *det*

  .. code-block:: sh

    cd WHERE_YOU_HAVE_INSTALL_PILATUS_TANGO_SERVER
    TANGO_HOST=Host:Port python LimaCCD.py instance_name

If the cameserver window notice a connection, seams to work ;)

How to use
``````````

This is a python code example for a simple test:

.. code-block:: python

  from Lima import Pilatus
  from Lima import Core

  cam = Pilatus.Camera()
  hwint = Pilatus.Interface(cam)
  ct = Core.CtControl(hwint)

  acq = ct.acquisition()

  # set some low level configuration
  cam.setThresholdGain(1)
  cam.setFillMode(True)
  cam.setEnergy(16.0)
  cam.setHardwareTriggerDelay(0)
  cam.setNbExposurePerFrame(1)

  # setting new file parameters and autosaving mode
  saving=ct.saving()

  pars=saving.getParameters()
  pars.directory='/buffer/lcb18012/opisg/test_lima'
  pars.prefix='test1_'
  pars.suffix='.edf'
  pars.fileFormat=Core.CtSaving.EDF
  pars.savingMode=Core.CtSaving.AutoFrame
  saving.setParameters(pars)

  # now ask for 2 sec. exposure and 10 frames
  acq.setAcqExpoTime(2)
  acq.setAcqNbFrames(10)

  ct.prepareAcq()
  ct.startAcq()

  # wait for last image (#9) ready
  lastimg = ct.getStatus().ImageCounters.LastImageReady
  while lastimg !=9:
    time.sleep(1)
    lastimg = ct.getStatus().ImageCounters.LastImageReady

  # read the first image
  im0 = ct.ReadImage(0)
