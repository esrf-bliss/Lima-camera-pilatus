namespace Pilatus
{
  class Camera
  {
%TypeHeaderCode
#include <PilatusInterface.h>
#include <PilatusCamera.h>
%End
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
	RESET_HIGH_VOLTAGE
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

    Camera(const std::string& host_name = "localhost",int host_port = 41234, const std::string& config_file = "/home/det/p2_det/config/cam_data/camera.def", const std::string& tmpfs_path = "/lima_data") /KeywordArgs="Optional"/;
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
    Pilatus::Camera::Gain gain() const;
    void setThresholdGain(int threshold,
			  Pilatus::Camera::Gain gain = DEFAULT_GAIN);
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

    int nbAcquiredImages() const;
    void version(int& major /Out/,int& minor /Out/,int& patch /Out/) const;

    bool hasRoiCapability() const;
    void setRoi(const std::string&);

    bool hasHighVoltageReset();
    void resetHighVoltage(unsigned int sleeptime = 1);
    void getTemperatureHumidity(std::vector<float>& values /Out/);
    
  private:
	  Camera(const Pilatus::Camera&);    
   };
};
