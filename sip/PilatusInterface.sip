namespace Pilatus
{

  class DetInfoCtrlObj: HwDetInfoCtrlObj
  {
%TypeHeaderCode
#include <PilatusInterface.h>
%End
  public:
    struct Info
    {
      Size		m_det_size;
      std::string 	m_det_model;
    };
    DetInfoCtrlObj(Pilatus::Camera& cam, const Pilatus::DetInfoCtrlObj::Info* = NULL);
    virtual ~DetInfoCtrlObj();

    virtual void getMaxImageSize(Size& max_image_size /Out/);
    virtual void getDetectorImageSize(Size& det_image_size /Out/);

    virtual void getDefImageType(ImageType& def_image_type /Out/);
    virtual void getCurrImageType(ImageType& curr_image_type /Out/);
    virtual void setCurrImageType(ImageType curr_image_type);

    virtual void getPixelSize(double& x_size,double &y_size);
    virtual void getDetectorType(std::string& det_type);
    virtual void getDetectorModel(std::string& det_model);

    virtual void registerMaxImageSizeCallback(HwMaxImageSizeCallback& cb);
    virtual void unregisterMaxImageSizeCallback(HwMaxImageSizeCallback& cb);

  private:
    DetInfoCtrlObj(const Pilatus::DetInfoCtrlObj&);
  };

  class Interface: HwInterface
  {
%TypeHeaderCode
#include <PilatusInterface.h>
%End
  public:
    Interface(Pilatus::Camera& cam /KeepReference/,
	      const Pilatus::DetInfoCtrlObj::Info* = NULL);
    virtual ~Interface();

    //- From HwInterface
    //    virtual void getCapList(CapList&) const;
    virtual void	getCapList(std::vector<HwCap> &cap_list /Out/) const;
    virtual void reset(ResetLevel reset_level);
    virtual void prepareAcq();
    virtual void startAcq();
    virtual void stopAcq();
    virtual void getStatus(StatusType& status /Out/);
    virtual int getNbHwAcquiredFrames();

    void setEnergy(double energy);
    double getEnergy();
    void setThresholdGain(int threshold, Pilatus::Camera::Gain gain);
    void setThreshold(int threshold,int energy = -1);
    int getThreshold();
    Pilatus::Camera::Gain getGain();
    void sendAnyCommand(const std::string& str);
    int getMaxFrameRate() const;
    double getReadoutTime() const;
  private:
	  Interface(const Pilatus::Interface&);    
  };
}; // namespace Pilatus
