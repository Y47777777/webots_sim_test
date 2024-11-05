#ifndef _MODELS_HPP_
#define _MODELS_HPP_

#include <map>
#include <string>
#include <vector>

#include "Parser/parser.hpp"

struct SpecialParam {
    std::string name;
    std::string value;
};

class AcuratorModel {
   protected:
    int start_p_;
    int length_;
    int end_p_;
    bool isNeedParsed_;
    uint8_t *buf_;

   public:
    AcuratorModel();
    virtual ~AcuratorModel();
    int getLength();
    int getEndP();
    bool isNeedParsed();
    void Print();

   public:
    virtual void solveValue(double *output) = 0;
    virtual int setValue(const uint8_t *value) = 0;
    virtual const void *getValue() = 0;

   public:
    virtual void config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length = -1,
                        bool is32bits = false) = 0;
};

class CommonParsedValueModel : public AcuratorModel {
   protected:
    bool isSigned_;
    bool is32bits_;
    int32_t val1_;
    uint32_t val2_;

   public:
    CommonParsedValueModel();
    virtual ~CommonParsedValueModel();
    virtual int setValue(const uint8_t *value);
    virtual const void *getValue();

   public:
    float getFloatValue();
};

class NothingTodoModel : public AcuratorModel {
   public:
    NothingTodoModel();
    virtual ~NothingTodoModel();
    virtual int setValue(const uint8_t *value);
    virtual const void *getValue();
    virtual void solveValue(double *output);
};

///////////////////////////////Accurator//////////////////////////////////

class NullActuator : public NothingTodoModel {
   public:
    NullActuator();
    ~NullActuator();
    virtual void config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length = -1, bool is32bits = false);
};

class MoveDevice : public CommonParsedValueModel {
   private:
    std::map<std::string, double> extra_val_;
    std::vector<float> ForwardPoly_;
    std::vector<float> BackwardPoly_;

   public:
    MoveDevice();
    ~MoveDevice();
    virtual void config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length = -1, bool is32bits = false);
    virtual void solveValue(double *output) override;
};

class SteeringDevice : public CommonParsedValueModel {
   private:
    std::vector<float> Poly_;

   public:
    SteeringDevice();
    ~SteeringDevice();
    virtual void config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length = -1, bool is32bits = false);
    virtual void solveValue(double *output) override;
};

class ForkDevice : public CommonParsedValueModel {
   private:
    std::map<std::string, double> extra_val_;
    std::vector<float> PositivePoly_;
    std::vector<float> NegativePoly_;

   public:
    ForkDevice();
    ~ForkDevice();
    virtual void config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length = -1, bool is32bits = false);
    virtual void solveValue(double *output) override;
};

class LiftDevice : public NothingTodoModel {
   public:
    LiftDevice();
    ~LiftDevice();
    virtual void config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length = -1, bool is32bits = false);
};

class SwitchActuator : public CommonParsedValueModel {
   private:
    std::vector<bool> bitmap_;

   public:
    SwitchActuator();
    ~SwitchActuator();
    virtual void config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length = -1, bool is32bits = false);
    virtual int setValue(const uint8_t *value) override;
    virtual const void *getValue() override;
    virtual void solveValue(double *output) override;
};

class ADataIndex : public CommonParsedValueModel {
   public:
    ADataIndex();
    ~ADataIndex();
    virtual void config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length = -1, bool is32bits = false);
    virtual void solveValue(double *output) override;
};

class ADataIgnore : public NothingTodoModel {
   public:
    ADataIgnore();
    ~ADataIgnore();
    virtual void config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length = -1, bool is32bits = false);
};

class MCUDataIndexReturn : public NothingTodoModel {
   public:
    MCUDataIndexReturn();
    ~MCUDataIndexReturn();
    virtual void config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length = -1, bool is32bits = false);
};

class ValveControleDevice : public NothingTodoModel {
   public:
    ValveControleDevice();
    ~ValveControleDevice();
    virtual void config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length = -1, bool is32bits = false);
};

class SerialDataActuator : public NothingTodoModel {
   public:
    SerialDataActuator();
    ~SerialDataActuator();
    virtual void config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length = -1, bool is32bits = false);
};

class GPIOSwitchActuator : public NothingTodoModel {
   public:
    GPIOSwitchActuator();
    ~GPIOSwitchActuator();
    virtual void config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length = -1, bool is32bits = false);
};

class CanToWifiActuator : public NothingTodoModel {
   public:
    CanToWifiActuator();
    ~CanToWifiActuator();
    virtual void config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length = -1, bool is32bits = false);
};

/////////////////////////////////Accurator///////////////////////////////////
/////////////////////////////////Sencer///////////////////////////////////
class SensorModel {
   protected:
    int start_p_;
    int length_;
    int end_p_;
    std::string type_;
    bool isNeedParsed_;
    uint8_t *buf_ = nullptr;

   public:
    SensorModel();
    virtual ~SensorModel();

   public:
    int getLength();
    const uint8_t *getbuf();
    int getStartP();
    int getEndP();
    bool isNeedParsed();
    void SencerModelConfig();
    void Print();
    std::string getType();

   protected:
    void _setType(bool isSigned = false);

   public:
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false) = 0;

   public:
    // virtual const void* getSolvedValue() = 0;
    virtual void solveValue(const std::vector<double> &input,
                            std::vector<double> &output) = 0;
    virtual int setValue(const std::vector<struct UpdateValue *> &val) = 0;
};

class CommonParsedSencerModel : public SensorModel {
   protected:
    bool isSigned_;
    bool is32bits_;

   public:
    CommonParsedSencerModel();
    virtual ~CommonParsedSencerModel();
    virtual int setValue(const std::vector<struct UpdateValue *> &val);
    // virtual const void* getSolvedValue();
};

class CommonNothingTodoSencerModel : public SensorModel {
   public:
    CommonNothingTodoSencerModel();
    virtual ~CommonNothingTodoSencerModel();
    virtual void solveValue(const std::vector<double> &input,
                            std::vector<double> &output);
    virtual int setValue(const std::vector<struct UpdateValue *> &val);
    // virtual const void* getSolvedValue();
};

////////////////////////////////////Sensor////////////////////////////////////
class DataHeader : public CommonNothingTodoSencerModel {
   public:
    DataHeader();
    ~DataHeader();
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);
};

class SDataIgnore : public CommonNothingTodoSencerModel {
   public:
    SDataIgnore();
    ~SDataIgnore();
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);
};

class WheelCoder : public CommonParsedSencerModel {
   private:
    std::map<std::string, double> extra_val_;

   public:
    WheelCoder();
    ~WheelCoder();
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);

   public:
    virtual void solveValue(const std::vector<double> &input,
                            std::vector<double> &output);
    virtual int setValue(const std::vector<struct UpdateValue *> &val) override;
};

class BatterySencer : public CommonParsedSencerModel {
   public:
    BatterySencer();
    ~BatterySencer();
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);
    virtual void solveValue(const std::vector<double> &input,
                            std::vector<double> &output);
};

class IncrementalSteeringCoder : public CommonParsedSencerModel {
   private:
    std::map<std::string, double> extra_val_;
    std::map<std::string, std::string> str_val_;

   public:
    IncrementalSteeringCoder();
    ~IncrementalSteeringCoder();
    virtual void solveValue(const std::vector<double> &input,
                            std::vector<double> &output) override;
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);
};

class Gyroscope : public CommonParsedSencerModel {
   private:
    std::map<std::string, double> extra_val_;

   public:
    Gyroscope();
    ~Gyroscope();
    virtual void solveValue(const std::vector<double> &input,
                            std::vector<double> &output) override;
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);
};
class ElePerceptionCameraDistance : public CommonParsedSencerModel {
   private:
    std::map<std::string, double> extra_val_;

   public:
    ElePerceptionCameraDistance();
    ~ElePerceptionCameraDistance();
    virtual void solveValue(const std::vector<double> &input,
                            std::vector<double> &output) override;
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);
};
class SDataIndex : public CommonParsedSencerModel {
   public:
    SDataIndex();
    ~SDataIndex();
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);
    virtual void solveValue(const std::vector<double> &input,
                            std::vector<double> &output);
};

class ForkDisplacementSencer : public CommonParsedSencerModel {
   private:
    std::map<std::string, double> extra_val_;

   public:
    ForkDisplacementSencer();
    ~ForkDisplacementSencer();
    virtual void solveValue(const std::vector<double> &input,
                            std::vector<double> &output) override;
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);
};

class HeightCoder : public CommonParsedSencerModel {
   private:
    std::map<std::string, double> extra_val_;

   public:
    HeightCoder();
    ~HeightCoder();
    virtual void solveValue(const std::vector<double> &input,
                            std::vector<double> &output) override;
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);
};

class DataIndexReturn : public CommonParsedSencerModel {
   public:
    DataIndexReturn();
    ~DataIndexReturn();
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);
    virtual void solveValue(const std::vector<double> &input,
                            std::vector<double> &output) override;
};

class NullSencer : public CommonNothingTodoSencerModel {
   public:
    NullSencer();
    ~NullSencer();
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);
};

class ErrorCode : public CommonNothingTodoSencerModel {
   public:
    ErrorCode();
    ~ErrorCode();
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);
};

class RPMSensor : public CommonParsedSencerModel {
   private:
    double Magnification_;

   public:
    RPMSensor();
    ~RPMSensor();
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);
    virtual void solveValue(const std::vector<double> &input,
                            std::vector<double> &output) override;
};

class VelocityControlLevel : public CommonNothingTodoSencerModel {
   public:
    VelocityControlLevel();
    ~VelocityControlLevel();
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);
};

class SwitchSencer : public SensorModel {
   public:
    SwitchSencer();
    ~SwitchSencer();
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);
    virtual void solveValue(const std::vector<double> &input,
                            std::vector<double> &output);
    virtual int setValue(const std::vector<struct UpdateValue *> &val);
};

class Accelerometer : public CommonParsedSencerModel {
   private:
    std::map<std::string, double> extra_val_;

   public:
    Accelerometer();
    ~Accelerometer();
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);
    virtual void solveValue(const std::vector<double> &input,
                            std::vector<double> &output) override;
};

class AngularVelocitySensor : public CommonParsedSencerModel {
   private:
    std::map<std::string, double> extra_val_;

   public:
    AngularVelocitySensor();
    ~AngularVelocitySensor();
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);
    virtual void solveValue(const std::vector<double> &input,
                            std::vector<double> &output) override;
};

class HydraulicPressureSensor : public CommonParsedSencerModel {
   private:
    std::map<std::string, double> extra_val_;

   public:
    HydraulicPressureSensor();
    ~HydraulicPressureSensor();
    virtual void solveValue(const std::vector<double> &input,
                            std::vector<double> &output) override;
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);
};

class DataCRC : public CommonParsedSencerModel {
   public:
    DataCRC();
    ~DataCRC();
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);
    virtual void solveValue(const std::vector<double> &input,
                            std::vector<double> &output) override;
};

class DataTail : public CommonNothingTodoSencerModel {
   public:
    DataTail();
    ~DataTail();
    virtual void config(const std::vector<struct SpecialParam> &param, int last,
                        int length = -1, bool is32bits = false,
                        bool isSigned = false);
};

////////////////////////////////////Sensor////////////////////////////////////
#endif