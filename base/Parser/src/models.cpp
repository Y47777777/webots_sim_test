#include <string.h>

#include <iostream>

#include "models.hpp"
#include "utils.hpp"

AcuratorModel::AcuratorModel() {
  start_p_ = 0;
  length_ = 0;
  end_p_ = 0;
  isNeedParsed_ = false;
  buf_ = nullptr;
}

AcuratorModel::~AcuratorModel() {}

// void AcuratorModel::solveValue(float input, double* output){
//     // Nothing for default base
// }

int AcuratorModel::getLength() {
  return length_;
}

int AcuratorModel::getEndP() {
  return end_p_;
}

bool AcuratorModel::isNeedParsed() {
  return isNeedParsed_;
}

void AcuratorModel::Print() {
  // std::cout << "start_p_ = " << start_p_ << std::endl;
  // std::cout << "length_ = " << length_ << std::endl;
  // std::cout << "endP = " << end_p_ << std::endl;
  // std::cout << "isSigned_ = " << isSigned_ << std::endl;
  // std::cout <<  "is32bits = " << is32bits_ << std::endl;
}

CommonParsedValueModel::CommonParsedValueModel() : AcuratorModel() {
  isSigned_ = false;
}

CommonParsedValueModel::~CommonParsedValueModel() {}

int CommonParsedValueModel::setValue(const uint8_t *value) {
  if (!is32bits_) {
    if (isSigned_) {
      val1_ = asint16_t(value + start_p_, length_);
    } else {
      val2_ = asuint16_t(value + start_p_, length_);
    }
  } else {
    if (isSigned_) {
      val1_ = asint32_t(value + start_p_, length_);
    } else {
      val2_ = asuint32_t(value + start_p_, length_);
    }
  }
  return 0;
}

const void *CommonParsedValueModel::getValue() {
  return (isSigned_ ? (const void *) &val1_ : (const void *) &val2_);
}

float CommonParsedValueModel::getFloatValue() {
  return (isSigned_ ? (float) val1_ : (float) val2_);
}

NothingTodoModel::NothingTodoModel() : AcuratorModel() {}

NothingTodoModel::~NothingTodoModel() {}

int NothingTodoModel::setValue(const uint8_t *value) {
  return 0;
}

void NothingTodoModel::solveValue(double *output) {
  return;
}

const void *NothingTodoModel::getValue() {
  // nothing to be done
  return nullptr;
}

///////////////////////////////Accurator//////////////////////////////////

NullActuator::NullActuator() : NothingTodoModel() {}

NullActuator::~NullActuator() {}

void NullActuator::config(const std::vector<struct SpecialParam> &param,
                          int start_p, int length, bool is32bits) {
  start_p_ = start_p;
  length_ = 2;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
}

MoveDevice::MoveDevice() : CommonParsedValueModel() {}

MoveDevice::~MoveDevice() {}

void MoveDevice::config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length, bool is32bits) {
  start_p_ = start_p;
  length_ = 2;
  is32bits_ = false;
  isSigned_ = true;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  isNeedParsed_ = true;
  for (auto &it : param) {
    if (it.name == "ForwardPoly") {
      std::size_t pos = std::string::npos;
      std::size_t len = it.value.length();
      pos = it.value.find_first_of(",");
      ForwardPoly_.push_back(
          strtof(it.value.substr(0, pos).c_str(), nullptr));  // high
      ForwardPoly_.push_back(
          strtof(it.value.substr(pos + 1, len - pos).c_str(), nullptr));  // low
      continue;
    }
    if (it.name == "BackwardPoly") {
      std::size_t pos = std::string::npos;
      std::size_t len = it.value.length();
      pos = it.value.find_first_of(",");
      BackwardPoly_.push_back(
          strtof(it.value.substr(0, pos).c_str(), nullptr));  // high
      BackwardPoly_.push_back(
          strtof(it.value.substr(pos + 1, len - pos).c_str(), nullptr));  // low
      continue;
    }
    extra_val_[it.name] = strtod(it.value.c_str(), nullptr);
  }
}

void MoveDevice::solveValue(double *output) {
  // use commonParsed one...
  // float t_input = input;
  float input = getFloatValue();
  float t_input = input;
  float high, low;
  double maxCtrl = extra_val_["MaxControl"];
  double minCtrl = extra_val_["MinControl"];
  if (input > 0) {
    high = ForwardPoly_[0];
    low = ForwardPoly_[1];
  } else {
    high = BackwardPoly_[0];
    low = BackwardPoly_[1];
  }
  double ret = 0.;
  if (t_input > maxCtrl)
    t_input = maxCtrl;
  else if (t_input < minCtrl)
    t_input = minCtrl;

  if (t_input == 0.) {
    *output = 0;
    return;
  }
  *output = (double) (t_input - low) / high;
  return;
}

SteeringDevice::SteeringDevice() : CommonParsedValueModel() {}

SteeringDevice::~SteeringDevice() {}

void SteeringDevice::config(const std::vector<struct SpecialParam> &param,
                            int start_p, int length, bool is32bits) {
  start_p_ = start_p;
  length_ = 2;
  isSigned_ = true;
  is32bits_ = false;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  isNeedParsed_ = true;
  for (auto &it : param) {
    if (it.name == "Poly") {
      std::size_t pos = std::string::npos;
      std::size_t len = it.value.length();
      pos = it.value.find_first_of(",");
      Poly_.push_back(
          strtof(it.value.substr(0, pos).c_str(), nullptr));  // high
      Poly_.push_back(
          strtof(it.value.substr(pos + 1, len - pos).c_str(), nullptr));  // low
      continue;
    }
  }
}

void SteeringDevice::solveValue(double *output) {
  float input = getFloatValue();
  float low = Poly_[1];
  float high = Poly_[0];
  *output = (input - low) / high;
  //*output = (input - low) - high;
  return;
}

ForkDeviceZ::ForkDeviceZ() : CommonParsedValueModel() {}

ForkDeviceZ::~ForkDeviceZ() {}

void ForkDeviceZ::config(const std::vector<struct SpecialParam> &param,
                         int start_p, int length, bool is32bits) {
  start_p_ = start_p;
  length_ = 2;
  is32bits_ = false;
  isSigned_ = true;
  if (length != -1)
    length_ = length;
  isNeedParsed_ = true;
  end_p_ = start_p_ + length_ - 1;
  for (auto &it : param) {
    if (it.name == "PositivePoly") {
      std::size_t pos = std::string::npos;
      std::size_t len = it.value.length();
      pos = it.value.find_first_of(",");
      PositivePoly_.push_back(
          strtof(it.value.substr(0, pos).c_str(), nullptr));  // high
      PositivePoly_.push_back(
          strtof(it.value.substr(pos + 1, len - pos).c_str(), nullptr));  // low
      continue;
    }
    if (it.name == "NegativePoly") {
      std::size_t pos = std::string::npos;
      std::size_t len = it.value.length();
      pos = it.value.find_first_of(",");
      NegativePoly_.push_back(
          strtof(it.value.substr(0, pos).c_str(), nullptr));  // high
      NegativePoly_.push_back(
          strtof(it.value.substr(pos + 1, len - pos).c_str(), nullptr));  // low
      continue;
    }
    extra_val_[it.name] = strtod(it.value.c_str(), nullptr);
  }
}

void ForkDeviceZ::solveValue(double *output) {
  float input = getFloatValue();
  float t_input = input;
  float high, low;
  double maxCtrl = extra_val_["MaxControl"];
  double minCtrl = extra_val_["MinControl"];
  if (input >= 0) {
    high = PositivePoly_[0];
    low = PositivePoly_[1];
  } else {
    high = NegativePoly_[0];
    low = NegativePoly_[1];
  }
  double ret = 0.;
  if (t_input >= maxCtrl)
    t_input = maxCtrl;
  else if (t_input <= minCtrl)
    t_input = minCtrl;

  if (t_input == 0.) {
    *output = 0;
    return;
  }
  *output = (double) (t_input - low) / high;
  return;
}

ForkDeviceP::ForkDeviceP() : CommonParsedValueModel() {}

ForkDeviceP::~ForkDeviceP() {}

void ForkDeviceP::config(const std::vector<struct SpecialParam> &param,
                         int start_p, int length, bool is32bits) {
  start_p_ = start_p;
  length_ = 2;
  is32bits_ = false;
  isSigned_ = true;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  isNeedParsed_ = false;
}

void ForkDeviceP::solveValue(double *output) {
  return;
}

LiftDevice::LiftDevice() : NothingTodoModel() {}

LiftDevice::~LiftDevice() {}

void LiftDevice::config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length, bool is32bits) {
  start_p_ = start_p;
  length_ = 2;
  // is32bits_ = false;
  // isSigned_ = false;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
}

SwitchActuator::SwitchActuator() : CommonParsedValueModel() {}

SwitchActuator::~SwitchActuator() {
  if (buf_ != nullptr) {
    delete[] buf_;
  }
}

void SwitchActuator::config(const std::vector<struct SpecialParam> &param,
                            int start_p, int length, bool is32bits) {
  start_p_ = start_p;
  length_ = 10;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  bitmap_.resize(length_ * 8);
  for (auto &it : param) { bitmap_[atoi(it.value.c_str())] = false; }
  if (buf_ == nullptr) {
    buf_ = new uint8_t[length_];
  }
}

int SwitchActuator::setValue(const uint8_t *value) {
  memcpy(buf_, value + start_p_, length_);
  return 0;
}

const void *SwitchActuator::getValue() {
  return buf_;
}

void SwitchActuator::solveValue(double *output) {
  return;
}

ADataIndex::ADataIndex() : CommonParsedValueModel() {}

ADataIndex::~ADataIndex() {}

void ADataIndex::config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length, bool is32bits) {
  start_p_ = start_p;
  length_ = 4;
  is32bits_ = true;
  isSigned_ = false;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
}

void ADataIndex::solveValue(double *output) {
  // Nothing to be parsed
  return;
}

ADataIgnore::ADataIgnore() : NothingTodoModel() {}

ADataIgnore::~ADataIgnore() {}

void ADataIgnore::config(const std::vector<struct SpecialParam> &param,
                         int start_p, int length, bool is32bits) {
  start_p_ = start_p;
  length_ = 2;
  // is32bits_ = false;
  // isSigned_ = false;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
}

MCUDataIndexReturn::MCUDataIndexReturn() : NothingTodoModel() {}

MCUDataIndexReturn::~MCUDataIndexReturn() {}

void MCUDataIndexReturn::config(const std::vector<struct SpecialParam> &param,
                                int start_p, int length, bool is32bits) {
  start_p_ = start_p;
  length_ = 4;
  // is32bits_ = false;
  // isSigned_ = false;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
}

ValveControleDevice::ValveControleDevice() : NothingTodoModel() {}

ValveControleDevice::~ValveControleDevice() {}

void ValveControleDevice::config(const std::vector<struct SpecialParam> &param,
                                 int start_p, int length, bool is32bits) {
  start_p_ = start_p;
  length_ = 0;
  // is32bits_ = false;
  // isSigned_ = false;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
}

SerialDataActuator::SerialDataActuator() : NothingTodoModel() {}

SerialDataActuator::~SerialDataActuator() {}

void SerialDataActuator::config(const std::vector<struct SpecialParam> &param,
                                int start_p, int length, bool is32bits) {
  start_p_ = start_p;
  length_ = 0;
  // is32bits_ = false;
  // isSigned_ = false;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
}

GPIOSwitchActuator::GPIOSwitchActuator() : NothingTodoModel() {}

GPIOSwitchActuator::~GPIOSwitchActuator() {}

void GPIOSwitchActuator::config(const std::vector<struct SpecialParam> &param,
                                int start_p, int length, bool is32bits) {
  start_p_ = start_p;
  length_ = 0;
  // is32bits_ = false;
  // isSigned_ = false;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
}

CanToWifiActuator::CanToWifiActuator() : NothingTodoModel() {}

CanToWifiActuator::~CanToWifiActuator() {}

void CanToWifiActuator::config(const std::vector<struct SpecialParam> &param,
                               int start_p, int length, bool is32bits) {
  start_p_ = start_p;
  length_ = 0;
  // is32bits_ = false;
  // isSigned_ = false;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
}

/////////////////////////////////Accurator///////////////////////////////////
/////////////////////////////////Sencer//////////////////////////////////////

SensorModel::SensorModel() {
  start_p_ = 0;
  length_ = 0;
  end_p_ = 0;
  isNeedParsed_ = false;
  type_ = "undefined";
  // isSigned_ = false;
}

SensorModel::~SensorModel() {
  if (buf_ != nullptr) {
    delete[] buf_;
  }
}

int SensorModel::getLength() {
  return length_;
}

const uint8_t *SensorModel::getbuf() {
  return buf_;
}

int SensorModel::getStartP() {
  return start_p_;
}

int SensorModel::getEndP() {
  return end_p_;
}

bool SensorModel::isNeedParsed() {
  return isNeedParsed_;
}

void SensorModel::SencerModelConfig() {
  if (buf_ == nullptr) {
    buf_ = new uint8_t[length_];
    for (int i = 0; i < length_; i++) { buf_[i] = 0x00; }
  }
}

// void SensorModel::solveValue(const std::vector<double>& input,
// std::vector<double>& output){
//     // Nothing to do with base model
// }

// int SensorModel::setValue(std::vector<struct UpdateValue*>& val){
//     if(!is32bits_){
//         if(isSigned_){
//             const int16_t* l_val = (const int16_t*)(val[0]->val);
//             to2BytesFromint16_t(l_val, buf_);
//         }else{
//             const uint16_t* l_val = (const uint16_t*)(val[0]->val);
//             to2BytesFromuint16_t(l_val, buf_);
//         }
//     }else{
//         if(isSigned_){
//             const int32_t* l_val = (const int32_t*)(val[0]->val);
//             to4BytesFromint32_t(l_val, buf_);
//         }else{
//             const uint32_t* l_val = (const uint32_t*)(val[0]->val);
//             to4BytesFromuint32_t(l_val, buf_);
//         }
//     }
//     return 0;
// }

std::string SensorModel::getType() {
  return type_;
}

void SensorModel::_setType(bool isSigned) {
  if (length_ == 2) {
    type_ = isSigned ? "int16_t" : "uint16_t";
  } else if (length_ == 4) {
    type_ = isSigned ? "int32_t" : "uint32_t";
  } else {
    type_ = "undefined";
  }
}

void SensorModel::Print() {
  // std::cout << "start_p_ = " << start_p_ << std::endl;
  // std::cout << "length_ = " << length_ << std::endl;
  // std::cout << "endP = " << end_p_ << std::endl;
  // std::cout << "isSigned_ = " << isSigned_ << std::endl;
  // std::cout <<  "is32bits = " << is32bits_ << std::endl;
}

CommonParsedSencerModel::CommonParsedSencerModel() : SensorModel() {
  isSigned_ = false;
  is32bits_ = false;
}

CommonParsedSencerModel::~CommonParsedSencerModel() {}

int CommonParsedSencerModel::setValue(
    const std::vector<struct UpdateValue *> &val) {
  if (!is32bits_) {
    if (isSigned_) {
      const int16_t *l_val = (const int16_t *) (val[0]->val);
      to2BytesFromint16_t(l_val, buf_);
    } else {
      const uint16_t *l_val = (const uint16_t *) (val[0]->val);
      to2BytesFromuint16_t(l_val, buf_);
    }
  } else {
    if (isSigned_) {
      const int32_t *l_val = (const int32_t *) (val[0]->val);
      to4BytesFromint32_t(l_val, buf_);
    } else {
      const uint32_t *l_val = (const uint32_t *) (val[0]->val);
      to4BytesFromuint32_t(l_val, buf_);
    }
  }
  return 0;
}

CommonNothingTodoSencerModel::CommonNothingTodoSencerModel() : SensorModel() {}

CommonNothingTodoSencerModel::~CommonNothingTodoSencerModel() {}

int CommonNothingTodoSencerModel::setValue(
    const std::vector<struct UpdateValue *> &val) {
  // Nothing todo with that
  return 0;
}

void CommonNothingTodoSencerModel::solveValue(const std::vector<double> &input,
                                              std::vector<double> &output) {
  return;
}

// const void* CommonNothingTodoSencerModel::getSolvedValue(){return 0;}

////////////////////////////////////Sensor////////////////////////////////////

DataHeader::DataHeader() : CommonNothingTodoSencerModel() {}

DataHeader::~DataHeader() {}

void DataHeader::config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length, bool is32bits, bool isSigned) {
  start_p_ = start_p;
  length_ = 1;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  SensorModel::SencerModelConfig();
  buf_[0] = 0xAA;
}

SDataIgnore::SDataIgnore() : CommonNothingTodoSencerModel() {}

SDataIgnore::~SDataIgnore() {}

void SDataIgnore::config(const std::vector<struct SpecialParam> &param,
                         int start_p, int length, bool is32bits,
                         bool isSigned) {
  start_p_ = start_p;
  length_ = 2;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  SensorModel::SencerModelConfig();
}

WheelCoder::WheelCoder() : CommonParsedSencerModel() {}

WheelCoder::~WheelCoder() {}

void WheelCoder::config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length, bool is32bits, bool isSigned) {
  start_p_ = start_p;
  length_ = 8;  // left, right
  isSigned_ = false;
  is32bits_ = true;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  isNeedParsed_ = true;
  SensorModel::SencerModelConfig();
  SensorModel::_setType(false);
  for (auto &it : param) {
    extra_val_[it.name] = strtod(it.value.c_str(), nullptr);
  }
}

void WheelCoder::solveValue(const std::vector<double> &input,
                            std::vector<double> &output) {
  // 0 is left, 1 is right
  double lscale = extra_val_["LeftScale"];
  double rscale = extra_val_["RightScale"];
  double Magnification = extra_val_["Magnification"];
  output.resize(2);
  output[0] = input[0] / Magnification / lscale;
  output[1] = input[1] / Magnification / rscale;
  // Currently we only know it is four bytes...
  uint32_t l_wheel = (uint32_t) output[0];
  uint32_t r_wheel = (uint32_t) output[1];
  to4BytesFromuint32_t(&l_wheel, buf_);
  to4BytesFromuint32_t(&r_wheel, &buf_[length_ / 2]);
}

int WheelCoder::setValue(const std::vector<struct UpdateValue *> &val) {
  if (!is32bits_) {
    if (isSigned_) {
      const int16_t *l_val = (const int16_t *) (val[0]->val);  // left
      to2BytesFromint16_t(l_val, buf_);
      l_val = (const int16_t *) (val[1]->val);  // right
      to2BytesFromint16_t(l_val, &buf_[length_ / 2]);
    } else {
      const uint16_t *l_val = (const uint16_t *) (val[0]->val);  // left
      to2BytesFromuint16_t(l_val, buf_);
      l_val = (const uint16_t *) (val[1]->val);  // right
      to2BytesFromuint16_t(l_val, &buf_[length_ / 2]);
    }
  } else {
    if (isSigned_) {
      const int32_t *l_val = (const int32_t *) (val[0]->val);  // left
      to4BytesFromint32_t(l_val, buf_);
      l_val = (const int32_t *) (val[1]->val);  // right
      to4BytesFromint32_t(l_val, &buf_[length_ / 2]);
    } else {
      const uint32_t *l_val = (const uint32_t *) (val[0]->val);  // right
      to4BytesFromuint32_t(l_val, buf_);
      l_val = (const uint32_t *) (val[1]->val);  // right
      to4BytesFromuint32_t(l_val, &buf_[length_ / 2]);
    }
  }
  return 0;
}

BatterySencer::BatterySencer() : CommonParsedSencerModel() {}

BatterySencer::~BatterySencer() {}

void BatterySencer::config(const std::vector<struct SpecialParam> &param,
                           int start_p, int length, bool is32bits,
                           bool isSigned) {
  start_p_ = start_p;
  length_ = 2;
  is32bits_ = false;
  isSigned_ = false;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  SensorModel::SencerModelConfig();
  SensorModel::_setType(false);
}

void BatterySencer::solveValue(const std::vector<double> &input,
                               std::vector<double> &output) {
  return;
}

IncrementalSteeringCoder::IncrementalSteeringCoder()
    : CommonParsedSencerModel() {}

IncrementalSteeringCoder::~IncrementalSteeringCoder() {}

void IncrementalSteeringCoder::config(
    const std::vector<struct SpecialParam> &param, int start_p, int length,
    bool is32bits, bool isSigned) {
  start_p_ = start_p;
  length_ = 2;
  is32bits_ = false;
  isSigned_ = true;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  isNeedParsed_ = true;
  SensorModel::SencerModelConfig();
  SensorModel::_setType(true);
  for (auto &it : param) {
    if (it.name == "RareUp") {
      // ST
      str_val_[it.name] = it.value;  // What does this do????
      continue;
    }
    extra_val_[it.name] = strtod(it.value.c_str(), nullptr);
  }
}

void IncrementalSteeringCoder::solveValue(const std::vector<double> &input,
                                          std::vector<double> &output) {
  double steeringZero = extra_val_["SteeringZero"];
  double magnification = extra_val_["Magnification"];
  output.resize(1);
  output[0] = steeringZero + (input[0]) / magnification;
  int16_t incrementalCoder = (int16_t) output[0];
  to2BytesFromint16_t(&incrementalCoder, buf_);
}

Gyroscope::Gyroscope() : CommonParsedSencerModel() {}

Gyroscope::~Gyroscope() {}

void Gyroscope::config(const std::vector<struct SpecialParam> &param,
                       int start_p, int length, bool is32bits, bool isSigned) {
  start_p_ = start_p;
  length_ = 4;
  isSigned_ = true;
  is32bits_ = true;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  isNeedParsed_ = true;
  SensorModel::SencerModelConfig();
  SensorModel::_setType(false);
  for (auto &it : param) {
    extra_val_[it.name] = strtod(it.value.c_str(), nullptr);
  }
}

void Gyroscope::solveValue(const std::vector<double> &input,
                           std::vector<double> &output) {
  // double zeroDrift = extra_val_["ZeroDrift"];
  double magnification = extra_val_["Magnification"];
  output.resize(1);
  output[0] = (input[0]) / magnification;  // No zero drift...
  int32_t gyro = (int32_t) output[0];
  to4BytesFromint32_t(&gyro, buf_);
}

SDataIndex::SDataIndex() : CommonParsedSencerModel() {}

SDataIndex::~SDataIndex() {}

void SDataIndex::config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length, bool is32bits, bool isSigned) {
  start_p_ = start_p;
  length_ = 4;
  isSigned_ = false;
  is32bits_ = true;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  SensorModel::SencerModelConfig();
  SensorModel::_setType(false);
}

void SDataIndex::solveValue(const std::vector<double> &input,
                            std::vector<double> &output) {
  return;
}

ForkDisplacementSencerZ::ForkDisplacementSencerZ()
    : CommonParsedSencerModel() {}

ForkDisplacementSencerZ::~ForkDisplacementSencerZ() {}

void ForkDisplacementSencerZ::config(
    const std::vector<struct SpecialParam> &param, int start_p, int length,
    bool is32bits, bool isSigned) {
  start_p_ = start_p;
  length_ = 2;
  isSigned_ = false;
  is32bits_ = false;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  isNeedParsed_ = true;
  SensorModel::SencerModelConfig();
  SensorModel::_setType(false);
  for (auto &it : param) {
    extra_val_[it.name] = strtod(it.value.c_str(), nullptr);
  }
}

void ForkDisplacementSencerZ::solveValue(const std::vector<double> &input,
                                         std::vector<double> &output) {
  double magnification = extra_val_["Magnification"];
  double zero = extra_val_["Zero"];
  output.resize(1);
  output[0] = input[0] / magnification + zero;
  uint16_t Z = (uint16_t) output[0];
  to2BytesFromuint16_t(&Z, buf_);
}

HeightCoder::HeightCoder() : CommonParsedSencerModel() {}

HeightCoder::~HeightCoder() {}

void HeightCoder::config(const std::vector<struct SpecialParam> &param,
                         int start_p, int length, bool is32bits,
                         bool isSigned) {
  start_p_ = start_p;
  length_ = 4;
  isSigned_ = true;
  is32bits_ = true;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  isNeedParsed_ = true;
  SensorModel::SencerModelConfig();
  SensorModel::_setType(false);
  for (auto &it : param) {
    extra_val_[it.name] = strtod(it.value.c_str(), nullptr);
  }
}

void HeightCoder::solveValue(const std::vector<double> &input,
                             std::vector<double> &output) {
  double magnification = extra_val_["Magnification"];
  double zero = extra_val_["Zero"];
  output.resize(1);
  output[0] = input[0] / magnification + zero;
  int32_t height_coder = (int32_t) output[0];
  to4BytesFromint32_t(&height_coder, buf_);
}

DataIndexReturn::DataIndexReturn() : CommonParsedSencerModel() {}

DataIndexReturn::~DataIndexReturn() {}

void DataIndexReturn::config(const std::vector<struct SpecialParam> &param,
                             int start_p, int length, bool is32bits,
                             bool isSigned) {
  start_p_ = start_p;
  length_ = 4;
  is32bits_ = true;
  isSigned_ = false;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  SensorModel::SencerModelConfig();
  SensorModel::_setType(false);
}

void DataIndexReturn::solveValue(const std::vector<double> &input,
                                 std::vector<double> &output) {
  return;
}

NullSencer::NullSencer() : CommonNothingTodoSencerModel() {}

NullSencer::~NullSencer() {}

void NullSencer::config(const std::vector<struct SpecialParam> &param,
                        int start_p, int length, bool is32bits, bool isSigned) {
  start_p_ = start_p;
  length_ = 2;
  // is32bits_ = false;
  // isSigned_ = false;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  SensorModel::SencerModelConfig();
}

ErrorCode::ErrorCode() : CommonNothingTodoSencerModel() {}

ErrorCode::~ErrorCode() {}

void ErrorCode::config(const std::vector<struct SpecialParam> &param,
                       int start_p, int length, bool is32bits, bool isSigned) {
  // DataCRC: Config remove Length, but default is 4,
  start_p_ = start_p;
  length_ = 4;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  SensorModel::SencerModelConfig();
}

RPMSensor::RPMSensor() : CommonParsedSencerModel() {}

RPMSensor::~RPMSensor() {}

void RPMSensor::config(const std::vector<struct SpecialParam> &param,
                       int start_p, int length, bool is32bits, bool isSigned) {
  start_p_ = start_p;
  length_ = 2;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  isNeedParsed_ = true;
  SensorModel::SencerModelConfig();
  SensorModel::_setType(isSigned);
  for (auto &it : param) {
    if (it.name == "Magnification")
      Magnification_ = strtod(it.value.c_str(), nullptr);
  }
}

void RPMSensor::solveValue(const std::vector<double> &input,
                           std::vector<double> &output) {
  output.resize(1);
  output[0] = input[0] / Magnification_;
  // output[0] = input[0];
  uint16_t motor = (uint16_t) output[0];
  to2BytesFromuint16_t(&motor, buf_);
}

VelocityControlLevel::VelocityControlLevel() : CommonNothingTodoSencerModel() {}

VelocityControlLevel::~VelocityControlLevel() {}

void VelocityControlLevel::config(const std::vector<struct SpecialParam> &param,
                                  int start_p, int length, bool is32bits,
                                  bool isSigned) {
  start_p_ = start_p;
  length_ = 1;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  SensorModel::SencerModelConfig();
  buf_[0] = 0x04;
}

SwitchSencer::SwitchSencer() : SensorModel() {}

SwitchSencer::~SwitchSencer() {}

void SwitchSencer::config(const std::vector<struct SpecialParam> &param,
                          int start_p, int length, bool is32bits,
                          bool isSigned) {
  start_p_ = start_p;
  length_ = 12;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  SensorModel::SencerModelConfig();
}

void SwitchSencer::solveValue(const std::vector<double> &input,
                              std::vector<double> &output) {}

int SwitchSencer::setValue(const std::vector<struct UpdateValue *> &val) {
  int size = val.size();
  // printf("Size = %d\n", size);
  for (int i = 0; i < size; i++) {
    // uint8_t* v = (uint8_t*)(val[i]->val);
    // printf("add %02X\n", *v);
    memcpy(buf_ + val[i]->subId, val[i]->val, val[i]->len);
  }
  return 0;
}

AccelerometerX::AccelerometerX() : CommonParsedSencerModel() {}

AccelerometerX::~AccelerometerX() {}

void AccelerometerX::config(const std::vector<struct SpecialParam> &param,
                            int start_p, int length, bool is32bits,
                            bool isSigned) {
  start_p_ = start_p;
  length_ = 2;
  isSigned_ = true;
  isNeedParsed_ = true;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  SensorModel::SencerModelConfig();
  _setType(isSigned);
  for (auto &it : param) {
    if (it.name == "Zero" || it.name == "Magnification")
      extra_val_[it.name] = strtod(it.value.c_str(), nullptr);
  }
}

void AccelerometerX::solveValue(const std::vector<double> &input,
                                std::vector<double> &output) {
  double magnification = extra_val_["Magnification"];
  double zero = extra_val_["Zero"];
  output.resize(1);
  output[0] = input[0] / magnification + zero;
  int16_t AccelerometerX = (int16_t) output[0];
  to2BytesFromint16_t(&AccelerometerX, buf_);
}

AccelerometerY::AccelerometerY() : CommonParsedSencerModel() {}

AccelerometerY::~AccelerometerY() {}

void AccelerometerY::config(const std::vector<struct SpecialParam> &param,
                            int start_p, int length, bool is32bits,
                            bool isSigned) {
  start_p_ = start_p;
  length_ = 2;
  isSigned_ = true;
  isNeedParsed_ = true;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  SensorModel::SencerModelConfig();
  _setType(isSigned);
  for (auto &it : param) {
    if (it.name == "Zero" || it.name == "Magnification")
      extra_val_[it.name] = strtod(it.value.c_str(), nullptr);
  }
}

void AccelerometerY::solveValue(const std::vector<double> &input,
                                std::vector<double> &output) {
  double magnification = extra_val_["Magnification"];
  double zero = extra_val_["Zero"];
  output.resize(1);
  output[0] = input[0] / magnification + zero;
  int16_t AccelerometerY = (int16_t) output[0];
  to2BytesFromint16_t(&AccelerometerY, buf_);
}

AccelerometerZ::AccelerometerZ() : CommonParsedSencerModel() {}

AccelerometerZ::~AccelerometerZ() {}

void AccelerometerZ::config(const std::vector<struct SpecialParam> &param,
                            int start_p, int length, bool is32bits,
                            bool isSigned) {
  start_p_ = start_p;
  length_ = 2;
  isSigned_ = true;
  isNeedParsed_ = true;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  SensorModel::SencerModelConfig();
  _setType(isSigned);
  for (auto &it : param) {
    if (it.name == "Zero" || it.name == "Magnification")
      extra_val_[it.name] = strtod(it.value.c_str(), nullptr);
  }
}

void AccelerometerZ::solveValue(const std::vector<double> &input,
                                std::vector<double> &output) {
  double magnification = extra_val_["Magnification"];
  double zero = extra_val_["Zero"];
  output.resize(1);
  output[0] = input[0] / magnification + zero;
  int16_t AccelerometerZ = (int16_t) output[0];
  to2BytesFromint16_t(&AccelerometerZ, buf_);
}

AngularVelocitySensorX::AngularVelocitySensorX() : CommonParsedSencerModel() {}

AngularVelocitySensorX::~AngularVelocitySensorX() {}

void AngularVelocitySensorX::config(
    const std::vector<struct SpecialParam> &param, int start_p, int length,
    bool is32bits, bool isSigned) {
  start_p_ = start_p;
  length_ = 2;
  isSigned_ = true;
  isNeedParsed_ = true;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  SensorModel::SencerModelConfig();
  for (auto &it : param) {
    if (it.name == "Zero" || it.name == "Magnification")
      extra_val_[it.name] = strtod(it.value.c_str(), nullptr);
  }
}

void AngularVelocitySensorX::solveValue(const std::vector<double> &input,
                                        std::vector<double> &output) {
  double magnification = extra_val_["Magnification"];
  double zero = extra_val_["Zero"];
  output.resize(1);
  output[0] = input[0] / magnification + zero;
  int16_t AngularVelocitySensorX = (int16_t) output[0];
  to2BytesFromint16_t(&AngularVelocitySensorX, buf_);
}

AngularVelocitySensorY::AngularVelocitySensorY() : CommonParsedSencerModel() {}

AngularVelocitySensorY::~AngularVelocitySensorY() {}

void AngularVelocitySensorY::config(
    const std::vector<struct SpecialParam> &param, int start_p, int length,
    bool is32bits, bool isSigned) {
  start_p_ = start_p;
  length_ = 2;
  isSigned_ = true;
  isNeedParsed_ = true;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  SensorModel::SencerModelConfig();
  for (auto &it : param) {
    if (it.name == "Zero" || it.name == "Magnification")
      extra_val_[it.name] = strtod(it.value.c_str(), nullptr);
  }
}

void AngularVelocitySensorY::solveValue(const std::vector<double> &input,
                                        std::vector<double> &output) {
  double magnification = extra_val_["Magnification"];
  double zero = extra_val_["Zero"];
  output.resize(1);
  output[0] = input[0] / magnification + zero;
  int16_t AngularVelocitySensorY = (int16_t) output[0];
  to2BytesFromint16_t(&AngularVelocitySensorY, buf_);
}

AngularVelocitySensorZ::AngularVelocitySensorZ() : CommonParsedSencerModel() {}

AngularVelocitySensorZ::~AngularVelocitySensorZ() {}

void AngularVelocitySensorZ::config(
    const std::vector<struct SpecialParam> &param, int start_p, int length,
    bool is32bits, bool isSigned) {
  start_p_ = start_p;
  length_ = 2;
  isSigned_ = true;
  isNeedParsed_ = true;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  SensorModel::SencerModelConfig();
  for (auto &it : param) {
    if (it.name == "Zero" || it.name == "Magnification")
      extra_val_[it.name] = strtod(it.value.c_str(), nullptr);
  }
}

void AngularVelocitySensorZ::solveValue(const std::vector<double> &input,
                                        std::vector<double> &output) {
  double magnification = extra_val_["Magnification"];
  double zero = extra_val_["Zero"];
  output.resize(1);
  output[0] = input[0] / magnification + zero;
  int16_t AngularVelocitySensorZ = (int16_t) output[0];
  to2BytesFromint16_t(&AngularVelocitySensorZ, buf_);
}

DataCRC::DataCRC() : CommonParsedSencerModel() {}

DataCRC::~DataCRC() {}

void DataCRC::config(const std::vector<struct SpecialParam> &param, int start_p,
                     int length, bool is32bits, bool isSigned) {
  start_p_ = start_p;
  length_ = 2;
  is32bits_ = false;
  isSigned_ = false;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  SensorModel::SencerModelConfig();
}

void DataCRC::solveValue(const std::vector<double> &input,
                         std::vector<double> &output) {
  return;
}

DataTail::DataTail() : CommonNothingTodoSencerModel() {}

DataTail::~DataTail() {}

void DataTail::config(const std::vector<struct SpecialParam> &param,
                      int start_p, int length, bool is32bits, bool isSigned) {
  start_p_ = start_p;
  length_ = 1;
  // is32bits_ = false;
  // isSigned_ = false;
  if (length != -1)
    length_ = length;
  end_p_ = start_p_ + length_ - 1;
  SensorModel::SencerModelConfig();
  buf_[0] = 0x55;
}

////////////////////////////////////Sensor////////////////////////////////////