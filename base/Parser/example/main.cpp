#include <stdint.h>
#include <stdio.h>

#include <string>

#include "Parser/parser.hpp"

const char *root_path = "/home/visionnav";

int main(void) {
    // std::string path_vnp_decode = std::string(root_path) +
    // "/workspace/vn_source/base/Parser/example/Actuators.config"; InputDecoder
    // decoder; decoder.loadConfig(path_vnp_decode.c_str());
    // // InputDecoder decoder2;
    // //
    // decoder.load("/home/vision/workspace/vn_source/base/Phaser/build/example/Sencers.config");
    // uint8_t sample_code [] = {\
    //     //
    //     AA-00-00-00-AE-00-00-00-00-00-00-00-00-00-00-00-00-04-00-FF-03-00-00-00-00-02-00-00-00-7F-BF-10-84-55
    //     0xAA,
    //     0x00,
    //     0x00,
    //     0xFF,
    //     0xC0,
    //     0x00,
    //     0x00,
    //     0x00,
    //     0x00,
    //     0x00,
    //     0x00,
    //     0x00,
    //     0x00,
    //     0x00,
    //     0x00,
    //     0x00,
    //     0x00,
    //     0x04,
    //     0x00,
    //     0xFF,
    //     0x03,
    //     0x00,
    //     0x00,
    //     0x00,
    //     0x00,
    //     0x02,
    //     0x00,
    //     0x00,
    //     0x00,
    //     0x7F,
    //     0xBF,
    //     0x10,
    //     0x84,
    //     0x55
    // };
    // struct Package pack = {sample_code, 34};
    // decoder.decodePackage(&pack);
    // // uint32_t index = 0;
    // // int DataIndex_A_Handle = decoder.getHandle("DataIndex");
    // // printf("DataIndex_A_Handle handle = [%d]\n", DataIndex_A_Handle);
    // // //decoder.getValue("DataIndex",&index);
    // // decoder.getValue(DataIndex_A_Handle, &index);
    // // printf("DataIndex = [%u]\n", index)；
    // std::string path_vnp_encode = std::string(root_path) +
    // "/workspace/vn_source/base/Parser/example/Sencers.config"; OutputEncoder
    // encoder; encoder.loadConfig(path_vnp_encode.c_str());
    // // int BatterySensor_handler = encoder.getHandle("BatterySencer");
    // // // int Gyroscope_handler = encoder.getHandle("Gyroscope");
    // // int DataIndexReturn_handler = encoder.getHandle("DataIndexReturn");
    // // int DataIndex_handler = encoder.getHandle("DataIndex");
    // // printf("BatterySensor_handler = [%d]\n", BatterySensor_handler);
    // // printf("DataIndexReturn_handler = [%d]\n", DataIndexReturn_handler);
    // // printf("DataIndex_handler = [%d]\n", DataIndex_handler);
    // // uint16_t battery = 100;
    // // struct UpdateValue b_val = {&battery,0};
    // // std::vector<struct UpdateValue*> va_list;
    // // va_list.push_back(&b_val);
    // // encoder.updateValue(BatterySensor_handler, va_list);
    // // va_list.clear();
    // // uint32_t dataIndex = 10672;
    // // struct UpdateValue dataIndex_val = {&dataIndex,0};
    // // va_list.push_back(&dataIndex_val);
    // // encoder.updateValue(DataIndexReturn_handler, va_list);
    // // va_list.clear();
    // // dataIndex = 64772;
    // // dataIndex_val = {&dataIndex,0};
    // // va_list.push_back(&dataIndex_val);
    // // encoder.updateValue(DataIndex_handler, va_list);
    // // va_list.clear();
    // // printf("OK --> next\n");
    // // const struct Package* e_pack = encoder.encodePackage();
    // // if(e_pack == nullptr){
    // //     printf("OK --> out\n");
    // //     return 1;
    // // }
    // // printf("[");
    // // for(int i = 0; i < e_pack->len; i++){
    // //     printf("%.2X ", e_pack->buf[i]);
    // // }
    // // printf("]\n");
    // // TEST ACURATOR solveValue
    // int16_t SteerVal = 0;
    // double real_SteerVal = 0;
    // // int SteeringDevice_Handle = decoder.getHandle("SteeringDevice");
    // // printf("DataIndex_A_Handle handle = [%d]\n", SteeringDevice_Handle);
    // // decoder.getValue(SteeringDevice_Handle, &SteerVal);
    // decoder.getValue("SteeringDevice", &SteerVal, &real_SteerVal);
    // printf("SteerVal = [%d], real_SteerVal = [%lf]\n", SteerVal,
    // real_SteerVal);
    // // decoder.solveValue(SteeringDevice_Handle, SteerVal, &real_SteerVal);
    // // printf("real_SteerVal = [%f]\n", real_SteerVal);
    // int16_t ForkDeviceZ = 0;
    // double real_ForkDeviceZ = 0;
    // // int ForkDeviceZ_Handle = decoder.getHandle("ForkDeviceZ");
    // // printf("ForkDeviceZ handle = [%d]\n", ForkDeviceZ_Handle);
    // // decoder.getValue(ForkDeviceZ_Handle, &ForkDeviceZ);
    // // printf("ForkDeviceZ = [%d]\n", ForkDeviceZ);
    // // decoder.solveValue(ForkDeviceZ_Handle, ForkDeviceZ,
    // &real_ForkDeviceZ);
    // // printf("real_ForkDeviceZ = [%f]\n", real_ForkDeviceZ);
    // decoder.getValue("ForkDeviceZ", &ForkDeviceZ, &real_ForkDeviceZ);
    // printf("SteerVal = [%d], real_SteerVal = [%lf]\n", ForkDeviceZ,
    // real_ForkDeviceZ);

    // // // TEST SENSOR solveValue
    // // int WheelCoder_Handle = encoder.getHandle("WheelCoder");
    // // int HeightCoder_Handle = encoder.getHandle("HeightCoder");
    // // int IncrementalSteeringCoder_Handle =
    // encoder.getHandle("IncrementalSteeringCoder");
    // // printf("WheelCoder_Handle = [%d], HeightCoder_Handle = [%d],
    // IncrementalSteeringCoder_Handle = [%d]\n", WheelCoder_Handle,
    // HeightCoder_Handle, IncrementalSteeringCoder_Handle);
    // // struct UpdateValue WheelLeft;
    // // struct UpdateValue WheelRight;
    // // struct UpdateValue Height;
    // // struct UpdateValue IncrementalSteering;
    // // std::vector<struct UpdateValue*> WheelCoder;
    // // std::vector<struct UpdateValue*> HeightCoder;
    // // std::vector<struct UpdateValue*> IncrementalSteeringCoder;
    // // std::vector<double> WheelCoder_input;
    // // // std::vector<double> WheelCoder_output;
    // // std::vector<double> HeightCoder_input;
    // // // std::vector<double> HeightCoder_output;
    // // std::vector<double> IncrementalSteeringCoder_input;
    // // std::vector<struct UpdateValue*> fake_direct_input;
    // // // std::vector<double> IncrementalSteeringCoder_output;
    // // // WheelCoder.push_back(&WheelLeft);
    // // // WheelCoder.push_back(&WheelRight);
    // // // HeightCoder.push_back(&Height);
    // // // IncrementalSteeringCoder.push_back(&IncrementalSteering);
    // // WheelCoder_input.push_back(64.1961);
    // // WheelCoder_input.push_back(63.1695);
    // // // WheelCoder_output.push_back(0);
    // // // WheelCoder_output.push_back(0);
    // // HeightCoder_input.push_back(1.4);
    // // // HeightCoder_output.push_back(0);
    // // IncrementalSteeringCoder_input.push_back(0);
    // // // IncrementalSteeringCoder_output.push_back(0);
    // // // encoder.solveValue(WheelCoder_Handle, WheelCoder_input,
    // WheelCoder_output);
    // // // encoder.solveValue(HeightCoder_Handle, HeightCoder_input,
    // HeightCoder_output);
    // // // encoder.solveValue(IncrementalSteeringCoder_Handle,
    // IncrementalSteeringCoder_input, IncrementalSteeringCoder_output);
    // // // WheelCoder[0]->val = &WheelCoder_output[0];
    // // // WheelCoder[0]->subId = 0;
    // // // WheelCoder[1]->val = &WheelCoder_output[1];
    // // // WheelCoder[1]->subId = 1;
    // // // HeightCoder[0]->val = &HeightCoder_output[0];
    // // // HeightCoder[0]->subId = 0;
    // // // IncrementalSteeringCoder[0]->val =
    // &IncrementalSteeringCoder_output[0];
    // // // IncrementalSteeringCoder[0]->subId = 0;
    // // encoder.updateValue("WheelCoder", WheelCoder_input,
    // fake_direct_input);
    // // encoder.updateValue("HeightCoder", HeightCoder_input,
    // fake_direct_input);
    // // encoder.updateValue("IncrementalSteeringCoder",
    // IncrementalSteeringCoder_input, fake_direct_input);
    // // double value = strtod("1.47201982991899E-06", nullptr);
    // // printf("value = [%.20lf]\n", value);
    //             double _vehicleYaw = 3.14009;
    //             double _steerYaw = 0;
    //             double mWheelCoder_wheel_l_code = 64.1961;
    //             double mWheelCoder_wheel_r_code = 63.1965;
    //             double _forkHeight = 0.02;
    //             uint32_t dataIdx= 0;
    //             uint32_t dataidx_upload = 405;
    //                 uint16_t battery_device = 100;
    //             uint32_t d_upload = dataidx_upload++;
    //             struct UpdateValue battery_sencer_value = {&battery_device,
    //             sizeof(uint16_t)}; struct UpdateValue dataIdx_value =
    //             {&dataIdx, sizeof(uint32_t)}; struct UpdateValue
    //             dataIdx_upload_value = {&d_upload, sizeof(uint32_t)};
    //             std::vector<double> incrementalCoder_cal_input;
    //             std::vector<double> GyroScope_cal_input;
    //             std::vector<double> WheelCoder_cal_input;
    //             std::vector<double> forkHeight_cal_input;
    //             std::vector<double> fake_cal_input;
    //             std::vector<struct UpdateValue*> fake_direct_input;
    //             std::vector<struct UpdateValue*> battery_sencer_input;
    //             std::vector<struct UpdateValue*> dataIdx_input;
    //             std::vector<struct UpdateValue*> dataIdx_upload_input;
    //             incrementalCoder_cal_input.push_back(_steerYaw);
    //             GyroScope_cal_input.push_back(_vehicleYaw);
    //             WheelCoder_cal_input.push_back(mWheelCoder_wheel_l_code); //
    //             left
    //             WheelCoder_cal_input.push_back(mWheelCoder_wheel_r_code);
    //             // right forkHeight_cal_input.push_back(_forkHeight);
    //             battery_sencer_input.push_back(&battery_sencer_value);
    //             dataIdx_input.push_back(&dataIdx_value);
    //             dataIdx_upload_input.push_back(&dataIdx_upload_value);
    //             encoder.updateValue("IncrementalSteeringCoder",
    //             incrementalCoder_cal_input, fake_direct_input);
    //             encoder.updateValue("Gyroscope", GyroScope_cal_input,
    //             fake_direct_input); encoder.updateValue("WheelCoder",
    //             WheelCoder_cal_input, fake_direct_input);
    //             encoder.updateValue("HeightCoder", forkHeight_cal_input,
    //             fake_direct_input);
    //             encoder.updateValue("ForkDisplacementSencerZ",
    //             forkHeight_cal_input, fake_direct_input);
    //             encoder.updateValue("BatterySencer", fake_cal_input,
    //             battery_sencer_input); encoder.updateValue("DataIndex",
    //             fake_cal_input, dataIdx_upload_input);
    //             encoder.updateValue("DataIndexReturn", fake_cal_input,
    //             dataIdx_input);
    // const struct Package* e_pack = encoder.encodePackage();
    // if(e_pack == nullptr){
    //     printf("OK --> out\n");
    //     return 1;
    // }
    // printf("Total Size = [%d]\n", e_pack->len);
    // printf("[");
    // for(int i = 0; i < e_pack->len; i++){
    //     printf("%.2X ", e_pack->buf[i]);
    // }
    // printf("]\n");

    // TEST ST
    // InputDecoder decoder_st;
    // std::string path_decode_st =
    //     std::string(root_path) +
    //     "/SimulateV2.0/simulation_world.weijian2/webots_ctrl/base/Parser/"
    //     "example/Actuators_ST.config";
    // decoder_st.loadConfig(path_decode_st.c_str());
    // uint8_t sample_code_ST[] = {
    //     //
    //     AA-00-00-00-00-FF-F1-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-E0-00-00-0
    //     0xAA, 0x00, 0x00, 0x00, 0x00, 0x09, 0xA1, 0x00, 0x00, 0x00, 0x00,
    //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00,
    //     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x00,
    //     0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0xD1, 0x7B, 0x00, 0x0B, 0x7A,
    //     0xC7, 0x7E, 0x7D, 0x55};
    // struct Package pack_st = {sample_code_ST, sizeof(sample_code_ST)};
    // // double nothing = 0;
    // // int16_t SteeringDevice = 0;
    // uint32_t dataIdx;
    // double realSteeringDevice = 0;
    // decoder_st.decodePackage(&pack_st);
    // printf("pack size = %ld\n", sizeof(sample_code_ST));
    // uint8_t switch_sensor[10] = {0xFF};
    // bool bit_set;
    // for (int i = 0; i < 10 * 8; i++) {
    //     decoder_st.getSwitchValue("SwitchActuator", i, &bit_set);
    //     // 00 00 00 E0 00 00 00 00 40 00
    //     printf("%d --> %d\n", bit_set, i);
    // }
    // decoder_st.getValue("SteeringDevice", &realSteeringDevice);
    // printf("SteeringDevice = %f\n", realSteeringDevice);
    // decoder_st.getValue2("DataIndex", &dataIdx, 4);
    // printf("DataIndex = %d\n", dataIdx);
    // printf("[");
    // for (int i = 0; i < 1; i++) { printf("%.2X ", switch_sensor[i]); }
    // printf("]\n");
    // printf("[");
    // int counter = 0;
    // int step = 0;
    // for (int i = 0; i < 10 * 8; i++) {
    //     printf("%d ", (switch_sensor[counter] >> (8 - step - 1)) & 0x01);
    //     step++;
    //     if (step == 8) {
    //         step = 0;
    //         counter++;
    //     }
    // }
    // printf("]\n");
    // std::string path_st_encode =
    //     std::string(root_path) +
    //     "/SimulateV2.0/simulation_world.weijian2/webots_ctrl/base/Parser/"
    //     "example/Sencers_ST.config";
    // OutputEncoder encoder_st;
    // // std::vector<double> nullCalculatedInput;
    // // std::vector<UpdateValue *> nullDirectInput;
    // // std::vector<UpdateValue *> VelocityControlLevelInput;
    // // std::vector<UpdateValue *> SwitchSencerInput;
    // // std::vector<double> AccelerometerXInput;
    // // std::vector<double> AccelerometerYInput;
    // // std::vector<double> AccelerometerZInput;
    // // struct UpdateValue VelocityControl;
    // // std::vector<double> RPMSensorInput;
    // // struct UpdateValue RPMSensor;
    // // struct UpdateValue SwitchSencer[12];
    // uint8_t controlLevel = 0x40;
    // // uint8_t switchValueList[12] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    // //                                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    // double RMPValue = 779 * 0.0634;
    // double AX = 0.00762565;
    // double AY = 0.0344975;
    // double AZ = 9.7804;
    // double gyro = 3.14;
    // uint32_t data_idx = 100;
    // uint32_t ll_data_idx = 100;
    // // VelocityControl.val = &controlLevel;
    // // VelocityControl.len = 1;
    // // VelocityControlLevelInput.push_back(&VelocityControl);
    // // RPMSensorInput.push_back(RMPValue);
    // // SwitchSencerInput.resize(12);
    // // switchValueList[4] ^= 0x80;
    // // switchValueList[4] ^= 0x40;
    // // for (int i = 0; i < 12; i++) {
    // //     SwitchSencer[i].val = &switchValueList[i];
    // //     SwitchSencer[i].len = 1;
    // //     SwitchSencer[i].subId = i;
    // //     SwitchSencerInput[i] = &SwitchSencer[i];
    // // }
    // // AccelerometerXInput.push_back(AX);
    // // AccelerometerYInput.push_back(AY);
    // // AccelerometerZInput.push_back(AZ);
    // // switchValueList[3] |= 0x00;
    // // switchValueList[4] |= 0x00;
    // encoder_st.loadConfig(path_st_encode.c_str());
    // encoder_st.updateValue2("VelocityControlLevel", &controlLevel, 1);
    // encoder_st.updateValue("Gyroscope", 1, "", gyro);
    // encoder_st.updateValue("RPMSensor", 1, "", RMPValue);
    // encoder_st.updateValue2("DataIndex", &ll_data_idx, 1);
    // encoder_st.updateValue2("DataIndexReturn", &data_idx, 1);
    // encoder_st.updateValue("RPMSensor", 1, "", RMPValue);
    // for (int i = 0; i < 12 * 8; i++) {
    //     if (i < 32 || i > 33)
    //         encoder_st.updateSwitchValue("SwitchSencer", i, true);
    //     else
    //         encoder_st.updateSwitchValue("SwitchSencer", i, false);
    // }
    // // encoder_st.updateSwitchValue("SwitchSencer", 35, true);
    // encoder_st.updateValue("Accelerometer", 1, "X", AX);
    // encoder_st.updateValue("Accelerometer", 1, "Y", AY);
    // encoder_st.updateValue("Accelerometer", 1, "Z", AZ);
    // const Package *st_pack = encoder_st.encodePackage();
    // printf("st_pack len = %d\n", st_pack->len);
    // printf("[");
    // for (int i = 0; i < st_pack->len; i++) { printf("%.2X ",
    // st_pack->buf[i]); } printf("]\n");

    // TEST VNE
    InputDecoder decoder_vne;
    std::string path_decode_vne =
        std::string(root_path) +
        "/SimulateV2.0/simulation_world.weijian2/webots_ctrl/base/Parser/"
        "example/Actuators_VNE40_2.config";
    printf("path_decode_vne = %s\n", path_decode_vne.c_str());
    decoder_vne.loadConfig(path_decode_vne.c_str());
    uint8_t sample_code_VNE[] = {
        // AA-00-00-00-00-FF-F1-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-E0-00-00-0
        0xAA, 0xFF, 0xFF, 0x00, 0x00, 0x09, 0xA1, 0x00, 0x00, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x00,
        0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0xD1, 0x7B, 0x00, 0x0B, 0x7A,
        0xC7, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x7D, 0x55};
    struct Package pack_vne = {sample_code_VNE, sizeof(sample_code_VNE)};
    // double nothing = 0;
    // int16_t SteeringDevice = 0;
    // uint32_t dataIdx;
    double realSteeringDeviceL = 0;
    double realSteeringDeviceR = 0;
    double realMoveDevices = 0;
    uint32_t vne_dataIdx = 0;
    decoder_vne.decodePackage(&pack_vne);
    printf("pack size = %ld\n", sizeof(sample_code_VNE));
    // uint8_t switch_sensor[10] = {0xFF};
    // bool bit_set;
    // for (int i = 0; i < 10 * 8; i++) {
    //     decoder_vne.getSwitchValue("SwitchActuator", i, &bit_set);
    //     // 00 00 00 E0 00 00 00 00 40 00
    //     printf("%d --> %d\n", bit_set, i);
    // }
    decoder_vne.getValue("MoveDevice", &realMoveDevices);  // steer wheel
    printf("MoveDevice = %f\n", realMoveDevices);
    decoder_vne.getValue("SteeringDevice", &realSteeringDeviceL, "LF");
    printf("SteeringDeviceL = %f\n", realSteeringDeviceL);
    decoder_vne.getValue("SteeringDevice", &realSteeringDeviceR, "RF");
    printf("SteeringDeviceR = %f\n", realSteeringDeviceR);
    decoder_vne.getValue2("DataIndex", &vne_dataIdx, 4);
    printf("DataIndex = %d\n", vne_dataIdx);

    OutputEncoder encoder_vne;
    std::string path_encoder_vne =
        std::string(root_path) +
        "/SimulateV2.0/simulation_world.weijian2/webots_ctrl/base/Parser/"
        "example/Sencers_VNE40_2.config";
    printf("path_encode_vne = %s\n", path_encoder_vne.c_str());
    encoder_vne.loadConfig(path_encoder_vne.c_str());
    double steeringAngle = 1.57;
    // double AX = 0.00762565;
    double AX = 9.7804;
    double AY = 0.0344975;
    double AZ = 9.7804;
    double gyro = 3.14;
    uint32_t data_idx = 100;
    uint32_t ll_data_idx = 100;
    double forkZ = 1.56;
    double forkC = 0.1;
    double forkY = 0.2;
    double forkP = 0.3;
    double MoveWheel = 10;
    encoder_vne.updateValue2("DataIndex", &ll_data_idx, 1);
    encoder_vne.updateValue2("DataIndexReturn", &data_idx, 1);
    encoder_vne.updateValue("WheelCoder", 2, "", MoveWheel, MoveWheel);
    encoder_vne.updateValue("IncrementalSteeringCoder", 1, "LF", steeringAngle);
    encoder_vne.updateValue("IncrementalSteeringCoder", 1, "RF", steeringAngle);
    encoder_vne.updateValue("Accelerometer", 1, "X", AX);
    encoder_vne.updateValue("Accelerometer", 1, "Y", AY);
    encoder_vne.updateValue("Accelerometer", 1, "Z", AZ);
    encoder_vne.updateValue("ForkDisplacementSencer", 1, "Y", forkY);
    encoder_vne.updateValue("ForkDisplacementSencer", 1, "C", forkC);
    encoder_vne.updateValue("ForkDisplacementSencer", 1, "P", forkP);
    encoder_vne.updateValue("HeightCoder", 1, "", forkZ);
    const Package *vne_pack = encoder_vne.encodePackage();
    printf("[");
    for (int i = 0; i < vne_pack->len; i++) {
        printf("%.2X ", vne_pack->buf[i]);
    }
    printf("]\n");
    return 0;
}