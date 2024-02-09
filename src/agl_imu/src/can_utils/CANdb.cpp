#include "agl_imu/can_utils/CANdb.hpp"
#include <iostream>
#include <signal.h>
#include <memory>
#include <cstring>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <bitset>

using namespace can_utils; 
 
uint64_t CANdb::CAN_Frame_Function(CANdb::CAN_msg msg){
    uint64_t frame=0;
    uint64_t frame_aux=0;
    for (auto i=0; i<msg.size; i++){
        frame_aux=msg.data[i];
        frame=frame | (frame_aux<<(8*i));
        frame_aux=0;
    }
    return frame;
}

CANdb::CAN_msg CANdb::CANTx_FrameMount(uint32_t ID2Send){
    CANdb::CAN_msg msg;
    uint64_t Frame_aux = 0;
    if (ID2Send == Aceinna_Address_Claiming_struct->ID){
        Frame_aux = ((uint64_t)Aceinna_Address_Claiming_struct->signal_value.J1939_Name_ID_Code) << Aceinna_Address_Claiming_struct->signal_info.J1939_Name_ID_Code.StartBit;
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Manufacture_Code) << Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Manufacture_Code.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Address_Claiming_struct->signal_value.J1939_Name_ECU_Example) << Aceinna_Address_Claiming_struct->signal_info.J1939_Name_ECU_Example.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Function_Example) << Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Function_Example.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Function) << Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Function.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Reserved0) << Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Reserved0.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Vehicle_System) << Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Vehicle_System.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Vehicle_Example) << Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Vehicle_Example.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Industry_Group) << Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Industry_Group.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Arbitrary_Address) << Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Arbitrary_Address.StartBit);
        msg.id = Aceinna_Address_Claiming_struct->ID;
        msg.size = Aceinna_Address_Claiming_struct->DLC;
        for (auto i=0; i<msg.size ; i++) { 
            msg.data.push_back((Frame_aux & ((uint64_t)0xFF<<(8*i)))>>(8*(i)));
        }
        return msg;
    }
    else if (ID2Send == Aceinna_Rapid_Attitude_struct->ID){
        Frame_aux = ((uint64_t)Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_SID_A) << Aceinna_Rapid_Attitude_struct->signal_info.Aceinna_SID_A.StartBit;
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Yaw) << Aceinna_Rapid_Attitude_struct->signal_info.Aceinna_ATT_Yaw.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Pitch) << Aceinna_Rapid_Attitude_struct->signal_info.Aceinna_ATT_Pitch.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Roll) << Aceinna_Rapid_Attitude_struct->signal_info.Aceinna_ATT_Roll.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Rsvd) << Aceinna_Rapid_Attitude_struct->signal_info.Aceinna_ATT_Rsvd.StartBit);
        msg.id = Aceinna_Rapid_Attitude_struct->ID;
        msg.size = Aceinna_Rapid_Attitude_struct->DLC;
        for (auto i=0; i<msg.size ; i++) { 
            msg.data.push_back((Frame_aux & ((uint64_t)0xFF<<(8*i)))>>(8*(i)));
        }
        return msg;
    }
    else if (ID2Send == Aceinna_Rate_struct->ID){
        Frame_aux = ((uint64_t)Aceinna_Rate_struct->signal_value.Aceinna_GyroX) << Aceinna_Rate_struct->signal_info.Aceinna_GyroX.StartBit;
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Rate_struct->signal_value.Aceinna_GyroY) << Aceinna_Rate_struct->signal_info.Aceinna_GyroY.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Rate_struct->signal_value.Aceinna_GyroZ) << Aceinna_Rate_struct->signal_info.Aceinna_GyroZ.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Rate_struct->signal_value.Aceinna_PitchRate_Figure_OfMerit) << Aceinna_Rate_struct->signal_info.Aceinna_PitchRate_Figure_OfMerit.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Rate_struct->signal_value.Aceinna_RollRate_Figure_OfMerit) << Aceinna_Rate_struct->signal_info.Aceinna_RollRate_Figure_OfMerit.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Rate_struct->signal_value.Aceinna_YawRate_Figure_OfMerit) << Aceinna_Rate_struct->signal_info.Aceinna_YawRate_Figure_OfMerit.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Rate_struct->signal_value.Aceinna_AngleRate_Latency) << Aceinna_Rate_struct->signal_info.Aceinna_AngleRate_Latency.StartBit);
        msg.id = Aceinna_Rate_struct->ID;
        msg.size = Aceinna_Rate_struct->DLC;
        for (auto i=0; i<msg.size ; i++) { 
            msg.data.push_back((Frame_aux & ((uint64_t)0xFF<<(8*i)))>>(8*(i)));
        }
        return msg;
    }
    else if (ID2Send == Aceinna_Accel_struct->ID){
        Frame_aux = ((uint64_t)Aceinna_Accel_struct->signal_value.Aceinna_AccX) << Aceinna_Accel_struct->signal_info.Aceinna_AccX.StartBit;
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Accel_struct->signal_value.Aceinna_AccY) << Aceinna_Accel_struct->signal_info.Aceinna_AccY.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Accel_struct->signal_value.Aceinna_AccZ) << Aceinna_Accel_struct->signal_info.Aceinna_AccZ.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Accel_struct->signal_value.Aceinna_LateralAcc_FigureOfMerit) << Aceinna_Accel_struct->signal_info.Aceinna_LateralAcc_FigureOfMerit.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Accel_struct->signal_value.Aceinna_LongiAcc_FigureOfMerit) << Aceinna_Accel_struct->signal_info.Aceinna_LongiAcc_FigureOfMerit.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Accel_struct->signal_value.Aceinna_VerticAcc_FigureOfMerit) << Aceinna_Accel_struct->signal_info.Aceinna_VerticAcc_FigureOfMerit.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Accel_struct->signal_value.Aceinna_Support_Rate_Acc) << Aceinna_Accel_struct->signal_info.Aceinna_Support_Rate_Acc.StartBit);
        msg.id = Aceinna_Accel_struct->ID;
        msg.size = Aceinna_Accel_struct->DLC;
        for (auto i=0; i<msg.size ; i++) { 
            msg.data.push_back((Frame_aux & ((uint64_t)0xFF<<(8*i)))>>(8*(i)));
        }
        return msg;
    }
    else if (ID2Send == Aceinna_Angles_struct->ID){
        Frame_aux = ((uint64_t)Aceinna_Angles_struct->signal_value.Aceinna_Pitch) << Aceinna_Angles_struct->signal_info.Aceinna_Pitch.StartBit;
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Angles_struct->signal_value.Aceinna_Roll) << Aceinna_Angles_struct->signal_info.Aceinna_Roll.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Angles_struct->signal_value.Aceinna_Pitch_Compensation) << Aceinna_Angles_struct->signal_info.Aceinna_Pitch_Compensation.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Angles_struct->signal_value.Aceinna_Pitch_Figure_OfMerit) << Aceinna_Angles_struct->signal_info.Aceinna_Pitch_Figure_OfMerit.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Angles_struct->signal_value.Aceinna_Roll_Compensation) << Aceinna_Angles_struct->signal_info.Aceinna_Roll_Compensation.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Angles_struct->signal_value.Aceinna_Roll_Figure_OfMerit) << Aceinna_Angles_struct->signal_info.Aceinna_Roll_Figure_OfMerit.StartBit);
        Frame_aux = Frame_aux | (((uint64_t)Aceinna_Angles_struct->signal_value.Aceinna_PitchRoll_Latency) << Aceinna_Angles_struct->signal_info.Aceinna_PitchRoll_Latency.StartBit);
        msg.id = Aceinna_Angles_struct->ID;
        msg.size = Aceinna_Angles_struct->DLC;
        for (auto i=0; i<msg.size ; i++) { 
            msg.data.push_back((Frame_aux & ((uint64_t)0xFF<<(8*i)))>>(8*(i)));
        }
        return msg;
    }
    else{
        std::cout << "Message ID not defined" << std::endl;
        return msg;
    }
}


void CANdb::CANRx_Read(CAN_msg msg) {
    if (msg.id == Aceinna_Address_Claiming_struct->ID) {
        Complete_Frame=CAN_Frame_Function(msg);
        Aceinna_Address_Claiming_struct->timestamp_us = msg.timestamp_us;
        Aceinna_Address_Claiming_struct->signal_value.J1939_Name_ID_Code = (Complete_Frame >> Aceinna_Address_Claiming_struct->signal_info.J1939_Name_ID_Code.StartBit) & 0x1fffff;
        Aceinna_Address_Claiming_struct->signal_value.J1939_Name_ID_Code = (Aceinna_Address_Claiming_struct->signal_value.J1939_Name_ID_Code * Aceinna_Address_Claiming_struct->signal_info.J1939_Name_ID_Code.Factor) + Aceinna_Address_Claiming_struct->signal_info.J1939_Name_ID_Code.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        J1939_Name_ID_Code: " << (int)Aceinna_Address_Claiming_struct->signal_value.J1939_Name_ID_Code<< std::endl;
        #endif
        Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Manufacture_Code = (Complete_Frame >> Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Manufacture_Code.StartBit) & 0x7ff;
        Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Manufacture_Code = (Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Manufacture_Code * Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Manufacture_Code.Factor) + Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Manufacture_Code.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        J1939_Name_Manufacture_Code: " << (int)Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Manufacture_Code<< std::endl;
        #endif
        Aceinna_Address_Claiming_struct->signal_value.J1939_Name_ECU_Example = (Complete_Frame >> Aceinna_Address_Claiming_struct->signal_info.J1939_Name_ECU_Example.StartBit) & 0x7;
        Aceinna_Address_Claiming_struct->signal_value.J1939_Name_ECU_Example = (Aceinna_Address_Claiming_struct->signal_value.J1939_Name_ECU_Example * Aceinna_Address_Claiming_struct->signal_info.J1939_Name_ECU_Example.Factor) + Aceinna_Address_Claiming_struct->signal_info.J1939_Name_ECU_Example.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        J1939_Name_ECU_Example: " << (int)Aceinna_Address_Claiming_struct->signal_value.J1939_Name_ECU_Example<< std::endl;
        #endif
        Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Function_Example = (Complete_Frame >> Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Function_Example.StartBit) & 0x1f;
        Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Function_Example = (Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Function_Example * Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Function_Example.Factor) + Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Function_Example.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        J1939_Name_Function_Example: " << (int)Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Function_Example<< std::endl;
        #endif
        Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Function = (Complete_Frame >> Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Function.StartBit) & 0xff;
        Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Function = (Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Function * Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Function.Factor) + Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Function.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        J1939_Name_Function: " << (int)Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Function<< std::endl;
        #endif
        Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Reserved0 = (Complete_Frame >> Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Reserved0.StartBit) & 0x1;
        Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Reserved0 = (Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Reserved0 * Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Reserved0.Factor) + Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Reserved0.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        J1939_Name_Reserved0: " << (int)Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Reserved0<< std::endl;
        #endif
        Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Vehicle_System = (Complete_Frame >> Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Vehicle_System.StartBit) & 0x7f;
        Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Vehicle_System = (Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Vehicle_System * Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Vehicle_System.Factor) + Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Vehicle_System.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        J1939_Name_Vehicle_System: " << (int)Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Vehicle_System<< std::endl;
        #endif
        Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Vehicle_Example = (Complete_Frame >> Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Vehicle_Example.StartBit) & 0xf;
        Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Vehicle_Example = (Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Vehicle_Example * Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Vehicle_Example.Factor) + Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Vehicle_Example.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        J1939_Name_Vehicle_Example: " << (int)Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Vehicle_Example<< std::endl;
        #endif
        Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Industry_Group = (Complete_Frame >> Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Industry_Group.StartBit) & 0x7;
        Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Industry_Group = (Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Industry_Group * Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Industry_Group.Factor) + Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Industry_Group.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        J1939_Name_Industry_Group: " << (int)Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Industry_Group<< std::endl;
        #endif
        Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Arbitrary_Address = (Complete_Frame >> Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Arbitrary_Address.StartBit) & 0x1;
        Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Arbitrary_Address = (Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Arbitrary_Address * Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Arbitrary_Address.Factor) + Aceinna_Address_Claiming_struct->signal_info.J1939_Name_Arbitrary_Address.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        J1939_Name_Arbitrary_Address: " << (int)Aceinna_Address_Claiming_struct->signal_value.J1939_Name_Arbitrary_Address<< std::endl;
        #endif
    }
    else if (msg.id == Aceinna_Rapid_Attitude_struct->ID) {
        Complete_Frame=CAN_Frame_Function(msg);
        Aceinna_Rapid_Attitude_struct->timestamp_us = msg.timestamp_us;
        Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_SID_A = (Complete_Frame >> Aceinna_Rapid_Attitude_struct->signal_info.Aceinna_SID_A.StartBit) & 0xff;
        Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_SID_A = (Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_SID_A * Aceinna_Rapid_Attitude_struct->signal_info.Aceinna_SID_A.Factor) + Aceinna_Rapid_Attitude_struct->signal_info.Aceinna_SID_A.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_SID_A: " << (int)Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_SID_A<< std::endl;
        #endif
        Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Yaw = (Complete_Frame >> Aceinna_Rapid_Attitude_struct->signal_info.Aceinna_ATT_Yaw.StartBit) & 0xffff;
        Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Yaw = (Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Yaw * Aceinna_Rapid_Attitude_struct->signal_info.Aceinna_ATT_Yaw.Factor) + Aceinna_Rapid_Attitude_struct->signal_info.Aceinna_ATT_Yaw.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_ATT_Yaw: " << (int)Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Yaw<< std::endl;
        #endif
        Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Pitch = (Complete_Frame >> Aceinna_Rapid_Attitude_struct->signal_info.Aceinna_ATT_Pitch.StartBit) & 0xffff;
        Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Pitch = (Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Pitch * Aceinna_Rapid_Attitude_struct->signal_info.Aceinna_ATT_Pitch.Factor) + Aceinna_Rapid_Attitude_struct->signal_info.Aceinna_ATT_Pitch.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_ATT_Pitch: " << (int)Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Pitch<< std::endl;
        #endif
        Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Roll = (Complete_Frame >> Aceinna_Rapid_Attitude_struct->signal_info.Aceinna_ATT_Roll.StartBit) & 0xffff;
        Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Roll = (Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Roll * Aceinna_Rapid_Attitude_struct->signal_info.Aceinna_ATT_Roll.Factor) + Aceinna_Rapid_Attitude_struct->signal_info.Aceinna_ATT_Roll.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_ATT_Roll: " << (int)Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Roll<< std::endl;
        #endif
        Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Rsvd = (Complete_Frame >> Aceinna_Rapid_Attitude_struct->signal_info.Aceinna_ATT_Rsvd.StartBit) & 0xff;
        Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Rsvd = (Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Rsvd * Aceinna_Rapid_Attitude_struct->signal_info.Aceinna_ATT_Rsvd.Factor) + Aceinna_Rapid_Attitude_struct->signal_info.Aceinna_ATT_Rsvd.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_ATT_Rsvd: " << (int)Aceinna_Rapid_Attitude_struct->signal_value.Aceinna_ATT_Rsvd<< std::endl;
        #endif
    }
    else if (msg.id == Aceinna_Rate_struct->ID) {
        Complete_Frame=CAN_Frame_Function(msg);
        Aceinna_Rate_struct->timestamp_us = msg.timestamp_us;
        Aceinna_Rate_struct->signal_value.Aceinna_GyroX = (Complete_Frame >> Aceinna_Rate_struct->signal_info.Aceinna_GyroX.StartBit) & 0xffff;
        Aceinna_Rate_struct->signal_value.Aceinna_GyroX = (Aceinna_Rate_struct->signal_value.Aceinna_GyroX * Aceinna_Rate_struct->signal_info.Aceinna_GyroX.Factor) + Aceinna_Rate_struct->signal_info.Aceinna_GyroX.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_GyroX: " << (int)Aceinna_Rate_struct->signal_value.Aceinna_GyroX<< std::endl;
        #endif
        Aceinna_Rate_struct->signal_value.Aceinna_GyroY = (Complete_Frame >> Aceinna_Rate_struct->signal_info.Aceinna_GyroY.StartBit) & 0xffff;
        Aceinna_Rate_struct->signal_value.Aceinna_GyroY = (Aceinna_Rate_struct->signal_value.Aceinna_GyroY * Aceinna_Rate_struct->signal_info.Aceinna_GyroY.Factor) + Aceinna_Rate_struct->signal_info.Aceinna_GyroY.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_GyroY: " << (int)Aceinna_Rate_struct->signal_value.Aceinna_GyroY<< std::endl;
        #endif
        Aceinna_Rate_struct->signal_value.Aceinna_GyroZ = (Complete_Frame >> Aceinna_Rate_struct->signal_info.Aceinna_GyroZ.StartBit) & 0xffff;
        Aceinna_Rate_struct->signal_value.Aceinna_GyroZ = (Aceinna_Rate_struct->signal_value.Aceinna_GyroZ * Aceinna_Rate_struct->signal_info.Aceinna_GyroZ.Factor) + Aceinna_Rate_struct->signal_info.Aceinna_GyroZ.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_GyroZ: " << (int)Aceinna_Rate_struct->signal_value.Aceinna_GyroZ<< std::endl;
        #endif
        Aceinna_Rate_struct->signal_value.Aceinna_PitchRate_Figure_OfMerit = (Complete_Frame >> Aceinna_Rate_struct->signal_info.Aceinna_PitchRate_Figure_OfMerit.StartBit) & 0x3;
        Aceinna_Rate_struct->signal_value.Aceinna_PitchRate_Figure_OfMerit = (Aceinna_Rate_struct->signal_value.Aceinna_PitchRate_Figure_OfMerit * Aceinna_Rate_struct->signal_info.Aceinna_PitchRate_Figure_OfMerit.Factor) + Aceinna_Rate_struct->signal_info.Aceinna_PitchRate_Figure_OfMerit.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_PitchRate_Figure_OfMerit: " << (int)Aceinna_Rate_struct->signal_value.Aceinna_PitchRate_Figure_OfMerit<< std::endl;
        #endif
        Aceinna_Rate_struct->signal_value.Aceinna_RollRate_Figure_OfMerit = (Complete_Frame >> Aceinna_Rate_struct->signal_info.Aceinna_RollRate_Figure_OfMerit.StartBit) & 0x3;
        Aceinna_Rate_struct->signal_value.Aceinna_RollRate_Figure_OfMerit = (Aceinna_Rate_struct->signal_value.Aceinna_RollRate_Figure_OfMerit * Aceinna_Rate_struct->signal_info.Aceinna_RollRate_Figure_OfMerit.Factor) + Aceinna_Rate_struct->signal_info.Aceinna_RollRate_Figure_OfMerit.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_RollRate_Figure_OfMerit: " << (int)Aceinna_Rate_struct->signal_value.Aceinna_RollRate_Figure_OfMerit<< std::endl;
        #endif
        Aceinna_Rate_struct->signal_value.Aceinna_YawRate_Figure_OfMerit = (Complete_Frame >> Aceinna_Rate_struct->signal_info.Aceinna_YawRate_Figure_OfMerit.StartBit) & 0x3;
        Aceinna_Rate_struct->signal_value.Aceinna_YawRate_Figure_OfMerit = (Aceinna_Rate_struct->signal_value.Aceinna_YawRate_Figure_OfMerit * Aceinna_Rate_struct->signal_info.Aceinna_YawRate_Figure_OfMerit.Factor) + Aceinna_Rate_struct->signal_info.Aceinna_YawRate_Figure_OfMerit.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_YawRate_Figure_OfMerit: " << (int)Aceinna_Rate_struct->signal_value.Aceinna_YawRate_Figure_OfMerit<< std::endl;
        #endif
        Aceinna_Rate_struct->signal_value.Aceinna_AngleRate_Latency = (Complete_Frame >> Aceinna_Rate_struct->signal_info.Aceinna_AngleRate_Latency.StartBit) & 0xff;
        Aceinna_Rate_struct->signal_value.Aceinna_AngleRate_Latency = (Aceinna_Rate_struct->signal_value.Aceinna_AngleRate_Latency * Aceinna_Rate_struct->signal_info.Aceinna_AngleRate_Latency.Factor) + Aceinna_Rate_struct->signal_info.Aceinna_AngleRate_Latency.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_AngleRate_Latency: " << (int)Aceinna_Rate_struct->signal_value.Aceinna_AngleRate_Latency<< std::endl;
        #endif
    }
    else if (msg.id == Aceinna_Accel_struct->ID) {
        Complete_Frame=CAN_Frame_Function(msg);
        Aceinna_Accel_struct->timestamp_us = msg.timestamp_us;
        Aceinna_Accel_struct->signal_value.Aceinna_AccX = (Complete_Frame >> Aceinna_Accel_struct->signal_info.Aceinna_AccX.StartBit) & 0xffff;
        Aceinna_Accel_struct->signal_value.Aceinna_AccX = (Aceinna_Accel_struct->signal_value.Aceinna_AccX * Aceinna_Accel_struct->signal_info.Aceinna_AccX.Factor) + Aceinna_Accel_struct->signal_info.Aceinna_AccX.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_AccX: " << (int)Aceinna_Accel_struct->signal_value.Aceinna_AccX<< std::endl;
        #endif
        Aceinna_Accel_struct->signal_value.Aceinna_AccY = (Complete_Frame >> Aceinna_Accel_struct->signal_info.Aceinna_AccY.StartBit) & 0xffff;
        Aceinna_Accel_struct->signal_value.Aceinna_AccY = (Aceinna_Accel_struct->signal_value.Aceinna_AccY * Aceinna_Accel_struct->signal_info.Aceinna_AccY.Factor) + Aceinna_Accel_struct->signal_info.Aceinna_AccY.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_AccY: " << (int)Aceinna_Accel_struct->signal_value.Aceinna_AccY<< std::endl;
        #endif
        Aceinna_Accel_struct->signal_value.Aceinna_AccZ = (Complete_Frame >> Aceinna_Accel_struct->signal_info.Aceinna_AccZ.StartBit) & 0xffff;
        Aceinna_Accel_struct->signal_value.Aceinna_AccZ = (Aceinna_Accel_struct->signal_value.Aceinna_AccZ * Aceinna_Accel_struct->signal_info.Aceinna_AccZ.Factor) + Aceinna_Accel_struct->signal_info.Aceinna_AccZ.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_AccZ: " << (int)Aceinna_Accel_struct->signal_value.Aceinna_AccZ<< std::endl;
        #endif
        Aceinna_Accel_struct->signal_value.Aceinna_LateralAcc_FigureOfMerit = (Complete_Frame >> Aceinna_Accel_struct->signal_info.Aceinna_LateralAcc_FigureOfMerit.StartBit) & 0x3;
        Aceinna_Accel_struct->signal_value.Aceinna_LateralAcc_FigureOfMerit = (Aceinna_Accel_struct->signal_value.Aceinna_LateralAcc_FigureOfMerit * Aceinna_Accel_struct->signal_info.Aceinna_LateralAcc_FigureOfMerit.Factor) + Aceinna_Accel_struct->signal_info.Aceinna_LateralAcc_FigureOfMerit.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_LateralAcc_FigureOfMerit: " << (int)Aceinna_Accel_struct->signal_value.Aceinna_LateralAcc_FigureOfMerit<< std::endl;
        #endif
        Aceinna_Accel_struct->signal_value.Aceinna_LongiAcc_FigureOfMerit = (Complete_Frame >> Aceinna_Accel_struct->signal_info.Aceinna_LongiAcc_FigureOfMerit.StartBit) & 0x3;
        Aceinna_Accel_struct->signal_value.Aceinna_LongiAcc_FigureOfMerit = (Aceinna_Accel_struct->signal_value.Aceinna_LongiAcc_FigureOfMerit * Aceinna_Accel_struct->signal_info.Aceinna_LongiAcc_FigureOfMerit.Factor) + Aceinna_Accel_struct->signal_info.Aceinna_LongiAcc_FigureOfMerit.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_LongiAcc_FigureOfMerit: " << (int)Aceinna_Accel_struct->signal_value.Aceinna_LongiAcc_FigureOfMerit<< std::endl;
        #endif
        Aceinna_Accel_struct->signal_value.Aceinna_VerticAcc_FigureOfMerit = (Complete_Frame >> Aceinna_Accel_struct->signal_info.Aceinna_VerticAcc_FigureOfMerit.StartBit) & 0x3;
        Aceinna_Accel_struct->signal_value.Aceinna_VerticAcc_FigureOfMerit = (Aceinna_Accel_struct->signal_value.Aceinna_VerticAcc_FigureOfMerit * Aceinna_Accel_struct->signal_info.Aceinna_VerticAcc_FigureOfMerit.Factor) + Aceinna_Accel_struct->signal_info.Aceinna_VerticAcc_FigureOfMerit.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_VerticAcc_FigureOfMerit: " << (int)Aceinna_Accel_struct->signal_value.Aceinna_VerticAcc_FigureOfMerit<< std::endl;
        #endif
        Aceinna_Accel_struct->signal_value.Aceinna_Support_Rate_Acc = (Complete_Frame >> Aceinna_Accel_struct->signal_info.Aceinna_Support_Rate_Acc.StartBit) & 0x3;
        Aceinna_Accel_struct->signal_value.Aceinna_Support_Rate_Acc = (Aceinna_Accel_struct->signal_value.Aceinna_Support_Rate_Acc * Aceinna_Accel_struct->signal_info.Aceinna_Support_Rate_Acc.Factor) + Aceinna_Accel_struct->signal_info.Aceinna_Support_Rate_Acc.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_Support_Rate_Acc: " << (int)Aceinna_Accel_struct->signal_value.Aceinna_Support_Rate_Acc<< std::endl;
        #endif
    }
    else if (msg.id == Aceinna_Angles_struct->ID) {
        Complete_Frame=CAN_Frame_Function(msg);
        Aceinna_Angles_struct->timestamp_us = msg.timestamp_us;
        Aceinna_Angles_struct->signal_value.Aceinna_Pitch = (Complete_Frame >> Aceinna_Angles_struct->signal_info.Aceinna_Pitch.StartBit) & 0xffffff;
        Aceinna_Angles_struct->signal_value.Aceinna_Pitch = (Aceinna_Angles_struct->signal_value.Aceinna_Pitch * Aceinna_Angles_struct->signal_info.Aceinna_Pitch.Factor) + Aceinna_Angles_struct->signal_info.Aceinna_Pitch.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_Pitch: " << (int)Aceinna_Angles_struct->signal_value.Aceinna_Pitch<< std::endl;
        #endif
        Aceinna_Angles_struct->signal_value.Aceinna_Roll = (Complete_Frame >> Aceinna_Angles_struct->signal_info.Aceinna_Roll.StartBit) & 0xffffff;
        Aceinna_Angles_struct->signal_value.Aceinna_Roll = (Aceinna_Angles_struct->signal_value.Aceinna_Roll * Aceinna_Angles_struct->signal_info.Aceinna_Roll.Factor) + Aceinna_Angles_struct->signal_info.Aceinna_Roll.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_Roll: " << (int)Aceinna_Angles_struct->signal_value.Aceinna_Roll<< std::endl;
        #endif
        Aceinna_Angles_struct->signal_value.Aceinna_Pitch_Compensation = (Complete_Frame >> Aceinna_Angles_struct->signal_info.Aceinna_Pitch_Compensation.StartBit) & 0x3;
        Aceinna_Angles_struct->signal_value.Aceinna_Pitch_Compensation = (Aceinna_Angles_struct->signal_value.Aceinna_Pitch_Compensation * Aceinna_Angles_struct->signal_info.Aceinna_Pitch_Compensation.Factor) + Aceinna_Angles_struct->signal_info.Aceinna_Pitch_Compensation.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_Pitch_Compensation: " << (int)Aceinna_Angles_struct->signal_value.Aceinna_Pitch_Compensation<< std::endl;
        #endif
        Aceinna_Angles_struct->signal_value.Aceinna_Pitch_Figure_OfMerit = (Complete_Frame >> Aceinna_Angles_struct->signal_info.Aceinna_Pitch_Figure_OfMerit.StartBit) & 0x3;
        Aceinna_Angles_struct->signal_value.Aceinna_Pitch_Figure_OfMerit = (Aceinna_Angles_struct->signal_value.Aceinna_Pitch_Figure_OfMerit * Aceinna_Angles_struct->signal_info.Aceinna_Pitch_Figure_OfMerit.Factor) + Aceinna_Angles_struct->signal_info.Aceinna_Pitch_Figure_OfMerit.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_Pitch_Figure_OfMerit: " << (int)Aceinna_Angles_struct->signal_value.Aceinna_Pitch_Figure_OfMerit<< std::endl;
        #endif
        Aceinna_Angles_struct->signal_value.Aceinna_Roll_Compensation = (Complete_Frame >> Aceinna_Angles_struct->signal_info.Aceinna_Roll_Compensation.StartBit) & 0x3;
        Aceinna_Angles_struct->signal_value.Aceinna_Roll_Compensation = (Aceinna_Angles_struct->signal_value.Aceinna_Roll_Compensation * Aceinna_Angles_struct->signal_info.Aceinna_Roll_Compensation.Factor) + Aceinna_Angles_struct->signal_info.Aceinna_Roll_Compensation.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_Roll_Compensation: " << (int)Aceinna_Angles_struct->signal_value.Aceinna_Roll_Compensation<< std::endl;
        #endif
        Aceinna_Angles_struct->signal_value.Aceinna_Roll_Figure_OfMerit = (Complete_Frame >> Aceinna_Angles_struct->signal_info.Aceinna_Roll_Figure_OfMerit.StartBit) & 0x3;
        Aceinna_Angles_struct->signal_value.Aceinna_Roll_Figure_OfMerit = (Aceinna_Angles_struct->signal_value.Aceinna_Roll_Figure_OfMerit * Aceinna_Angles_struct->signal_info.Aceinna_Roll_Figure_OfMerit.Factor) + Aceinna_Angles_struct->signal_info.Aceinna_Roll_Figure_OfMerit.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_Roll_Figure_OfMerit: " << (int)Aceinna_Angles_struct->signal_value.Aceinna_Roll_Figure_OfMerit<< std::endl;
        #endif
        Aceinna_Angles_struct->signal_value.Aceinna_PitchRoll_Latency = (Complete_Frame >> Aceinna_Angles_struct->signal_info.Aceinna_PitchRoll_Latency.StartBit) & 0xff;
        Aceinna_Angles_struct->signal_value.Aceinna_PitchRoll_Latency = (Aceinna_Angles_struct->signal_value.Aceinna_PitchRoll_Latency * Aceinna_Angles_struct->signal_info.Aceinna_PitchRoll_Latency.Factor) + Aceinna_Angles_struct->signal_info.Aceinna_PitchRoll_Latency.Offset;
        #ifdef Debug_long
        std::cout << std::hex << "        Aceinna_PitchRoll_Latency: " << (int)Aceinna_Angles_struct->signal_value.Aceinna_PitchRoll_Latency<< std::endl;
        #endif
    }
    else { 
        std::cout << "CAN ID NOT FOUND, ID: " << msg.id << std::endl; 
    } 
}
 
CANdb::CANdb() { 
    Aceinna_Address_Claiming_struct = std::make_unique<Aceinna_Address_Claiming>(Aceinna_Address_Claiming {
        /*ID*/ 418316160,
        /*DLC*/ 8,
        /*timestamp_us*/ rclcpp::Time(0, 0, rcl_clock_type_t::RCL_CLOCK_UNINITIALIZED),
        /*number_of_signals*/ 10,
        /*node*/ "MainECU",
        /*signal_info*/ {
            /*J1939_Name_ID_Code*/ {
                /*StartBit*/ 0,
                /*Lenght*/ 21,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ -16,
                /*Maximum*/ 15,
            },
            /*J1939_Name_Manufacture_Code*/ {
                /*StartBit*/ 21,
                /*Lenght*/ 11,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
            /*J1939_Name_ECU_Example*/ {
                /*StartBit*/ 32,
                /*Lenght*/ 3,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ -4,
                /*Maximum*/ 3,
            },
            /*J1939_Name_Function_Example*/ {
                /*StartBit*/ 35,
                /*Lenght*/ 5,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
            /*J1939_Name_Function*/ {
                /*StartBit*/ 40,
                /*Lenght*/ 8,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
            /*J1939_Name_Reserved0*/ {
                /*StartBit*/ 48,
                /*Lenght*/ 1,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
            /*J1939_Name_Vehicle_System*/ {
                /*StartBit*/ 49,
                /*Lenght*/ 7,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
            /*J1939_Name_Vehicle_Example*/ {
                /*StartBit*/ 56,
                /*Lenght*/ 4,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
            /*J1939_Name_Industry_Group*/ {
                /*StartBit*/ 60,
                /*Lenght*/ 3,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
            /*J1939_Name_Arbitrary_Address*/ {
                /*StartBit*/ 63,
                /*Lenght*/ 1,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
		},
		/*signal_value*/ {
			/*J1939_Name_ID_Code*/ 0,
			/*J1939_Name_Manufacture_Code*/ 0,
			/*J1939_Name_ECU_Example*/ 0,
			/*J1939_Name_Function_Example*/ 0,
			/*J1939_Name_Function*/ 0,
			/*J1939_Name_Reserved0*/ 0,
			/*J1939_Name_Vehicle_System*/ 0,
			/*J1939_Name_Vehicle_Example*/ 0,
			/*J1939_Name_Industry_Group*/ 0,
			/*J1939_Name_Arbitrary_Address*/ 0,
		},
	});

    Aceinna_Rapid_Attitude_struct = std::make_unique<Aceinna_Rapid_Attitude>(Aceinna_Rapid_Attitude {
        /*ID*/ 435231104,
        /*DLC*/ 8,
        /*timestamp_us*/ rclcpp::Time(0, 0, rcl_clock_type_t::RCL_CLOCK_UNINITIALIZED),
        /*number_of_signals*/ 5,
        /*node*/ "Aceinna",
        /*signal_info*/ {
            /*Aceinna_SID_A*/ {
                /*StartBit*/ 0,
                /*Lenght*/ 8,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
            /*Aceinna_ATT_Yaw*/ {
                /*StartBit*/ 8,
                /*Lenght*/ 16,
                /*Factor*/ 0.0001,
                /*Offset*/ -3.141592653589,
                /*Minimum*/ -3.141592653589,
                /*Maximum*/ 3.141592653589,
            },
            /*Aceinna_ATT_Pitch*/ {
                /*StartBit*/ 24,
                /*Lenght*/ 16,
                /*Factor*/ 0.0001,
                /*Offset*/ -3.141592653589,
                /*Minimum*/ -3.141592653589,
                /*Maximum*/ 3.141592653589,
            },
            /*Aceinna_ATT_Roll*/ {
                /*StartBit*/ 40,
                /*Lenght*/ 16,
                /*Factor*/ 0.0001,
                /*Offset*/ -3.141592653589,
                /*Minimum*/ -3.141592653589,
                /*Maximum*/ 3.141592653589,
            },
            /*Aceinna_ATT_Rsvd*/ {
                /*StartBit*/ 56,
                /*Lenght*/ 8,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
		},
		/*signal_value*/ {
			/*Aceinna_SID_A*/ 0,
			/*Aceinna_ATT_Yaw*/ 0,
			/*Aceinna_ATT_Pitch*/ 0,
			/*Aceinna_ATT_Roll*/ 0,
			/*Aceinna_ATT_Rsvd*/ 0,
		},
	});

    Aceinna_Rate_struct = std::make_unique<Aceinna_Rate>(Aceinna_Rate {
        /*ID*/ 217066112,
        /*DLC*/ 8,
        /*timestamp_us*/ rclcpp::Time(0, 0, rcl_clock_type_t::RCL_CLOCK_UNINITIALIZED),
        /*number_of_signals*/ 7,
        /*node*/ "Aceinna",
        /*signal_info*/ {
            /*Aceinna_GyroX*/ {
                /*StartBit*/ 0,
                /*Lenght*/ 16,
                /*Factor*/ 0.0078125,
                /*Offset*/ -250,
                /*Minimum*/ -250,
                /*Maximum*/ 261.9921875,
            },
            /*Aceinna_GyroY*/ {
                /*StartBit*/ 16,
                /*Lenght*/ 16,
                /*Factor*/ 0.0078125,
                /*Offset*/ -250,
                /*Minimum*/ -250,
                /*Maximum*/ 261.9921875,
            },
            /*Aceinna_GyroZ*/ {
                /*StartBit*/ 32,
                /*Lenght*/ 16,
                /*Factor*/ 0.0078125,
                /*Offset*/ -250,
                /*Minimum*/ -250,
                /*Maximum*/ 261.9921875,
            },
            /*Aceinna_PitchRate_Figure_OfMerit*/ {
                /*StartBit*/ 48,
                /*Lenght*/ 2,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
            /*Aceinna_RollRate_Figure_OfMerit*/ {
                /*StartBit*/ 50,
                /*Lenght*/ 2,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
            /*Aceinna_YawRate_Figure_OfMerit*/ {
                /*StartBit*/ 52,
                /*Lenght*/ 2,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
            /*Aceinna_AngleRate_Latency*/ {
                /*StartBit*/ 56,
                /*Lenght*/ 8,
                /*Factor*/ 0.5,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
		},
		/*signal_value*/ {
			/*Aceinna_GyroX*/ 0,
			/*Aceinna_GyroY*/ 0,
			/*Aceinna_GyroZ*/ 0,
			/*Aceinna_PitchRate_Figure_OfMerit*/ 0,
			/*Aceinna_RollRate_Figure_OfMerit*/ 0,
			/*Aceinna_YawRate_Figure_OfMerit*/ 0,
			/*Aceinna_AngleRate_Latency*/ 0,
		},
	});

    Aceinna_Accel_struct = std::make_unique<Aceinna_Accel>(Aceinna_Accel {
        /*ID*/ 149958016,
        /*DLC*/ 8,
        /*timestamp_us*/ rclcpp::Time(0, 0, rcl_clock_type_t::RCL_CLOCK_UNINITIALIZED),
        /*number_of_signals*/ 7,
        /*node*/ "Aceinna",
        /*signal_info*/ {
            /*Aceinna_AccX*/ {
                /*StartBit*/ 0,
                /*Lenght*/ 16,
                /*Factor*/ 0.01,
                /*Offset*/ -320,
                /*Minimum*/ -320,
                /*Maximum*/ 335.35,
            },
            /*Aceinna_AccY*/ {
                /*StartBit*/ 16,
                /*Lenght*/ 16,
                /*Factor*/ 0.01,
                /*Offset*/ -320,
                /*Minimum*/ -320,
                /*Maximum*/ 335.35,
            },
            /*Aceinna_AccZ*/ {
                /*StartBit*/ 32,
                /*Lenght*/ 16,
                /*Factor*/ 0.01,
                /*Offset*/ -320,
                /*Minimum*/ -320,
                /*Maximum*/ 335.35,
            },
            /*Aceinna_LateralAcc_FigureOfMerit*/ {
                /*StartBit*/ 48,
                /*Lenght*/ 2,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
            /*Aceinna_LongiAcc_FigureOfMerit*/ {
                /*StartBit*/ 50,
                /*Lenght*/ 2,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
            /*Aceinna_VerticAcc_FigureOfMerit*/ {
                /*StartBit*/ 52,
                /*Lenght*/ 2,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
            /*Aceinna_Support_Rate_Acc*/ {
                /*StartBit*/ 54,
                /*Lenght*/ 2,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
		},
		/*signal_value*/ {
			/*Aceinna_AccX*/ 0,
			/*Aceinna_AccY*/ 0,
			/*Aceinna_AccZ*/ 0,
			/*Aceinna_LateralAcc_FigureOfMerit*/ 0,
			/*Aceinna_LongiAcc_FigureOfMerit*/ 0,
			/*Aceinna_VerticAcc_FigureOfMerit*/ 0,
			/*Aceinna_Support_Rate_Acc*/ 0,
		},
	});

    Aceinna_Angles_struct = std::make_unique<Aceinna_Angles>(Aceinna_Angles {
        /*ID*/ 217065856,
        /*DLC*/ 8,
        /*timestamp_us*/ rclcpp::Time(0, 0, rcl_clock_type_t::RCL_CLOCK_UNINITIALIZED),
        /*number_of_signals*/ 7,
        /*node*/ "Aceinna",
        /*signal_info*/ {
            /*Aceinna_Pitch*/ {
                /*StartBit*/ 0,
                /*Lenght*/ 24,
                /*Factor*/ 3.0517578125e-05,
                /*Offset*/ -250,
                /*Minimum*/ -250,
                /*Maximum*/ 252,
            },
            /*Aceinna_Roll*/ {
                /*StartBit*/ 24,
                /*Lenght*/ 24,
                /*Factor*/ 3.0517578125e-05,
                /*Offset*/ -250,
                /*Minimum*/ -250,
                /*Maximum*/ 252,
            },
            /*Aceinna_Pitch_Compensation*/ {
                /*StartBit*/ 48,
                /*Lenght*/ 2,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
            /*Aceinna_Pitch_Figure_OfMerit*/ {
                /*StartBit*/ 50,
                /*Lenght*/ 2,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
            /*Aceinna_Roll_Compensation*/ {
                /*StartBit*/ 52,
                /*Lenght*/ 2,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
            /*Aceinna_Roll_Figure_OfMerit*/ {
                /*StartBit*/ 54,
                /*Lenght*/ 2,
                /*Factor*/ 1,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
            /*Aceinna_PitchRoll_Latency*/ {
                /*StartBit*/ 56,
                /*Lenght*/ 8,
                /*Factor*/ 0.5,
                /*Offset*/ 0,
                /*Minimum*/ 0,
                /*Maximum*/ 0,
            },
		},
		/*signal_value*/ {
			/*Aceinna_Pitch*/ 0,
			/*Aceinna_Roll*/ 0,
			/*Aceinna_Pitch_Compensation*/ 0,
			/*Aceinna_Pitch_Figure_OfMerit*/ 0,
			/*Aceinna_Roll_Compensation*/ 0,
			/*Aceinna_Roll_Figure_OfMerit*/ 0,
			/*Aceinna_PitchRoll_Latency*/ 0,
		},
	});

}