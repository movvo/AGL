#ifndef DBC_CANDB_H
#define DBC_CANDB_H

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <chrono>

namespace can_utils {

class CANdb {
public:
	CANdb();
	struct CAN_msg {
		uint32_t id; // CAN ID of the message sender.
		uint8_t size; // Number of bytes of the payload.
		rclcpp::Time timestamp_us; // Timestamp of the message in microseconds (using clock of the context).
		std::vector<uint8_t> data; // Payload
	};

	uint64_t CAN_Frame_Function(CAN_msg msg);
	CAN_msg CANTx_FrameMount(uint32_t ID2Send);
	void CANRx_Read(CAN_msg msg);

	std::map<std::string, std::vector<std::uint64_t>> ids = {
		{"MainECU", {418316160} },
		{"Aceinna", {435231104, 217066112, 149958016, 217065856} }
	};

	struct Aceinna_Address_Claiming {
		uint32_t ID;
		uint8_t DLC;
		rclcpp::Time timestamp_us;
		uint8_t number_of_signals;
		std::string node; 
		struct signal_info {
			struct J1939_Name_ID_Code {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				int Minimum;
				uint8_t Maximum;
			} J1939_Name_ID_Code;
			struct J1939_Name_Manufacture_Code {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} J1939_Name_Manufacture_Code;
			struct J1939_Name_ECU_Example {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				int Minimum;
				uint8_t Maximum;
			} J1939_Name_ECU_Example;
			struct J1939_Name_Function_Example {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} J1939_Name_Function_Example;
			struct J1939_Name_Function {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} J1939_Name_Function;
			struct J1939_Name_Reserved0 {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} J1939_Name_Reserved0;
			struct J1939_Name_Vehicle_System {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} J1939_Name_Vehicle_System;
			struct J1939_Name_Vehicle_Example {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} J1939_Name_Vehicle_Example;
			struct J1939_Name_Industry_Group {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} J1939_Name_Industry_Group;
			struct J1939_Name_Arbitrary_Address {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} J1939_Name_Arbitrary_Address;
		} signal_info;
		struct __attribute__((packed)) signal_value{
			int32_t J1939_Name_ID_Code;
			uint16_t J1939_Name_Manufacture_Code;
			int8_t J1939_Name_ECU_Example;
			uint8_t J1939_Name_Function_Example;
			uint8_t J1939_Name_Function;
			uint8_t J1939_Name_Reserved0;
			uint8_t J1939_Name_Vehicle_System;
			uint8_t J1939_Name_Vehicle_Example;
			uint8_t J1939_Name_Industry_Group;
			uint8_t J1939_Name_Arbitrary_Address;
		} signal_value;
	};

	std::unique_ptr<struct Aceinna_Address_Claiming> Aceinna_Address_Claiming_struct; 

	struct Aceinna_Rapid_Attitude {
		uint32_t ID;
		uint8_t DLC;
		rclcpp::Time timestamp_us;
		uint8_t number_of_signals;
		std::string node; 
		struct signal_info {
			struct Aceinna_SID_A {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} Aceinna_SID_A;
			struct Aceinna_ATT_Yaw {
				uint8_t StartBit;
				uint8_t Lenght;
				float Factor;
				float Offset;
				float Minimum;
				float Maximum;
			} Aceinna_ATT_Yaw;
			struct Aceinna_ATT_Pitch {
				uint8_t StartBit;
				uint8_t Lenght;
				float Factor;
				float Offset;
				float Minimum;
				float Maximum;
			} Aceinna_ATT_Pitch;
			struct Aceinna_ATT_Roll {
				uint8_t StartBit;
				uint8_t Lenght;
				float Factor;
				float Offset;
				float Minimum;
				float Maximum;
			} Aceinna_ATT_Roll;
			struct Aceinna_ATT_Rsvd {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} Aceinna_ATT_Rsvd;
		} signal_info;
		struct __attribute__((packed)) signal_value{
			uint8_t Aceinna_SID_A;
			double Aceinna_ATT_Yaw;
			double Aceinna_ATT_Pitch;
			double Aceinna_ATT_Roll;
			uint8_t Aceinna_ATT_Rsvd;
		} signal_value;
	};

	std::unique_ptr<struct Aceinna_Rapid_Attitude> Aceinna_Rapid_Attitude_struct; 

	struct Aceinna_Rate {
		uint32_t ID;
		uint8_t DLC;
		rclcpp::Time timestamp_us;
		uint8_t number_of_signals;
		std::string node; 
		struct signal_info {
			struct Aceinna_GyroX {
				uint8_t StartBit;
				uint8_t Lenght;
				float Factor;
				int Offset;
				int Minimum;
				float Maximum;
			} Aceinna_GyroX;
			struct Aceinna_GyroY {
				uint8_t StartBit;
				uint8_t Lenght;
				float Factor;
				int Offset;
				int Minimum;
				float Maximum;
			} Aceinna_GyroY;
			struct Aceinna_GyroZ {
				uint8_t StartBit;
				uint8_t Lenght;
				float Factor;
				int Offset;
				int Minimum;
				float Maximum;
			} Aceinna_GyroZ;
			struct Aceinna_PitchRate_Figure_OfMerit {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} Aceinna_PitchRate_Figure_OfMerit;
			struct Aceinna_RollRate_Figure_OfMerit {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} Aceinna_RollRate_Figure_OfMerit;
			struct Aceinna_YawRate_Figure_OfMerit {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} Aceinna_YawRate_Figure_OfMerit;
			struct Aceinna_AngleRate_Latency {
				uint8_t StartBit;
				uint8_t Lenght;
				float Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} Aceinna_AngleRate_Latency;
		} signal_info;
		struct __attribute__((packed)) signal_value{
			double Aceinna_GyroX;
			double Aceinna_GyroY;
			double Aceinna_GyroZ;
			uint8_t Aceinna_PitchRate_Figure_OfMerit;
			uint8_t Aceinna_RollRate_Figure_OfMerit;
			uint8_t Aceinna_YawRate_Figure_OfMerit;
			uint8_t Aceinna_AngleRate_Latency;
		} signal_value;
	};

	std::unique_ptr<struct Aceinna_Rate> Aceinna_Rate_struct; 

	struct Aceinna_Accel {
		uint32_t ID;
		uint8_t DLC;
		rclcpp::Time timestamp_us;
		uint8_t number_of_signals;
		std::string node; 
		struct signal_info {
			struct Aceinna_AccX {
				uint8_t StartBit;
				uint8_t Lenght;
				float Factor;
				int Offset;
				int Minimum;
				float Maximum;
			} Aceinna_AccX;
			struct Aceinna_AccY {
				uint8_t StartBit;
				uint8_t Lenght;
				float Factor;
				int Offset;
				int Minimum;
				float Maximum;
			} Aceinna_AccY;
			struct Aceinna_AccZ {
				uint8_t StartBit;
				uint8_t Lenght;
				float Factor;
				int Offset;
				int Minimum;
				float Maximum;
			} Aceinna_AccZ;
			struct Aceinna_LateralAcc_FigureOfMerit {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} Aceinna_LateralAcc_FigureOfMerit;
			struct Aceinna_LongiAcc_FigureOfMerit {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} Aceinna_LongiAcc_FigureOfMerit;
			struct Aceinna_VerticAcc_FigureOfMerit {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} Aceinna_VerticAcc_FigureOfMerit;
			struct Aceinna_Support_Rate_Acc {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} Aceinna_Support_Rate_Acc;
		} signal_info;
		struct __attribute__((packed)) signal_value{
			double Aceinna_AccX;
			double Aceinna_AccY;
			double Aceinna_AccZ;
			uint8_t Aceinna_LateralAcc_FigureOfMerit;
			uint8_t Aceinna_LongiAcc_FigureOfMerit;
			uint8_t Aceinna_VerticAcc_FigureOfMerit;
			uint8_t Aceinna_Support_Rate_Acc;
		} signal_value;
	};

	std::unique_ptr<struct Aceinna_Accel> Aceinna_Accel_struct; 

	struct Aceinna_Angles {
		uint32_t ID;
		uint8_t DLC;
		rclcpp::Time timestamp_us;
		uint8_t number_of_signals;
		std::string node; 
		struct signal_info {
			struct Aceinna_Pitch {
				uint8_t StartBit;
				uint8_t Lenght;
				float Factor;
				int Offset;
				int Minimum;
				uint8_t Maximum;
			} Aceinna_Pitch;
			struct Aceinna_Roll {
				uint8_t StartBit;
				uint8_t Lenght;
				float Factor;
				int Offset;
				int Minimum;
				uint8_t Maximum;
			} Aceinna_Roll;
			struct Aceinna_Pitch_Compensation {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} Aceinna_Pitch_Compensation;
			struct Aceinna_Pitch_Figure_OfMerit {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} Aceinna_Pitch_Figure_OfMerit;
			struct Aceinna_Roll_Compensation {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} Aceinna_Roll_Compensation;
			struct Aceinna_Roll_Figure_OfMerit {
				uint8_t StartBit;
				uint8_t Lenght;
				uint8_t Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} Aceinna_Roll_Figure_OfMerit;
			struct Aceinna_PitchRoll_Latency {
				uint8_t StartBit;
				uint8_t Lenght;
				float Factor;
				uint8_t Offset;
				uint8_t Minimum;
				uint8_t Maximum;
			} Aceinna_PitchRoll_Latency;
		} signal_info;
		struct __attribute__((packed)) signal_value{
			int32_t Aceinna_Pitch;
			int32_t Aceinna_Roll;
			uint8_t Aceinna_Pitch_Compensation;
			uint8_t Aceinna_Pitch_Figure_OfMerit;
			uint8_t Aceinna_Roll_Compensation;
			uint8_t Aceinna_Roll_Figure_OfMerit;
			uint8_t Aceinna_PitchRoll_Latency;
		} signal_value;
	};

	std::unique_ptr<struct Aceinna_Angles> Aceinna_Angles_struct; 


private: 
    uint64_t Complete_Frame=0;

}; 
} // namespace can_utils 
#endif /* DBC_CANDB_H */