#include "motor_ros2/motor_cfg.h"
#include <sys/select.h> // For select() timeout

void RobStrideMotor::init_serial()
{
    // Open Serial Port
    serial_fd = open(iface.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0)
    {
        perror(("Error opening serial port: " + iface).c_str());
        exit(1);
    }

    struct termios tty;
    if (tcgetattr(serial_fd, &tty) != 0)
    {
        perror("Error from tcgetattr");
        exit(1);
    }

    // Set Baud Rate to 921600
    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);

    // 8N1 Configuration (8 Data bits, No Parity, 1 Stop bit)
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_cflag &= ~PARENB;                     // No parity
    tty.c_cflag &= ~CSTOPB;                     // 1 stop bit
    tty.c_cflag |= (CLOCAL | CREAD);            // Ignore modem controls, enable reading

    // Raw mode (Disable software flow control, echo, signals, etc.)
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    // Set Read Timers (Non-blocking read)
    tty.c_cc[VMIN] = 0;  
    tty.c_cc[VTIME] = 1; // 0.1 seconds read timeout

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0)
    {
        perror("Error from tcsetattr");
        exit(1);
    }
    
    // Clear buffer
    tcflush(serial_fd, TCIOFLUSH);
    std::cout << "[INFO] Serial port initialized: " << iface << " @ 921600 bps" << std::endl;
}

// Helper: Convert internal CAN frame to NiRen Serial Packet and send
void RobStrideMotor::send_can_frame_niren(const struct can_frame& frame)
{
    uint8_t buffer[17];
    uint32_t id = frame.can_id & CAN_EFF_MASK; // Extract 29-bit ID

    // Header
    buffer[0] = 0xAA;
    buffer[1] = 0x01; // Extended Frame Flag (0x01 = Extended)
    buffer[2] = 0x00; // Remote Flag (0x00 = Data)
    buffer[3] = 0x08; // Length

    // CAN ID (Big Endian placement in packet)
    buffer[4] = (id >> 24) & 0xFF;
    buffer[5] = (id >> 16) & 0xFF;
    buffer[6] = (id >> 8) & 0xFF;
    buffer[7] = (id >> 0) & 0xFF;

    // Data Payload
    for(int i = 0; i < 8; i++) {
        buffer[8 + i] = frame.data[i];
    }

    // Tail
    buffer[16] = 0x7A;

    int n = write(serial_fd, buffer, 17);
    if (n != 17)
    {
        perror("Error writing to serial port");
    }
}

ReceiveResult RobStrideMotor::receive(double timeout_sec)
{
    // Simple read state machine to find AA ... 7A packet
    uint8_t byte;
    std::vector<uint8_t> packet;
    
    // Timeout setup using select()
    fd_set set;
    struct timeval timeout;
    struct timeval* timeout_ptr = nullptr;

    if (timeout_sec > 0) {
        timeout.tv_sec = (int)timeout_sec;
        timeout.tv_usec = (int)((timeout_sec - timeout.tv_sec) * 1e6);
        timeout_ptr = &timeout;
    }

    // Loop until we get a full packet or timeout
    while(true) {
        FD_ZERO(&set);
        FD_SET(serial_fd, &set);

        int rv = select(serial_fd + 1, &set, NULL, NULL, timeout_ptr);
        if(rv == -1) {
            perror("select error"); // Error
            return std::nullopt;
        } else if(rv == 0) {
            return std::nullopt; // Timeout
        }

        int n = read(serial_fd, &byte, 1);
        if (n > 0) {
            // State 0: Waiting for Header 0xAA
            if (packet.empty()) {
                if (byte == 0xAA) {
                    packet.push_back(byte);
                }
            } 
            // State 1: Reading Body
            else {
                packet.push_back(byte);
                // Check if full packet received
                if (packet.size() == 17) {
                    // Check Tail
                    if (packet[16] == 0x7A) {
                        // VALID PACKET FOUND -> Parse it
                        
                        // Reconstruct CAN ID
                        uint32_t can_id = 0;
                        can_id |= packet[4] << 24;
                        can_id |= packet[5] << 16;
                        can_id |= packet[6] << 8;
                        can_id |= packet[7];

                        uint8_t communication_type = (can_id >> 24) & 0x1F;
                        uint16_t extra_data = (can_id >> 8) & 0xFFFF;
                        uint8_t host_id = can_id & 0xFF;

                        error_code = uint8_t((can_id >> 16) & 0x3F);
                        pattern = uint8_t((can_id >> 22) & 0x03);

                        // Extract Data
                        std::vector<uint8_t> data_vec;
                        for(int i=0; i<8; i++) {
                            data_vec.push_back(packet[8+i]);
                        }

                        // Debug prints
                        // std::cout << "RX ID: " << std::hex << can_id << std::dec << std::endl;

                        return std::make_tuple(communication_type, extra_data, host_id, data_vec);
                    } else {
                        // Invalid tail, reset but keep scanning (maybe the byte 0xAA was data)
                        // Simple reset for now
                        packet.clear(); 
                    }
                }
            }
        }
    }
}

void RobStrideMotor::receive_status_frame()
{
    auto result = receive(0.1); // Small timeout to prevent blocking forever
    if (!result)
    {
        // Suppress error if just polling, or log warning
        // std::cerr << "No status frame received." << std::endl;
        return;
    }

    auto [communication_type, extra_data, host_id, data] = *result;

    // uint8_t status_mode = (extra_data >> 14) & 0x03;
    // uint8_t status_uncalibrated = (extra_data >> 13) & 0x01;
    // uint8_t status_hall_encoder_fault = (extra_data >> 12) & 0x01;
    // uint8_t status_magnetic_encoder_fault = (extra_data >> 11) & 0x01;
    // uint8_t status_overtemperature = (extra_data >> 10) & 0x01;
    // uint8_t status_overcurrent = (extra_data >> 9) & 0x01;
    // uint8_t status_undervoltage = (extra_data >> 8) & 0x01;
    // uint8_t device_id = (extra_data >> 0) & 0xFF;
    
    if (data.size() < 8)
    {
        throw std::runtime_error("Data size too small");
    }

    // std::cout << "communication_type: " << static_cast<int>(communication_type) << std::endl;
    if (communication_type == Communication_Type_MotorRequest) // Type 2 Feedback
    {
        // Parse Big Endian Data
        uint16_t position_u16 = (data[0] << 8) | data[1];
        uint16_t velocity_u16 = (data[2] << 8) | data[3];
        uint16_t torque_i16 = (data[4] << 8) | data[5];
        uint16_t temperature_u16 = (data[6] << 8) | data[7];

        // Convert to physical units
        position_ = ((static_cast<float>(position_u16) / 32767.0f) - 1.0f) * (ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).position);
        velocity_ = ((static_cast<float>(velocity_u16) / 32767.0f) - 1.0f) * (ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).velocity);
        torque_ = ((static_cast<float>(torque_i16) / 32767.0f) - 1.0f) * (ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).torque);
        temperature_ = static_cast<float>(temperature_u16) * 0.1f;
    }
    else if(communication_type == 17) // Parameter Read Response
    {
        for (int index_num = 0; index_num <= 14; index_num++)
        {
            if ((data[1]<<8|data[0]) == Index_List[index_num])
                switch(index_num)
                {
                    case 0:
                        drw.run_mode.data = uint8_t(data[4]);
                        std::cout << "mode data: " << static_cast<int>(data[4]) << std::endl;
                        break;
                    case 1:
                        drw.iq_ref.data = Byte_to_float(data);
                        break;
                    case 2:
                        drw.spd_ref.data = Byte_to_float(data);
                        break;
                    case 3:
                        drw.imit_torque.data = Byte_to_float(data);
                        break;
                    case 4:
                        drw.cur_kp.data = Byte_to_float(data);
                        break;
                    case 5:
                        drw.cur_ki.data = Byte_to_float(data);
                        break;
                    case 6:
                        drw.cur_filt_gain.data = Byte_to_float(data);
                        break;
                    case 7:
                        drw.loc_ref.data = Byte_to_float(data);
                        break;
                    case 8:
                        drw.limit_spd.data = Byte_to_float(data);
                        break;
                    case 9:
                        drw.limit_cur.data = Byte_to_float(data);
                        break;	
                    case 10:
                        drw.mechPos.data = Byte_to_float(data);
                        break;	
                    case 11:
                        drw.iqf.data = Byte_to_float(data);
                        break;	
                    case 12:
                        drw.mechVel.data =Byte_to_float(data);
                        break;	
                    case 13:
                        drw.VBUS.data = Byte_to_float(data);
                        break;
                    case 14:
                        drw.rotation.data = (int16_t)((data[5] << 8) | data[4]);
                        break;	
                }
		}
    }
}

void RobStrideMotor::Set_RobStrite_Motor_parameter(uint16_t Index, float Value, char Value_mode)
{
    struct can_frame frame{};

    // Construct ID
    frame.can_id = Communication_Type_SetSingleParameter << 24 | master_id << 8 | motor_id;
    frame.can_id |= CAN_EFF_FLAG; 
    frame.can_dlc = 0x08;

    frame.data[0] = Index;
    frame.data[1] = Index >> 8;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;

    if (Value_mode == 'p')
    {
        memcpy(&frame.data[4], &Value, 4);
    }
    else if (Value_mode == 'j')
    {
        frame.data[4] = (uint8_t)Value;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
    }

    send_can_frame_niren(frame);
    receive_status_frame();
}

std::tuple<float, float, float, float> RobStrideMotor::enable_motor()
{
    struct can_frame frame{};
    frame.can_id = (Communication_Type_MotorEnable << 24) | (master_id << 8) | motor_id;
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 8;
    memset(frame.data, 0, 8);

    send_can_frame_niren(frame);
    std::cout << "[✓] Motor enable command sent." << std::endl;
    receive_status_frame();
    
    return std::make_tuple(position_, velocity_, torque_, temperature_);
}

// 发送运控模式（控制角度 + 速度 + KP + KD）
std::tuple<float, float, float, float> RobStrideMotor::send_motion_command(float torque,
                                                                           float position_rad,
                                                                           float velocity_rad_s,
                                                                           float kp,
                                                                           float kd)
{
    if(drw.run_mode.data != 0 && pattern == 2)
    {
        Disenable_Motor(0);        
        usleep(1000);
        Set_RobStrite_Motor_parameter(0X7005, move_control_mode, Set_mode);
        usleep(1000);
        Get_RobStrite_Motor_parameter(0x7005);
        usleep(1000);
        enable_motor();
        usleep(1000);
        Motor_Set_All.set_motor_mode = move_control_mode;
    }

    struct can_frame frame{};
    // Construct 29-bit ID
    uint32_t id_val = (Communication_Type_MotionControl << 24) 
                    | (float_to_uint(torque, -ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).torque, ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).torque, 16) << 8) 
                    | motor_id;
    frame.can_id = id_val | CAN_EFF_FLAG;
    frame.can_dlc = 8;

    uint16_t pos = float_to_uint(position_rad, -ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).position, ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).position, 16);
    uint16_t vel = float_to_uint(velocity_rad_s, -ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).velocity, ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).velocity, 16);
    uint16_t kp_u = float_to_uint(kp, 0.0f, ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).kp, 16);
    uint16_t kd_u = float_to_uint(kd, 0.0f, ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).kd, 16);

    frame.data[0] = (pos >> 8);
    frame.data[1] = pos;
    frame.data[2] = (vel >> 8);
    frame.data[3] = vel;
    frame.data[4] = (kp_u >> 8);
    frame.data[5] = kp_u;
    frame.data[6] = (kd_u >> 8);
    frame.data[7] = kd_u;

    send_can_frame_niren(frame);
    receive_status_frame();
    return std::make_tuple(position_, velocity_, torque_, temperature_);
}

std::tuple<float, float, float, float> RobStrideMotor::send_velocity_mode_command(float velocity_rad_s, float curr_limit, float acc)
{
    Motor_Set_All.set_limit_cur = curr_limit;
    Motor_Set_All.set_acc = acc;
    Motor_Set_All.set_speed = velocity_rad_s;

    if(drw.run_mode.data != 2 && pattern == 2)
    {
        Disenable_Motor(0);
        usleep(1000);
        Set_RobStrite_Motor_parameter(0X7005, Speed_control_mode, Set_mode);
        usleep(1000);
        Get_RobStrite_Motor_parameter(0x7005);
        usleep(1000);
        enable_motor();
        usleep(1000);
        Motor_Set_All.set_motor_mode = Speed_control_mode;
    }

    Set_RobStrite_Motor_parameter(0X7018, Motor_Set_All.set_limit_cur, Set_parameter);
    Set_RobStrite_Motor_parameter(0X7022, Motor_Set_All.set_acc,   Set_parameter);
    Set_RobStrite_Motor_parameter(0X700A, Motor_Set_All.set_speed, Set_parameter);

    return std::make_tuple(position_, velocity_, torque_, temperature_);
}

float RobStrideMotor::read_initial_position()
{
    // Note: This blocking read might need refactoring if your application is single threaded.
    // For now, receive() handles timeout.
    auto result = receive(1.0); 
    if (result) {
        auto [type, extra, host, data] = *result;
        if (type == Communication_Type_MotorRequest) { // Type 2
             uint16_t p_uint = (data[0] << 8) | data[1];
             float pos = uint_to_float(p_uint, -4 * M_PI, 4 * M_PI, 16);
             return pos;
        }
    }
    return 0.0f;
}

void RobStrideMotor::Get_RobStrite_Motor_parameter(uint16_t Index)
{
    struct can_frame frame{};
    frame.can_id = (Communication_Type_GetSingleParameter << 24) | (master_id << 8) | motor_id;
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 8;

    frame.data[0] = Index;
    frame.data[1] = Index >> 8;
    memset(&frame.data[2], 0, 6);

    send_can_frame_niren(frame);
    std::cout << "[✓] Get single parameter command sent. " << std::hex << Index;
    std::cout << std::dec << std::endl;
    receive_status_frame();
}

std::tuple<float, float, float, float> RobStrideMotor::RobStrite_Motor_PosPP_control(float Angle, float Speed, float Acceleration)
{
    if(drw.run_mode.data != 1 && pattern == 2)
    {
        Disenable_Motor(0);
        usleep(1000);
        Set_RobStrite_Motor_parameter(0X7005, PosPP_control_mode, Set_mode);
        usleep(1000);
        Get_RobStrite_Motor_parameter(0x7005);
        usleep(1000);
        enable_motor();
        usleep(1000);
        Motor_Set_All.set_motor_mode = PosPP_control_mode;
    }

	Motor_Set_All.set_speed = Speed;
	Motor_Set_All.set_acc   = Acceleration;
	Motor_Set_All.set_angle = Angle;

	Set_RobStrite_Motor_parameter(0X7024, Motor_Set_All.set_speed, Set_parameter);
	Set_RobStrite_Motor_parameter(0X7025, Motor_Set_All.set_acc,   Set_parameter);
	Set_RobStrite_Motor_parameter(0X7016, Motor_Set_All.set_angle, Set_parameter);

    return std::make_tuple(position_, velocity_, torque_, temperature_);
}

std::tuple<float, float, float, float> RobStrideMotor::RobStrite_Motor_Current_control(float IqCommand, float IdCommand) 
{
    if(drw.run_mode.data != 3)
    {
        Disenable_Motor(0);
        usleep(1000);
        Set_RobStrite_Motor_parameter(0X7005, Elect_control_mode, Set_mode);
        usleep(1000);
        Get_RobStrite_Motor_parameter(0x7005);
        usleep(1000);
        enable_motor();
        usleep(1000);
        Motor_Set_All.set_motor_mode = Elect_control_mode;
    }

    Motor_Set_All.set_iq = IqCommand;
    Motor_Set_All.set_id = IdCommand;

    // Warning: Logic here might need adjustment based on drw.iq_ref updates
    // Assuming float_to_uint conversion is handled by motor logic or unnecessary for direct parameter write
    // If param write expects float, pass float directly.
    // Motor_Set_All.set_iq = float_to_uint(Motor_Set_All.set_iq, -11.0f,11.0f, 16);
    
    Set_RobStrite_Motor_parameter(0X7006, Motor_Set_All.set_iq, Set_parameter);
    Set_RobStrite_Motor_parameter(0X7007, Motor_Set_All.set_id, Set_parameter);

    return std::make_tuple(position_, velocity_, torque_, temperature_);
}

void RobStrideMotor::RobStrite_Motor_Set_Zero_control()
{
	Set_RobStrite_Motor_parameter(0X7005, Set_Zero_mode, Set_mode);
}

void RobStrideMotor::Disenable_Motor(uint8_t clear_error)
{
    struct can_frame frame{};
    frame.can_id = (Communication_Type_MotorStop << 24) | (master_id << 8) | motor_id;
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 8;
    memset(frame.data, 0, 8);
    frame.data[0] = clear_error;

    send_can_frame_niren(frame); 
    std::cout << "[✓] Motor disable command sent." << std::endl;
    receive_status_frame();
}

void RobStrideMotor::Set_CAN_ID(uint8_t Set_CAN_ID)
{
	Disenable_Motor(0);

    struct can_frame frame{};
    frame.can_id = (Communication_Type_Can_ID<<24) | (Set_CAN_ID<<16) | (master_id << 8) | motor_id;
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 8;
    memset(frame.data, 0, 8);

    send_can_frame_niren(frame);
    std::cout << "[✓] Motor Set_CAN_ID command sent." << std::endl;
}

std::tuple<float, float, float, float> RobStrideMotor::RobStrite_Motor_PosCSP_control(float Speed, float Angle)
{
	Motor_Set_All.set_speed = Speed;
	Motor_Set_All.set_angle = Angle;
	if (drw.run_mode.data != 5 && pattern == 2)
	{
        Disenable_Motor(0);
        usleep(1000);
		Set_RobStrite_Motor_parameter(0X7005, PosCSP_control_mode, Set_mode);
        usleep(1000);
		Get_RobStrite_Motor_parameter(0x7005);
        usleep(1000);
        enable_motor();
        usleep(1000);
		Motor_Set_All.set_motor_mode = PosCSP_control_mode;
	}
    // Note: Assuming logic for float conversion is correct for your application requirements
	// Motor_Set_All.set_speed = float_to_uint(Motor_Set_All.set_speed, 0.0f, ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).velocity, 16);
	
    Set_RobStrite_Motor_parameter(0X7017, Motor_Set_All.set_speed, Set_parameter);
	Set_RobStrite_Motor_parameter(0X7016, Motor_Set_All.set_angle, Set_parameter);

    return std::make_tuple(position_, velocity_, torque_, temperature_);
}

void RobStrideMotor::Set_ZeroPos()
{
	Disenable_Motor(0);

    if(drw.run_mode.data != 4)
    {
        Set_RobStrite_Motor_parameter(0X7005, Speed_control_mode, Set_mode);
        usleep(1000);
        Get_RobStrite_Motor_parameter(0x7005);
        usleep(1000);
    }

    struct can_frame frame{};
    frame.can_id = (Communication_Type_SetPosZero << 24) | (master_id << 8) | motor_id;
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 8;
    memset(frame.data, 0, 8);
    frame.data[0] = 1;

    send_can_frame_niren(frame);
    std::cout << "[✓] Motor Set_ZeroPos command sent." << std::endl;
	enable_motor();
}