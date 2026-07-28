#include "robot.h"

Robot::Robot() {

}

Robot::~Robot() {

}

#if defined(ARCH_ARM64)
errno_t Robot::login_in(const char *ip) {
    if (sim_mode_){
        std::cout << "[sim mode]::login in" << std::endl;
        return 0;
    }
    else
        return jaka_robot_.login_in(ip);
}
#else
errno_t Robot::login_in(const char *ip, int port) {
    if (sim_mode_){
        std::cout << "[sim mode]::login in" << std::endl;
        return 0;
    }
    else
        return jaka_robot_.login_in(ip, port);
}
#endif

errno_t Robot::power_on() {
    if (sim_mode_){
        std::cout << "[sim mode]::power on" << std::endl;
        return 0;
    }
    else
        return jaka_robot_.power_on();
}

errno_t Robot::power_off() {
    if (sim_mode_){
        std::cout << "[sim mode]::power off" << std::endl;
        return 0;
    }
    else
        return jaka_robot_.power_off(); 
}

errno_t Robot::enable_robot() {
    if (sim_mode_){
        std::cout << "[sim mode]::enable robot" << std::endl;
        return 0;
    }
    else
        return jaka_robot_.enable_robot();
}   

errno_t Robot::disable_robot() {
    if (sim_mode_){
        std::cout << "[sim mode]::disable robot" << std::endl;
        return 0;
    }
    else
        return jaka_robot_.disable_robot();
}

errno_t Robot::set_user_frame_id(int id) {      
    if (sim_mode_){
        std::cout << "[sim mode]::set user frame id" << std::endl;
        return 0;
    }
    else
        return jaka_robot_.set_user_frame_id(id);
}

errno_t Robot::clear_error() {
    if (sim_mode_){
        std::cout << "[sim mode]::clear error" << std::endl;
        return 0;
    }
    else
        return jaka_robot_.clear_error();
}


errno_t Robot::get_joint_position(JointValue *joint_pos) {
    if (sim_mode_){
        std::cout << "[sim mode]::get joint position" << std::endl;
        joint_pos->jVal[0] = joint_pos_current_sim_.jVal[0];
        joint_pos->jVal[1] = joint_pos_current_sim_.jVal[1];
        joint_pos->jVal[2] = joint_pos_current_sim_.jVal[2];
        joint_pos->jVal[3] = joint_pos_current_sim_.jVal[3];
        joint_pos->jVal[4] = joint_pos_current_sim_.jVal[4];
        joint_pos->jVal[5] = joint_pos_current_sim_.jVal[5];
        joint_pos->jVal[6] = joint_pos_current_sim_.jVal[6];        
        return 0;
    }
    else
        return jaka_robot_.get_joint_position(joint_pos);
}

errno_t Robot::get_tcp_position(CartesianPose *cart_pos) {
    if (sim_mode_){
        std::cout << "[sim mode]::get tcp position" << std::endl;
        cart_pos->tran.x = cart_pos_current_sim_.tran.x;
        cart_pos->tran.y = cart_pos_current_sim_.tran.y;
        cart_pos->tran.z = cart_pos_current_sim_.tran.z;
        cart_pos->rpy.rx = cart_pos_current_sim_.rpy.rx;
        cart_pos->rpy.ry = cart_pos_current_sim_.rpy.ry;
        cart_pos->rpy.rz = cart_pos_current_sim_.rpy.rz;
        return 0;

    }
    else
        return jaka_robot_.get_tcp_position(cart_pos);
}

errno_t Robot::get_robot_state(RobotState *state) {
    if (sim_mode_){
        std::cout << "[sim mode]::get robot state" << std::endl;
        return 0;
    }
    else
        return jaka_robot_.get_robot_state(state);
}

errno_t Robot::get_robot_status(RobotStatus *status) {
    if (sim_mode_){
        std::cout << "[sim mode]::get robot status" << std::endl;
        status->inpos = 1;
        return 0;
    }
    else
        return jaka_robot_.get_robot_status(status);
}

errno_t Robot::joint_move(const JointValue *joint_pos, MoveMode move_mode, BOOL is_block, double speed) {
    if (sim_mode_){
        std::cout << "[sim mode]::joint move" << std::endl;
        joint_pos_current_sim_ = *joint_pos;
        return 0;
    }
    else
        return jaka_robot_.joint_move(joint_pos, move_mode, is_block, speed);
}

errno_t Robot::servo_move_enable(BOOL is_enable) {
    if (sim_mode_){
        std::cout << "[sim mode]::servo move enable" << std::endl;
        return 0;
    }
    else {
#if defined(ARCH_ARM64)
        return jaka_robot_.servo_move_enable(is_enable);
#else
        return jaka_robot_.servo_move_enable(is_enable, 0);
#endif
    }
}   

errno_t Robot::servo_p(CartesianPose *cart_pos, MoveMode move_mode, BOOL is_block) {
    if (sim_mode_){
        std::cout << "[sim mode]::servo p" << std::endl;
        cart_pos_current_sim_ = *cart_pos;
        return 0;
    }
    return jaka_robot_.servo_p(cart_pos, move_mode, is_block);
}

void Robot::set_sim_mode(bool sim_mode) {
    sim_mode_ = sim_mode;
    std::cout << "set sim mode to " << sim_mode_ << std::endl;
}


#if defined(ARCH_ARM64)
errno_t Robot::kine_inverse(const JointValue *ref_pos, const CartesianPose *cartesian_pose, JointValue *joint_pos) {
    if (sim_mode_){
        std::cout << "[sim mode]::kine inverse" << std::endl;
        joint_pos->jVal[0] = 0;
        joint_pos->jVal[1] = 0;
        joint_pos->jVal[2] = 0;
        joint_pos->jVal[3] = 0;
        joint_pos->jVal[4] = 0;
        joint_pos->jVal[5] = 0;
        joint_pos->jVal[6] = 0;
        return 0;
    }
    else
        return jaka_robot_.kine_inverse(ref_pos, cartesian_pose, joint_pos);
}
#else
errno_t Robot::kine_inverse(const JointValue *ref_pos, const CartesianPose *cartesian_pose, JointValue *joint_pos) {
    if (sim_mode_){
        std::cout << "[sim mode]::kine inverse" << std::endl;
        joint_pos->jVal[0] = 0;
        joint_pos->jVal[1] = 0;
        joint_pos->jVal[2] = 0;
        joint_pos->jVal[3] = 0;
        joint_pos->jVal[4] = 0;
        joint_pos->jVal[5] = 0;
        joint_pos->jVal[6] = 0;
        return 0;
    }
    else
        return jaka_robot_.kine_inverse(0,ref_pos, cartesian_pose, joint_pos);
}
#endif

errno_t Robot::servo_j(
    const JointValue *joint_pos,
    MoveMode move_mode,
    unsigned int step_num) {
    if (sim_mode_){
        std::cout << "[sim mode]::servo j" << std::endl;
        joint_pos_current_sim_ = *joint_pos;
        return 0;
    }
    else
        return jaka_robot_.servo_j(joint_pos, move_mode, step_num);
}
 
errno_t Robot::servo_move_use_joint_LPF(double cutoffFreq){
    if (sim_mode_){
        std::cout << "[sim mode]::add filter" << std::endl;
        return 0;
    }
    else
        return jaka_robot_.servo_move_use_joint_LPF(cutoffFreq);
}

#if !defined(ARCH_ARM64)
errno_t Robot::edg_get_stat(
    unsigned char robot_index,
    JointValue *joint_pos,
    CartesianPose *cartesian_pose) {
    if (sim_mode_) {
        *joint_pos = joint_pos_current_sim_;
        *cartesian_pose = cart_pos_current_sim_;
        return ERR_SUCC;
    }
    return jaka_robot_.edg_get_stat(robot_index, joint_pos, cartesian_pose);
}

errno_t Robot::edg_servo_j(
    unsigned char robot_index,
    const JointValue *joint_pos,
    MoveMode move_mode,
    unsigned int step_num) {
    if (sim_mode_){
        std::cout << "[sim mode]::servo j" << std::endl;
        return 0;
    }
    else
        return jaka_robot_.edg_servo_j(
            robot_index, joint_pos, move_mode, step_num);

}
errno_t Robot::edg_send(){
    if (sim_mode_){
        std::cout << "[sim mode]::servo j" << std::endl;
        return 0;
    }
    else
        return jaka_robot_.edg_send();
}
#endif


// add signal to robot： RS485
void Robot::Add_Signal()
{    
    memcpy(sign_info_angles[0].sig_name, "Little_fin", sizeof("Little_fin"));
    sign_info_angles[0].chn_id = 1;
    sign_info_angles[0].sig_addr = 0x060A; // 1546
    sign_info_angles[0].frequency = 10;
    sign_info_angles[0].sig_type = 3;
    memcpy(sign_info_angles[1].sig_name, "Ring_fin", sizeof("Ring_fin"));
    sign_info_angles[1].chn_id = 1;
    sign_info_angles[1].sig_addr = 0x060C; // 1548
    sign_info_angles[1].frequency = 10;
    sign_info_angles[1].sig_type = 3;
    memcpy(sign_info_angles[2].sig_name, "Middle_fin", sizeof("Middle_fin"));
    sign_info_angles[2].chn_id = 1;
    sign_info_angles[2].sig_addr = 0x060E; // 1550
    sign_info_angles[2].frequency = 10;
    sign_info_angles[2].sig_type = 3;
    memcpy(sign_info_angles[3].sig_name, "Index_fin", sizeof("Index_fin"));
    sign_info_angles[3].chn_id = 1;
    sign_info_angles[3].sig_addr = 0x0610; // 1552
    sign_info_angles[3].frequency = 10;
    sign_info_angles[3].sig_type = 3;
    memcpy(sign_info_angles[4].sig_name, "Thumb_crooked", sizeof("Thumb_crooked"));
    sign_info_angles[4].chn_id = 1;
    sign_info_angles[4].sig_addr = 0x0612; // 1554
    sign_info_angles[4].frequency = 10;
    sign_info_angles[4].sig_type = 3;
    memcpy(sign_info_angles[5].sig_name, "Thumb_rotary", sizeof("Thumb_rotary"));
    sign_info_angles[5].chn_id = 1;
    sign_info_angles[5].sig_addr = 0x0614; // 1556
    sign_info_angles[5].frequency = 10;
    sign_info_angles[5].sig_type = 3;
    

    memcpy(sign_info_angles[6].sig_name, "Index_fin_force", sizeof("Index_fin_force"));
    sign_info_angles[6].chn_id = 1;
    sign_info_angles[6].sig_addr = 0x0634; // 1588
    sign_info_angles[6].frequency = 10;
    sign_info_angles[6].sig_type = 3;

    memcpy(sign_info_angles[7].sig_name, "Thumb_rotary_force", sizeof("Thumb_rotary_force"));
    sign_info_angles[7].chn_id = 1;
    sign_info_angles[7].sig_addr = 0x0638; // 1592
    sign_info_angles[7].frequency = 10;
    sign_info_angles[7].sig_type = 3;
    for (int i = 0; i < 8; i++)
    {
        const int result = jaka_robot_.add_tio_rs_signal(sign_info_angles[i]);
        if (result != ERR_SUCC)
        {
            std::cerr
                << "add_tio_rs_signal failed at index " << i
                << ", error=" << result << std::endl;
        }
    }
}


// 更改485通道二的通讯模式,手的通讯模式是modbus rtu
errno_t Robot::change_mode()
{
    /*获取通道1的通信协议
    int chn_id=0;
    int chn_mode;
    m_JAKAZuRobot.get_rs485_chn_mode(chn_id,&chn_mode);
    qDebug()<<"通道"<<chn_id+1<<" chn_mode :"<<chn_mode;
    m_JAKAZuRobot.set_rs485_chn_mode(0,1);
*/

    // 获取通道2的通信协议
    int chn_id_ = 1;
    int chn_mode_;
    jaka_robot_.get_rs485_chn_mode(chn_id_, &chn_mode_);
    std::cout << "通道" << chn_id_ + 1 << " chn_mode :" << chn_mode_ << std::endl;

    jaka_robot_.set_rs485_chn_mode(1, 0);
    /*RS485通道1通讯参数配置
    int ret;
    hand.chn_id=0;
    hand.parity=78;
    hand.baudrate=115200;
    hand.databit=8;
    hand.stopbit=1;
    hand.slaveId=1;
    ret=m_JAKAZuRobot.set_rs485_chn_comm(hand);
    ERR(ret,"RS485通道1通讯参数配置");
    qDebug()<<"RS485通道"<<hand.chn_id+1<<"配置完毕";
    */
    // RS485通道2通讯参数配置
    hand.chn_id = 1;
    hand.parity = 78;
    hand.baudrate = 115200;
    hand.databit = 8;
    hand.stopbit = 1;
    hand.slaveId = 1;
    return jaka_robot_.set_rs485_chn_comm(hand);
    
}

void Robot::write_SeriesPort_modbus(uint16_t angle_1, uint16_t angle_2, uint16_t angle_3, uint16_t angle_4, uint16_t angle_5, uint16_t angle_6, uint16_t Register_Number)
{
    uint8_t Register_Number_H = (Register_Number >> 8) & 0xFF;
    uint8_t Register_Number_L = Register_Number & 0xFF;
    uint8_t Register_amount_H = 0x00;
    uint8_t Register_amount_L = 0x06;
    uint8_t BYTE_LEN = 0x12;
    std::cout << "write_SeriesPort_modbus" << std::endl;
    std::vector<uint16_t> ARRAY;
    ARRAY.push_back(angle_1);
    ARRAY.push_back(angle_2);
    ARRAY.push_back(angle_3);
    ARRAY.push_back(angle_4);
    ARRAY.push_back(angle_5);
    ARRAY.push_back(angle_6);

    std::vector<uint8_t> ARRAY2;
    ARRAY2.push_back((ARRAY[0] >> 8) & 0xff);
    ARRAY2.push_back(ARRAY[0] & 0xff);
    ARRAY2.push_back((ARRAY[1] >> 8) & 0xff);
    ARRAY2.push_back(ARRAY[1] & 0xff);
    ARRAY2.push_back((ARRAY[2] >> 8) & 0xff);
    ARRAY2.push_back(ARRAY[2] & 0xff);
    ARRAY2.push_back((ARRAY[3] >> 8) & 0xff);
    ARRAY2.push_back(ARRAY[3] & 0xff);
    ARRAY2.push_back((ARRAY[4] >> 8) & 0xff);
    ARRAY2.push_back(ARRAY[4] & 0xff);
    ARRAY2.push_back((ARRAY[5] >> 8) & 0xff);
    ARRAY2.push_back(ARRAY[5] & 0xff);


    uint8_t data[] = {SLAVE_STATION_ADD, FUNCTION_MULTI_REGISTER, Register_Number_H, Register_Number_L, Register_amount_H, Register_amount_L, BYTE_LEN, ARRAY2[0], ARRAY2[1], ARRAY2[2], ARRAY2[3], ARRAY2[4], ARRAY2[5], ARRAY2[6], ARRAY2[7], ARRAY2[8], ARRAY2[9], ARRAY2[10], ARRAY2[11]};
    int ret = jaka_robot_.send_tio_rs_command(1, data, sizeof(data));
    std::cout << "ret   " << ret << std::endl;
}
