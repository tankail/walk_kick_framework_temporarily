#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "serial_struct.hpp"
#include <stdint.h>
#include <unordered_map>
#include "parse_robot_params.hpp"

#define my_2pi (6.28318530717f)
#define my_pi (3.14159265358f)

#define MEM_INDEX_ID(id) ((id) - 1)    


enum motor_type 
{
    null = 0,

    /* 旧名称 */
    m3536_32,
    m4538_19,
    m5046_20,
    m5047_09,
    m5047_36,
    m5047_36_2,
    m4438_30,
    m4438_32,
    m6056_36,
    m5043_20,
    m7256_35,
    m60sg_35,
    m60bm_35,

    /* 新名称 */
    m3508_02,
    m3516_02,
    m3532_02,
    m4530_02,
    m5009_02,
    m5036_02,
    m6036_02,
    m5031_04,
    m7033_04,
    m7535_02,
    m3532_02_8353,
    m4530_02_8353,
    m5036_02_8353,

    mGeneral,  
    mNone,
};


const std::unordered_map<std::string, motor_type> motor_type2 = 
{
    {"NULL", motor_type::null},

    /* 旧名称 */
    {"3536_32", motor_type::m3536_32},
    {"4538_19", motor_type::m4538_19},
    {"5046_20", motor_type::m5046_20},
    {"5047_9", motor_type::m5047_09},
    {"5047_36", motor_type::m5047_36},    
    {"5047_36_2", motor_type::m5047_36_2},
    {"4438_30", motor_type::m4438_30},
    {"4438_32", motor_type::m4438_32},
    {"6056_36", motor_type::m6056_36},
    {"5043_20", motor_type::m5043_20}, 
    {"7256_35", motor_type::m7256_35},   
    {"60SG_35", motor_type::m60sg_35},
    {"60BM_35", motor_type::m60bm_35},  

    /* 新名称 */
    {"3508_02", motor_type::m3508_02},
    {"3516_02", motor_type::m3516_02},
    {"3532_02", motor_type::m3532_02},
    {"4530_02", motor_type::m4530_02},
    {"5009_02", motor_type::m5009_02},
    {"5036_02", motor_type::m5036_02},
    {"6036_02", motor_type::m6036_02},
    {"5031_04", motor_type::m5031_04},
    {"7033_04", motor_type::m7033_04},
    {"7535_02", motor_type::m7535_02},
    {"3532_02_8353", motor_type::m3532_02_8353},
    {"4530_02_8353", motor_type::m4530_02_8353},
    {"5036_02_8353", motor_type::m5036_02_8353},

    {"General", motor_type::mGeneral},  // 遇到暂无力矩修正系数的电机时临时用
    {"NONE", motor_type::mNone},        // 无修正，已在电机内部修正
};

const std::unordered_map<motor_type, float> motor_tqe_adj = 
{
    /* 旧名称 */
    {motor_type::m3536_32,   0.4581f},
    {motor_type::m5046_20,   0.5280f},
    {motor_type::m4538_19,   0.4450f},
    {motor_type::m5047_09,   0.5330f},
    {motor_type::m5047_36,   0.4938f},
    {motor_type::m5047_36_2, 0.8030f},
    {motor_type::m4438_30,   0.5256f},
    {motor_type::m4438_32,   0.5584f},
    {motor_type::m6056_36,   0.6770f},
    {motor_type::m5043_20,   0.9660f},
    {motor_type::m7256_35,   0.6770f},
    {motor_type::m60sg_35,   0.7942f},
    {motor_type::m60bm_35,   0.7942f},

    /* 新名称 */
    {motor_type::m3508_02,   0.37f},
    {motor_type::m3516_02,   0.37f},
    {motor_type::m3532_02,   0.37f},
    {motor_type::m4530_02,   0.62f},
    {motor_type::m5009_02,   0.71f},
    {motor_type::m5036_02,   0.67f},
    {motor_type::m6036_02,   0.66f},
    {motor_type::m5031_04,   0.39f},
    {motor_type::m7033_04,   0.84f},
    {motor_type::m7535_02,   0.73f},
    {motor_type::m3532_02_8353,   0.61f},
    {motor_type::m4530_02_8353,   0.64f},
    {motor_type::m5036_02_8353,   0.70f},
    
    {motor_type::mGeneral,   0.5000f},
    {motor_type::mNone,      1.0000f}
};


enum pos_vel_convert_type
{
    radian_2pi = 0,  // 弧度制
    angle_360,       // 角度制
    turns,           // 圈数
};

extern const std::unordered_map<std::string, motor_type> motor_type2;


class motor
{
private:
    int id, num, CANport_num, CANboard_num;
    motor_back_t data;
    std::string motor_name;
    motor_type type_ = motor_type::null;
    cdc_tr_message_s *p_cdc_tx_message = NULL;
    int id_max = 0;
    pos_vel_convert_type pos_vel_type = radian_2pi; 
    bool pos_limit_enable = false; 
    float pos_upper = 0.0f;
    float pos_lower = 0.0f;
    bool tor_limit_enable = false;
    float tor_upper = 0.0f;
    float tor_lower = 0.0f;
    cdc_rx_motor_version_s version = {0};
    uint8_t tqe_adjust_flag = 0xff;

public:
    motor_pos_vel_tqe_kp_kd_s cmd_int16_5param;
    int pos_limit_flag = 0;     // 0 表示正常，1 表示超出上限， -1 表示超出下限
    int tor_limit_flag = 0;     // 0 表示正常，1 表示超出上限

    motor(int _motor_num, int _CANport_num, int _CANboard_num, cdc_tr_message_s *_p_cdc_tx_message, int _id_max, MotorParams &motor_params);
    ~motor() {}

    inline int16_t pos_float2int(float in_data, uint8_t type);
    inline int16_t vel_float2int(float in_data, uint8_t type);
    inline int16_t tqe_float2int(float in_data, motor_type motor_type);
    inline float pos_int2float(int16_t in_data, uint8_t type);
    inline float vel_int2float(int16_t in_data, uint8_t type);
    inline float tqe_int2float(int16_t in_data, motor_type type);
    inline float pid_scale(float in_data, motor_type motor_type);
    inline int16_t kp_float2int(float in_data, uint8_t type, motor_type motor_type);
    inline int16_t ki_float2int(float in_data, uint8_t type, motor_type motor_type);
    inline int16_t kd_float2int(float in_data, uint8_t type, motor_type motor_type);
    inline int16_t int16_limit(int32_t data);
    uint16_t get_data_len(uint8_t mode, uint16_t num);

    void position(float position);
    void velocity(float velocity);
    void torque(float torque);
    void voltage(float voltage);
    void current(float current);
    void set_motorout(int16_t t_ms);
    void pos_vel_MAXtqe(float position, float velocity, float torque_max);
    void pos_vel_tqe_kp_kd(float position, float velocity, float torque, float Kp, float Kd);
    void pos_vel_kp_kd(float position, float velocity, float Kp, float Kd);
    void pos_vel_acc(float position, float velocity, float acc);

    void stop();
    void brake();
    void reset();
    void send_state_cmd();

    void fresh_data(uint8_t mode, uint8_t fault, int16_t position, int16_t velocity, int16_t torque);

    int get_motor_id();
    int get_motor_type();
    motor_type get_motor_enum_type();
    int get_motor_num();
    void set_motor_type(std::string type_str);
    int get_motor_belong_canport();
    int get_motor_belong_canboard();
    motor_pos_vel_tqe_kp_kd_s *return_pos_vel_tqe_kp_kd_p();
    size_t return_size_motor_pos_vel_tqe_kp_kd_s();
    motor_back_t *get_current_motor_state();
    std::string get_motor_name();
    cdc_rx_motor_version_s* get_version();
    void set_version(cdc_rx_motor_version_s &v);
    void print_version();
    void set_type(motor_type t);
    void set_tqe_adjust_flag(uint8_t flag);
    uint8_t get_tqe_adjust_flag();
    void set_num();
};
#endif