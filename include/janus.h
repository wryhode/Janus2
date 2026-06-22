#pragma once

#include <Arduino.h>
#include <Servo.h>
#include <PacketSerial.h>

struct dynamixel_state {
    float radians;
    float velocity;
    float acceleration;
};

class PWMConfig {
    private:
        unsigned int bit_depth = 8;

    public:
        void set_resolution(unsigned int depth);
        unsigned int get_resolution();
        unsigned int max_value();
};

class Escon50Config {
    private:
        float rpm_ramp_low = 0.0f;
        float rpm_ramp_high = 600.0f;
        float pwm_ramp_low = 0.1;
        float pwm_ramp_high = 0.9;
    
    public:
        Escon50Config(float low_rpm, float high_rpm, float low_pwm, float high_pwm) :
            rpm_ramp_low{ low_rpm }, rpm_ramp_high{ high_rpm }, pwm_ramp_low{ low_pwm }, pwm_ramp_high{ high_pwm } {};
        float rpm_to_dutycycle(float rpm);
};

class VelocityMotor {
    public:
        virtual ~VelocityMotor() = default;
        virtual void init() = 0;
        virtual void set_rpm(float rpm) = 0;
};

class EsconPWMMotor : public VelocityMotor {
    private:
        unsigned int pin_direction;
        unsigned int pin_enable;
        unsigned int pin_pwm;
        Escon50Config* esc_config;
        PWMConfig* pwm_config;

    public:
        EsconPWMMotor(unsigned int p_dir, unsigned int p_ena, unsigned int p_pwm, Escon50Config* esc_conf, PWMConfig* pwm_conf) :
            pin_direction{ p_dir }, pin_enable{ p_ena }, pin_pwm{ p_pwm }, esc_config{ esc_conf }, pwm_config{ pwm_conf } {};
        
        void init() override;
        void set_rpm(float rpm) override;
        void set_enable(bool enabled);
};

class PositionMotor {
    public:
        virtual ~PositionMotor() = default;
        virtual void init() = 0;
        virtual void set_position(float radians) = 0;
        virtual float get_position() = 0;
};

class ServoMotor : public PositionMotor{
    private:
        float angle;
        unsigned int pin_pwm;
        PWMConfig* pwm_config;
        Servo s;

    public:
        ServoMotor(unsigned int p_pwm, PWMConfig* pwm_cfg) : pin_pwm{ p_pwm }, pwm_config{ pwm_cfg } {};
        void init() override;
        void set_position(float radians) override;
        float get_position() override;
};

class OpenCRDynamixelBridge {
    private:
        struct control_packet {
            uint32_t goal[2];
            uint32_t velocity[2];
            uint32_t acceleration[2];
        };

        struct status_packet {
            int32_t current_position[2];
            int32_t current_velocity[2];
            int32_t current[2];
            int32_t input_voltage[2];
            int32_t temperature[2];
            int32_t moving[2];
        };

        enum PacketTypes {
            PKT_CONTROL = 0xff,
            PKT_ARM = 0x80,
            PKT_STATUS = 0x00
        };

        Stream* serial;
        unsigned long baudrate;
        PacketSerial packet_serial;
        control_packet motor_states;

        void send_control_packet(control_packet p);
        void send_control_packet(dynamixel_state s_1, dynamixel_state s_2);
        void send_status_packet();
        void send_arm_packet(bool armed);
        void convert_dxl_to_packet(int motor_number, dynamixel_state s, control_packet* p);
        static void on_packet_received(const uint8_t* buffer, size_t size);
    
    public:
        OpenCRDynamixelBridge(Stream* ser, unsigned long baudrate) :
            serial{ ser }, baudrate{ baudrate } {};
        void init();
        void update();
        void send_arm(bool armed);
        void send_motors();
        void id_set_state(unsigned char id, dynamixel_state d);
};

class OpenCRDynamixelMotor : public PositionMotor {
    private:
        unsigned char id;
        float radians;
        float velocity;
        float acceleration;
        OpenCRDynamixelBridge* bridge;
    
    public:
        OpenCRDynamixelMotor(unsigned char motor_id, float velocity, float acceleration, OpenCRDynamixelBridge* opencr_bridge) :
            id{ motor_id }, velocity{ velocity }, acceleration{ acceleration }, bridge{ opencr_bridge } {};

        void init() override;
        void set_position(float rad) override;
        float get_position() override;
        void update_bridge();
};