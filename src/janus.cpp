#include <Arduino.h>
#include "janus.h"

void PWMConfig::set_resolution(unsigned int depth)
{
    bit_depth = constrain(static_cast<int>(depth), 8, 15);
    analogWriteResolution(bit_depth);
}

unsigned int PWMConfig::get_resolution()
{
    return bit_depth;
}

unsigned int PWMConfig::max_value()
{
    return (1 << bit_depth);
}

float Escon50Config::rpm_to_dutycycle(float rpm)
{
    float rpm_norm = (rpm - rpm_ramp_low) / (rpm_ramp_high - rpm_ramp_low);
    return pwm_ramp_low + rpm_norm * (pwm_ramp_high - pwm_ramp_low);
}

void EsconPWMMotor::init()
{
    pinMode(pin_pwm, OUTPUT);
    pinMode(pin_enable, OUTPUT);
    pinMode(pin_direction, OUTPUT);

    set_enable(false);
}

void EsconPWMMotor::set_rpm(float rpm)
{
    unsigned int period = esc_config->rpm_to_dutycycle(abs(rpm)) * pwm_config->max_value();
    analogWrite(pin_pwm, period);
    digitalWrite(pin_direction, rpm < 0);
}

void EsconPWMMotor::set_enable(bool enabled)
{
    digitalWrite(pin_enable, enabled);
}

void ServoMotor::init()
{
    pinMode(pin_pwm, OUTPUT);
    s.attach(pin_pwm);
}

void ServoMotor::set_position(float radians)
{
    angle = constrain(radians, 0.0f, M_PI); // constrain 0 - 180 degrees
    float servo_angle = angle * (180.0f / M_PI);
    s.write(servo_angle);
}

float ServoMotor::get_position()
{
    return angle;
}

void OpenCRDynamixelBridge::send_control_packet(control_packet p)
{
    uint8_t tx_packet[sizeof(p) + 1];
    tx_packet[0] = PKT_CONTROL;
    memcpy(tx_packet + 1, &p, sizeof(p));
    packet_serial.send(tx_packet, sizeof(p) + 1);
}

void OpenCRDynamixelBridge::send_control_packet(dynamixel_state s_1, dynamixel_state s_2)
{
    control_packet p;
    convert_dxl_to_packet(0, s_1, &p);
    convert_dxl_to_packet(1, s_2, &p);
    send_control_packet(p);
}

void OpenCRDynamixelBridge::send_status_packet()
{
    uint8_t tx_packet[1];
    tx_packet[0] = PKT_STATUS;
    packet_serial.send(tx_packet, sizeof(tx_packet));
}

void OpenCRDynamixelBridge::send_arm_packet(bool armed)
{
    uint8_t tx_packet[2];
    tx_packet[0] = PKT_ARM;
    tx_packet[1] = armed;
    packet_serial.send(tx_packet, sizeof(tx_packet));
}

void OpenCRDynamixelBridge::convert_dxl_to_packet(int motor_number, dynamixel_state s, control_packet *p)
{
    p->goal[motor_number] = (s.radians / M_TWOPI) * 4095.0f;
    p->velocity[motor_number] = s.velocity / 0.229f;
    p->acceleration[motor_number] = s.acceleration / 214.577f;
    Serial.println(p->goal[motor_number]);
}

void OpenCRDynamixelBridge::on_packet_received(const uint8_t *buffer, size_t size)
{
    status_packet packet;
    memcpy(&packet, buffer, size);
}

void OpenCRDynamixelBridge::init()
{
    static_cast<HardwareSerial*>(serial)->begin(baudrate);
    packet_serial.setStream(serial);
    packet_serial.setPacketHandler(on_packet_received);
}

void OpenCRDynamixelBridge::update()
{
    unsigned long time = millis();
    static unsigned long last_status_packet;
    
    packet_serial.update();

    if (time - last_status_packet >= 100) {
        last_status_packet = time;
        send_status_packet();
    }
}

void OpenCRDynamixelBridge::send_arm(bool armed)
{
    send_arm_packet(armed);
}

void OpenCRDynamixelBridge::send_motors()
{
    send_control_packet(motor_states);
}

void OpenCRDynamixelBridge::id_set_state(unsigned char id, dynamixel_state d)
{
    Serial.print("Set state id");
    Serial.print(id);
    Serial.print(d.radians);
    convert_dxl_to_packet(id, d, &motor_states);
}

void OpenCRDynamixelMotor::init()
{
}

void OpenCRDynamixelMotor::set_position(float rad)
{
    radians = rad;
}

float OpenCRDynamixelMotor::get_position()
{
    return radians;
}

void OpenCRDynamixelMotor::update_bridge()
{
    dynamixel_state s;
    s.radians = radians;
    s.velocity = velocity;
    s.acceleration = acceleration;
    bridge->id_set_state(id, s);
}
