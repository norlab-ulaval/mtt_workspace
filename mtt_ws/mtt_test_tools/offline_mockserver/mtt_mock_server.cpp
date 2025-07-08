#include <iostream>
#include <string>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>

#ifndef CAN_SFF_MASK
#define CAN_SFF_MASK 0x7FF
#endif
#ifndef CAN_EFF_MASK
#define CAN_EFF_MASK 0x1FFFFFFF
#endif
#ifndef CAN_EFF_FLAG
#define CAN_EFF_FLAG 0x80000000U
#endif

// MTT System CAN IDs
constexpr uint32_t CAN_CONTROL_FRAME = 0x100;
constexpr uint32_t CAN_REMOTE_FRAME = 0x001;
constexpr uint32_t CAN_MAIN = 0x2FF;
constexpr uint32_t CAN_BMS_TRAME_1 = 0x600;
constexpr uint32_t CAN_BMS_TRAME_2 = 0x601;
constexpr uint32_t CAN_BMS_TRAME_3 = 0x602;
constexpr uint32_t CAN_MAIN_BOARD1 = 0x300;
constexpr uint32_t CAN_MAIN_BOARD2 = 0x301;

// Control frame byte positions
constexpr uint8_t MTT_SWITCHES_VEHICLE_TYPE = 0;
constexpr uint8_t MTT_SWITCHES_GLOBAL = 1;
constexpr uint8_t MTT_ANALOG_THROTTLE = 2;
constexpr uint8_t MTT_ANALOG_WINCH = 3;
constexpr uint8_t MTT_ANALOG_BRAKE = 4;
constexpr uint8_t MTT_ANALOG_STEER = 5;
constexpr uint8_t MTT_SWITCHES_DIRECTION_MODE = 6;

struct VehicleState {
    uint8_t vehicle_type = 0;
    uint8_t global_switches = 0;
    uint8_t throttle = 0;
    uint8_t winch = 0x7F;  // neutral position
    uint8_t brake = 0;
    uint8_t steer = 128;   // center position
    uint8_t direction_mode = 0;
    
    // Decoded state
    bool safety_unlocked = false;
    bool forward_direction = true;
    bool light_on = false;
    bool closed_loop_steer = false;
    
    // Simulated sensor data
    int16_t instant_tachometer = 0;
    uint32_t cumulative_ticks = 0;
    int8_t temp_a = 25;
    int8_t temp_b = 22;
    
    // Simulated BMS data
    float battery_voltage = 48.0f;
    float battery_current = 0.0f;
    uint8_t battery_soc = 85;
    int16_t battery_temps[4] = {25, 26, 24, 27};
    
    std::mutex state_mutex;
};

static VehicleState vehicle_state;
static std::atomic<bool> should_stop(false);
static int can_socket = -1;

void send_frame(int fd, const struct can_frame &frame) {
    if (write(fd, &frame, sizeof(frame)) != sizeof(frame)) {
        std::cerr << "[MTT_MOCK] Failed to send CAN frame: " << strerror(errno) << std::endl;
    }
}

void decode_control_frame(const uint8_t* data) {
    std::lock_guard<std::mutex> lock(vehicle_state.state_mutex);
    
    vehicle_state.vehicle_type = data[MTT_SWITCHES_VEHICLE_TYPE];
    vehicle_state.global_switches = data[MTT_SWITCHES_GLOBAL];
    vehicle_state.throttle = data[MTT_ANALOG_THROTTLE];
    vehicle_state.winch = data[MTT_ANALOG_WINCH];
    vehicle_state.brake = data[MTT_ANALOG_BRAKE];
    vehicle_state.steer = data[MTT_ANALOG_STEER];
    vehicle_state.direction_mode = data[MTT_SWITCHES_DIRECTION_MODE];
    
    // Bit-level decoding
    vehicle_state.safety_unlocked = (vehicle_state.global_switches & 0x80) != 0;
    vehicle_state.forward_direction = (vehicle_state.global_switches & 0x20) == 0;
    vehicle_state.light_on = (vehicle_state.global_switches & 0x40) == 0;
    vehicle_state.closed_loop_steer = (vehicle_state.direction_mode & 0x01) != 0;
    
    //TODO Verify if vehicule brake overrides throttle for safety
    if (vehicle_state.safety_unlocked) {
        int speed_factor = vehicle_state.forward_direction ? 1 : -1;
        int net_force = 0;
        
        if (vehicle_state.brake > 10) {  // threshold to avoid noise
            net_force = -vehicle_state.brake;
            vehicle_state.battery_current = 0;
        } else if (vehicle_state.throttle > 0) {
            net_force = vehicle_state.throttle;
            vehicle_state.battery_current = vehicle_state.throttle * 0.1f;
        } else {
            net_force = 0;
            vehicle_state.battery_current = 0;
        }
        
        vehicle_state.instant_tachometer = (net_force * speed_factor) / 4;
        vehicle_state.cumulative_ticks += abs(vehicle_state.instant_tachometer);
        
    } else {
        vehicle_state.instant_tachometer = 0;
        vehicle_state.battery_current = 0;
    }
    
    std::string vehicle_mode = "IDLE";
    if (vehicle_state.brake > 10) {
        vehicle_mode = "BRAKING";
    } else if (vehicle_state.throttle > 0) {
        vehicle_mode = "THROTTLING";
    }
    
    std::cout << "[MTT_MOCK] Control received - "
              << "Safety: " << (vehicle_state.safety_unlocked ? "UNLOCKED" : "LOCKED") << ", "
              << "Dir: " << (vehicle_state.forward_direction ? "FWD" : "REV") << ", "
              << "Mode: " << vehicle_mode << ", "
              << "Throttle: " << (int)vehicle_state.throttle << ", "
              << "Brake: " << (int)vehicle_state.brake << ", "
              << "Steer: " << (int)vehicle_state.steer << ", "
              << "Speed: " << vehicle_state.instant_tachometer << " ticks, "
              << "Current: " << vehicle_state.battery_current << "A"
              << std::endl;
}

void send_main_data() {
    std::lock_guard<std::mutex> lock(vehicle_state.state_mutex);
    
    struct can_frame frame = {};
    frame.can_id = CAN_MAIN;
    frame.can_dlc = 8;
    
    // Pack according to 0x2FF specification
    frame.data[0] = vehicle_state.temp_a;
    frame.data[1] = vehicle_state.temp_b;
    frame.data[2] = (vehicle_state.instant_tachometer >> 8) & 0xFF;
    frame.data[3] = vehicle_state.instant_tachometer & 0xFF;
    frame.data[4] = (vehicle_state.cumulative_ticks >> 24) & 0xFF;
    frame.data[5] = (vehicle_state.cumulative_ticks >> 16) & 0xFF;
    frame.data[6] = (vehicle_state.cumulative_ticks >> 8) & 0xFF;
    frame.data[7] = vehicle_state.cumulative_ticks & 0xFF;
    
    send_frame(can_socket, frame);
}

void send_bms_data() {
    std::lock_guard<std::mutex> lock(vehicle_state.state_mutex);
    
    // BMS Frame 1 - Battery cell temperatures
    struct can_frame frame1 = {};
    frame1.can_id = CAN_BMS_TRAME_1;
    frame1.can_dlc = 8;
    
    for (int i = 0; i < 4; i++) {
        frame1.data[i * 2] = (vehicle_state.battery_temps[i] >> 8) & 0xFF;
        frame1.data[i * 2 + 1] = vehicle_state.battery_temps[i] & 0xFF;
    }
    send_frame(can_socket, frame1);
    
    // BMS Frame 2 - Additional battery diagnostics (placeholder)
    struct can_frame frame2 = {};
    frame2.can_id = CAN_BMS_TRAME_2;
    frame2.can_dlc = 8;
    
    // TODO: Implement proper BMS frame 2 data according to specification
    // Simple placeholder data for frame 2
    frame2.data[0] = 0x00; // Reserved
    frame2.data[1] = 0x00; // Reserved
    frame2.data[2] = 0x00; // Reserved
    frame2.data[3] = 0x00; // Reserved
    frame2.data[4] = 0x00; // Reserved
    frame2.data[5] = 0x00; // Reserved
    frame2.data[6] = 0x00; // Reserved
    frame2.data[7] = 0x00; // Reserved
    send_frame(can_socket, frame2);
    
    // BMS Frame 3 - Battery status
    struct can_frame frame3 = {};
    frame3.can_id = CAN_BMS_TRAME_3;
    frame3.can_dlc = 8;
    
    frame3.data[0] = vehicle_state.battery_soc;
    
    uint16_t current_raw = (uint16_t)(vehicle_state.battery_current * 100);
    frame3.data[1] = (current_raw >> 8) & 0xFF;
    frame3.data[2] = current_raw & 0xFF;
    
    uint16_t voltage_raw = (uint16_t)(vehicle_state.battery_voltage * 100);
    frame3.data[3] = (voltage_raw >> 8) & 0xFF;
    frame3.data[4] = voltage_raw & 0xFF;
    
    send_frame(can_socket, frame3);
}

void periodic_sender_thread() {
    while (!should_stop) {
        send_main_data();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        send_bms_data();
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }
}

int main(int argc, char *argv[]) {
    std::string channel = "can0";
    int opt;
    while ((opt = getopt(argc, argv, "c:")) != -1) {
        if (opt == 'c') channel = optarg;
    }
    
    std::cout << "[MTT_MOCK] Starting MTT vehicle mock server on " << channel << std::endl;
    std::cout << "[MTT_MOCK] Listening for control frames on 0x100 (and 0x101 for current system compatibility)" << std::endl;
    std::cout << "[MTT_MOCK] Sending data on 0x2FF (main), 0x600-0x602 (BMS)" << std::endl;
    
    can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket < 0) { 
        perror("[MTT_MOCK] socket"); 
        return 1; 
    }
    
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, channel.c_str(), IFNAMSIZ);
    if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0) { 
        perror("[MTT_MOCK] ioctl"); 
        return 1; 
    }
    
    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("[MTT_MOCK] bind"); 
        return 1; 
    }
    
    // Start periodic sender thread
    std::thread sender_thread(periodic_sender_thread);
    
    // Main loop - listen for control frames
    struct can_frame frame;
    while (read(can_socket, &frame, sizeof(frame)) > 0) {
        uint32_t raw_id = frame.can_id;
        bool is_ext = raw_id & CAN_EFF_FLAG;
        uint32_t id = is_ext ? (raw_id & CAN_EFF_MASK) : (raw_id & CAN_SFF_MASK);
        
        if (id == CAN_CONTROL_FRAME || id == 0x101) {  // Support both control IDs
            if (frame.can_dlc >= 7) {
                decode_control_frame(frame.data);
            }
        }
    }
    
    should_stop = true;
    sender_thread.join();
    close(can_socket);
    return 0;
}
