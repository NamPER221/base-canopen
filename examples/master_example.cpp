/**
 * @file master_example.cpp
 * @brief CANopen Master Example
 */

#include <canopen/device/canopen_device.hpp>
#include <canopen/can/raw/socket_can.hpp>
#include <iostream>
#include <thread>
#include <chrono>

using namespace canopen;

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <can_interface>\n";
        std::cerr << "Example: " << argv[0] << " can0\n";
        return 1;
    }

    std::string interface = argv[1];

    std::cout << "CANopen Master Example\n";
    std::cout << "=====================\n\n";
    std::cout << "Interface: " << interface << "\n\n";

    // Create SocketCAN
    SocketCAN can(interface);

    // Open CAN interface
    std::cout << "Opening CAN interface...\n";
    int result = can.open();
    if (result < 0) {
        std::cerr << "Failed to open CAN interface: " << can.get_error_str() << "\n";
        return 1;
    }
    std::cout << "CAN interface opened successfully.\n";

    // Create Master
    CANopenMaster master(0, nullptr);  // Node ID 0 for master
    master.init();

    // Add slaves
    master.add_slave(1, 1000);  // Node 1 with 1000ms heartbeat
    master.add_slave(2, 1000);  // Node 2 with 1000ms heartbeat
    master.add_slave(3, 1000);  // Node 3 with 1000ms heartbeat

    std::cout << "Added 3 slaves for monitoring.\n";

    // Start master
    master.start();
    std::cout << "Master started.\n\n";

    std::cout << "Monitoring CAN bus...\n";
    std::cout << "Press Ctrl+C to exit.\n\n";

    // Main loop - read and process frames
    can_frame frame;
    while (true) {
        struct timespec timeout = {0, 10000000};  // 10ms
        ssize_t n = can.read(frame, &timeout);

        if (n > 0) {
            std::cout << "RX: ID=0x" << std::hex << frame.can_id << std::dec
                      << " DLC=" << (int)frame.can_dlc << " Data=";
            for (int i = 0; i < frame.can_dlc; i++) {
                std::cout << std::hex << (int)frame.data[i] << " " << std::dec;
            }
            std::cout << "\n";
        }

        // Small delay
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Cleanup
    master.shutdown();
    can.close();

    std::cout << "\nExiting.\n";
    return 0;
}
