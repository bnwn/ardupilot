

uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_MMWRadar, 0);
if (uart != nullptr) {
    uint32_t baudrate = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_MMWRadar, 0);
    printf ("begin mmwradar uart. baudrate is: %d \n", baudrate);
    uart->begin(baudrate);
}
