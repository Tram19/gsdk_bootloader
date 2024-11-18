#include "read_imu.h"

// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
static Serial_Port *serial_port_quit;
static Gimbal_Interface *gimbal_interface_quit;

void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate, int &imu_rate)
{
    // string for command line usage
    const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate> -p <path>";

    // Read input arguments
    for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

        // Help
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("%s\n", commandline_usage);
            throw EXIT_FAILURE;
        }

        // UART device ID
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
            if (argc > i + 1) {
                uart_name = argv[i + 1];

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Baud rate
        if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
            if (argc > i + 1) {
                baudrate = atoi(argv[i + 1]);

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Path
        if (strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--rate") == 0) {
            if (argc > i + 1) {
                imu_rate = atoi(argv[i + 1]);

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }
    }
    
    // end: for each input argument
    // Done!
}
void quit_handler(int sig)
{
    printf("\n");
    printf("TERMINATING AT USER REQUEST\n");
    printf("\n");

    // autopilot interface
    try {
        gimbal_interface_quit->handle_quit(sig);

    } catch (int error) {}

    // serial port
    try {
        serial_port_quit->handle_quit(sig);

    } catch (int error) {}

    // end program here
    exit(0);
}
int main(int argc, char **argv)
{
    // --------------------------------------------------------------------------
    //   PARSE THE COMMANDS
    // --------------------------------------------------------------------------
    // Default input arguments   
    #ifdef __APPLE__
    char *uart_name = (char *)"/dev/tty.usbmodem1";
    #else
    char *uart_name = (char *)"/dev/ttyUSB0";
    #endif
    int baudrate = 115200;
    string path = "";
    // do the parse, will throw an int if it fails

    int imu_rate  = 100;
    parse_commandline(argc, argv, uart_name, baudrate, imu_rate);


    Serial_Port serial_port(uart_name, baudrate);

    Gimbal_Interface gimbal_interface(&serial_port, 1, MAV_COMP_ID_ONBOARD_COMPUTER, Gimbal_Interface::MAVLINK_GIMBAL_V2, MAVLINK_COMM_0);   
    
    serial_port_quit      = &serial_port;
    gimbal_interface_quit = &gimbal_interface;
    signal(SIGINT, quit_handler);

    serial_port.start();
    gimbal_interface.start();
    usleep(2000000); // wait for interface start
    int result = -1;
    int timeout = 0;
    do
    {
        usleep(10);
        result = gimbal_interface.set_msg_raw_imu_rate(imu_rate);
        GSDK_DebugInfo("Try to set raw imu messages rate: %d\r\n",imu_rate);
        if(timeout++ > 30){
            GSDK_DebugWarning("Cannot set up the raw imu messages rate due to timeout!");
            break;
        }
    }  while (result != 0);
    GSDK_DebugSuccess("Raw imu messages rate set successfully.");
    Gimbal_Interface::imu_t my_imu;
    while (1)
    {
        my_imu =  gimbal_interface.get_gimbal_raw_imu();
        GSDK_DebugInfo("\n\nRaw imu:  xacc:%d, yacc:%d, zacc:%d, xgyro:%d, xgyro:%d, xgyro:%d(raw)\n",
                                                        my_imu.accel.x,
                                                        my_imu.accel.y,
                                                        my_imu.accel.z,
                                                        my_imu.gyro.x,
                                                        my_imu.gyro.y,
                                                        my_imu.gyro.z);
        usleep(100000);
 
    }
    
    return 1;
}
