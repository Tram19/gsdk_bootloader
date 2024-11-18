#include "bootloader_mavlink.h"

static void *start_bootloader_read_thread(void *args);

BootloaderMavlink::BootloaderMavlink(Serial_Port *serial_port , string path)
{
    this -> path = path;
    this -> _serial_port = serial_port;
}

BootloaderMavlink::~BootloaderMavlink()
{
    time_to_exit = true;
    this -> _serial_port->close_serial();
    this -> _serial_port = NULL;
    pthread_cancel(read_tid);
    binary_close(p_st);
}

bool BootloaderMavlink::init()
{
    if (_serial_port->status != SERIAL_PORT_OPEN)
    {
        _serial_port->open_serial();
    }
    int result = pthread_create(&read_tid, NULL, &start_bootloader_read_thread, this);
    if (result) throw result;

    // try to init parser
    p_st = binary_init();
    if (!p_st)
    {
        fprintf(stderr, " Parser failed to initialize\n");
        return false;
    }

	if (binary_open(p_st, path.c_str(), 0) != PARSER_ERR_OK) {
		fprintf(stderr, "Parser failed to open\n");
        return false;
	}
    init_flag = true;
    return true;
}

bool BootloaderMavlink::run()
{
    if (!init_flag)
    {
        printf("Bootloader Mavlink is not initialized yet!\n");
        return false;
    }
    uint8_t total_e = 0;
    while (1)
    {
        switch (state)
        {
        case CONNECT:
        {

        }
            break;
        case PARSER:
        {
            if (parse_bin_and_send_seq())
            {
                /* code */
            }
            return true;
        }
            break;
        
        default:
            break;
        }
        
    }
    
}
BootloaderMavlink::err_t BootloaderMavlink::parse_bin_and_send_seq()
{
    unsigned int size = 0;
    uint64_t total_size = 0;
    size = 256;
    binArray.flash_data.resize(size);
    uint16_t count = 0;
    while (size > 0)
    {
        if (binary_read(p_st,&binArray.flash_data,&size) != PARSER_ERR_OK)
        {
            printf("Cannot read the bin file\n");
            // return ERR_UNKNOWN;
        }
        count++;
        printf("Read data => %u - %u\n" , size,count);
        total_size += size;
        
    }
    


    printf("Read data => %u - E : %u\n",total_size,binary_size(p_st));
    return ERR_OK;
    
}
// ------------------------------------------------------------------------------
//  Start read thread
// ------------------------------------------------------------------------------
void BootloaderMavlink::start_read_thread(void)
{
    this->read_thread();
}

void BootloaderMavlink::read_thread(void)
{
    mavlink_message_t message     = { 0 };
    mavlink_status_t mav_status;
    if (_serial_port == nullptr) {
        printf("ERROR: serial port not exist\n");
        throw 1;
    }
    while (!time_to_exit) {
        if (_serial_port->status == SERIAL_PORT_OPEN)
        {
            char buf[256] = { 0 };
            int result = _serial_port->read_message(buf, 256);
                for (int i = 0; i < result; ++i) {
                    // the parsing
                    if (mavlink_parse_char(_channel, (uint8_t)buf[i], &message, &mav_status)) {
                        messages_handler(message);
                    }
                }
        }
    }
}
bool BootloaderMavlink::is_gimbal(uint8_t compid)
{

    switch (compid)
    {
    case MAV_COMP_ID_GIMBAL:
    case MAV_COMP_ID_GIMBAL2:
    case MAV_COMP_ID_GIMBAL3:
    case MAV_COMP_ID_GIMBAL4:
    case MAV_COMP_ID_GIMBAL5:
    case MAV_COMP_ID_GIMBAL6:
        return true;
    break;
    
    default:
        break;
    }
    return false;
}
void BootloaderMavlink::messages_handler(const mavlink_message_t &message)
{
    if (is_gimbal(message.compid))
    {
        switch (message.msgid)
        {
        case MAVLINK_MSG_ID_HEARTBEAT: 
        {

        }
        break;
        case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
        {

        }
        break;
        case MAVLINK_MSG_ID_SERIAL_CONTROL:
        {

        }
        break;
        default:
        break;
        }
    }
}
// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------
void *start_bootloader_read_thread(void *args)
{
    // takes an bootloader object argument
    BootloaderMavlink *boot_loader = (BootloaderMavlink *)args;
    // run the object's read thread
    boot_loader->start_read_thread();
    // done!
    return NULL;
}