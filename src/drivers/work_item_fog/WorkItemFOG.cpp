// WorkItemFOG.cpp

#include "WorkItemFOG.hpp"

WorkItemFOG::WorkItemFOG() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

WorkItemFOG::~WorkItemFOG()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
	if (uart_fd >= 0) {
		::close(uart_fd);
    	}
}

void WorkItemFOG::init()
{
    // Open UART
    PX4_INFO("Initializing WorkItemFOG");
    uart_fd = ::open("/dev/ttyS1", O_RDWR | O_NOCTTY);
    if (uart_fd < 0) {
        PX4_ERR("Failed to open UART device: %d (errno: %d)", uart_fd, errno);
        return;
    }
    PX4_INFO("UART opened successfully with fd: %d", uart_fd);

    // Configure UART
    struct termios uart_config;
    int termios_state;
    if ((termios_state = tcgetattr(uart_fd, &uart_config)) < 0) {
        PX4_ERR("Error getting UART config: %d", termios_state);
        return;
    }
    uart_config.c_oflag &= ~ONLCR;
    if ((termios_state = cfsetispeed(&uart_config, B921600)) < 0 ||
        (termios_state = cfsetospeed(&uart_config, B921600)) < 0 ||
        (termios_state = tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0) {
        PX4_ERR("Error setting UART config: %d", termios_state);
        return;
    }
    if ((termios_state = cfsetospeed(&uart_config, B921600)) < 0) {
        PX4_ERR("Error setting UART output speed: %d", termios_state);
        return;
    }
    if ((termios_state = tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0) {
        PX4_ERR("Error setting UART config: %d", termios_state);
        return;
    }
    ScheduleOnInterval(1250); // 800 Hz rate
    PX4_INFO("WorkItemFOG initialized");
}

void WorkItemFOG::Run()
{
    perf_begin(_loop_perf);
	uint8_t buffer[512]; // Buffer to hold start byte and 9 data bytes
	FOG_DATA_Type data;

	if (fcntl(uart_fd, F_GETFD) == -1) {
        if (uart_fd >= 0) {
            ::close(uart_fd);
        }
        uart_fd = ::open("/dev/ttyS1", O_RDWR | O_NOCTTY);
        if (uart_fd < 0) {
            PX4_ERR("Failed to reopen UART device: %d (errno: %d)", uart_fd, errno);
            return;
        }
    	}

	int bytes_read = read(uart_fd, buffer, sizeof(buffer));
	hrt_abstime timestamp_sample = hrt_absolute_time(); // Capture timestamp after reading

	int index = -1;
	for(int i=0; i < bytes_read - 9; i++) {
		if(buffer[i] == 0x80) {
		index = i;
		break;
		}
	}

	if(index == -1) {
		return;
	}

	if (buffer[index] == 0x80) {
		// Start byte is correct, process the data
		data.x = ((uint32_t)buffer[index+1] << 16) | ((uint32_t)buffer[index+2] << 8) | ((uint32_t)buffer[index+3]);
		data.y = ((int32_t)buffer[index+4] << 16) | ((int32_t)buffer[index+5] << 8) | buffer[index+6];
		data.z = ((int32_t)buffer[index+7] << 16) | ((int32_t)buffer[index+8] << 8) | buffer[index+9];

		// Sign extension
		if (data.x & 0x800000) data.x |= 0xFF000000;
		if (data.y & 0x800000) data.y |= 0xFF000000;
		if (data.z & 0x800000) data.z |= 0xFF000000;

		sensor_gyro_s sensor_gyro{};
		sensor_gyro.timestamp_sample = timestamp_sample;
		sensor_gyro.timestamp = hrt_absolute_time();
		sensor_gyro.device_id = DEVICE_ID;
		sensor_gyro.x = ((float)data.z / 3600.0f) * DEG_TO_RAD;
		sensor_gyro.y = -((float)data.y / 3600.0f) * DEG_TO_RAD;
		sensor_gyro.z = ((float)data.x / 3600.0f) * DEG_TO_RAD;
		sensor_gyro.temperature = 30.0f;
		sensor_gyro.samples = 10;

		_gyro_pub.publish(sensor_gyro);
	}
    perf_end(_loop_perf);
    perf_count(_loop_interval_perf);
}

int WorkItemFOG::task_spawn(int argc, char *argv[])
{
    WorkItemFOG *instance = new WorkItemFOG();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;
	instance->init();
        return PX4_OK;

    } else {
        PX4_ERR("alloc failed");
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}

int WorkItemFOG::print_status()
{
    perf_print_counter(_loop_perf);
    perf_print_counter(_loop_interval_perf);
    return 0;
}

int WorkItemFOG::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int WorkItemFOG::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
FOG (Fiber Optic Gyroscope) driver running as a work queue task.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("work_item_fog", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int work_item_fog_main(int argc, char *argv[])
{
    return WorkItemFOG::main(argc, argv);
}
