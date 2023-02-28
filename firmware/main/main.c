#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "math.h"

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"

#include "msp.h"
#include "PMW3389DM/spi_pmw3389dm.h"

#include <std_msgs/msg/header.h>
#include <holohover_msgs/msg/holohover_imu.h>
#include <holohover_msgs/msg/holohover_mouse.h>
#include <holohover_msgs/msg/holohover_control.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif
#ifdef RMW_UXRCE_TRANSPORT_CUSTOM
//#include "nvs_flash.h"
//#include "bluetooth/esp32_bluetooth_serial_transport.h"
#include "esp32_serial_transport.h"
#elif defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
#include <uros_network_interfaces.h>
#endif

#define TAG "HOLOHOVER_ESP32_MAIN"

#define STRING_BUFFER_LEN 50

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECKRET(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);return temp_rc;}}

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

#define GPIO_NUM_LED GPIO_NUM_13

#define MOTOR_A_1  3
#define MOTOR_A_2  5
#define MOTOR_B_1  2
#define MOTOR_B_2  0
#define MOTOR_C_1  4
#define MOTOR_C_2  1
// Time for watchdog to turn off motors if no commands arrive: 100ms
#define MOTOR_WATCHDOG_TIMEOUT  100

#define UART_PORT          UART_NUM_2
#define UART_TXD           17
#define UART_RXD           16
#define UART_RTS           18
#define UART_CTS           19
#define UART_BAUD_RATE     1000000
#define UART_BUF_SIZE      1024
// Timeout of 100ms
#define UART_TIMEOUT  (100 / portTICK_RATE_MS)
// Sampling frequency of 100Hz
#define IMU_SAMPLE_TIME_MS 10



#define SPI_MISO 19
#define SPI_MOSI 18
#define SPI_CLK  5
#define SPI_CS   33
#define SPI_RS   27
// Sampling frequency of 100Hz
#define MOUSE_SAMPLE_TIME_MS 10

pmw3389dm_handle_t pmw3389dm_handle;

rcl_init_options_t init_options;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;

rcl_timer_t imu_timer;
rcl_timer_t mouse_timer;

rcl_publisher_t imu_publisher;
rcl_publisher_t mouse_publisher;
rcl_subscription_t control_subscriber;

rcl_publisher_t pong_publisher;
rcl_subscription_t ping_subscriber;

holohover_msgs__msg__HolohoverIMU outgoing_imu_measurement;
holohover_msgs__msg__HolohoverMouse outgoing_mouse_measurement;
holohover_msgs__msg__HolohoverControl incoming_control;
std_msgs__msg__Header incoming_ping;

struct msp_attitude_t attitude;
struct msp_raw_imu_t imu;
struct msp_motor_t motors;

uint8_t burst_buffer[12];

int64_t last_control_time;

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    if (timer != NULL) {

        int msp_attitude_result = msp_request(UART_PORT, MSP_ATTITUDE, &attitude, sizeof(attitude), UART_TIMEOUT);
        int msp_raw_imu_result = msp_request(UART_PORT, MSP_RAW_IMU, &imu, sizeof(imu), UART_TIMEOUT);

        if (msp_attitude_result >= 0 && msp_raw_imu_result >= 0) {
            outgoing_imu_measurement.atti.roll  = (double) attitude.pitch / 1800 * M_PI;
            outgoing_imu_measurement.atti.pitch = -((double) attitude.roll / 1800 * M_PI);
            outgoing_imu_measurement.atti.yaw   = -((double) attitude.yaw / 180 * M_PI);
            if (outgoing_imu_measurement.atti.yaw < -M_PI) {
                outgoing_imu_measurement.atti.yaw += 2 * M_PI;
            }

            outgoing_imu_measurement.acc.x = (double) imu.acc[1] / 32767 * 16 * 4 * 9.81;
            outgoing_imu_measurement.acc.y = -((double) imu.acc[0] / 32767 * 16 * 4 * 9.81);
            outgoing_imu_measurement.acc.z = -((double) imu.acc[2] / 32767 * 16 * 4 * 9.81);

            outgoing_imu_measurement.gyro.x = (double) imu.gyro[1] / 32767 * 2000 / 180 * M_PI;
            outgoing_imu_measurement.gyro.y = -((double) imu.gyro[0] / 32767 * 2000 / 180 * M_PI);
            outgoing_imu_measurement.gyro.z = (double) imu.gyro[2] / 32767 * 2000 / 180 * M_PI;

            RCSOFTCHECK(rcl_publish(&imu_publisher, (const void*) &outgoing_imu_measurement, NULL));
        } else {
            if (msp_attitude_result < 0) {
                ESP_LOGW(TAG, "MSP attitude request failed.");
            }
            if (msp_raw_imu_result < 0) {
                ESP_LOGW(TAG, "MSP raw imu request failed.");
            }
        }
    }
}

void mouse_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    if (timer != NULL) {

        esp_err_t err = spi_pmw3389dm_burst_read(pmw3389dm_handle, burst_buffer);
        if (err == ESP_OK) {
            bool motion = (burst_buffer[0] & 0x80) > 0;
            bool lifted = (burst_buffer[0] & 0x08) > 0;

            if (lifted) {
                ESP_LOGI(TAG, "Holohover ist lifted.");
                return;
            }

            if (motion) {
                // movement count since last call
                int16_t delta_x = (int16_t) ((burst_buffer[3] << 8) | burst_buffer[2]);
                int16_t delta_y = (int16_t) ((burst_buffer[5] << 8) | burst_buffer[4]);

                // last term convert inch to meters
                double dist_x = (double) delta_x / 16000 * 0.0254;
                double dist_y = (double) delta_y / 16000 * 0.0254;

                outgoing_mouse_measurement.v_x = dist_x / (double) last_call_time * 1000 * 1000 * 1000;
                outgoing_mouse_measurement.v_y = -dist_y / (double) last_call_time * 1000 * 1000 * 1000;
            } else {
                outgoing_mouse_measurement.v_x = 0;
                outgoing_mouse_measurement.v_y = 0;
            }

            RCSOFTCHECK(rcl_publish(&mouse_publisher, (const void*) &outgoing_mouse_measurement, NULL));
        } else {
            ESP_LOGW(TAG, "Burst read failed.");
        }
    }
}

void motor_control_subscription_callback(const void * msgin)
{
    const holohover_msgs__msg__HolohoverControl *msg = (const holohover_msgs__msg__HolohoverControl*) msgin;

    motors.motor[MOTOR_A_1] = 1000 + (int)(1000 * msg->motor_a_1);
    motors.motor[MOTOR_A_2] = 1000 + (int)(1000 * msg->motor_a_2);
    motors.motor[MOTOR_B_1] = 1000 + (int)(1000 * msg->motor_b_1);
    motors.motor[MOTOR_B_2] = 1000 + (int)(1000 * msg->motor_b_2);
    motors.motor[MOTOR_C_1] = 1000 + (int)(1000 * msg->motor_c_1);
    motors.motor[MOTOR_C_2] = 1000 + (int)(1000 * msg->motor_c_2);

    last_control_time = uxr_millis();
    if (msp_command(UART_PORT, MSP_SET_MOTOR, &motors, sizeof(motors), UART_TIMEOUT) < 0) {
        ESP_LOGW(TAG, "MSP set motor command failed.");
    }
}

void ping_subscription_callback(const void * msgin)
{
    const std_msgs__msg__Header *msg = (const std_msgs__msg__Header*) msgin;

    RCSOFTCHECK(rcl_publish(&pong_publisher, (const void*) msg, NULL));
}

void configure_led()
{
    gpio_reset_pin(GPIO_NUM_LED);
    // Set the GPIO as a push/pull output
    gpio_set_direction(GPIO_NUM_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_LED, 0);
}

void reset_motors()
{
    // 1000 = 0%, 2000 = 100%
    for (int i = 0; i < MSP_MAX_SUPPORTED_MOTORS; ++i) {
        motors.motor[i] = 1000;
    }
    ESP_LOGI(TAG, "Init motors to 0.");
    if (msp_command(UART_PORT, MSP_SET_MOTOR, &motors, sizeof(motors), UART_TIMEOUT) < 0) {
        ESP_LOGW(TAG, "Initial MSP set motor command failed.");
    }
}

rcl_ret_t create_entities()
{
    ESP_LOGI(TAG, "Setup ros node, publishers and subscribers.");

    // create init_options
    RCSOFTCHECKRET(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Create node
    RCSOFTCHECKRET(rclc_node_init_default(&node, "holohover_firmware", "", &support));

    // Create publishers and subscribers for measurements and control commands
    RCSOFTCHECKRET(rclc_publisher_init_best_effort(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(holohover_msgs, msg, HolohoverIMU), "/drone/imu"));
    RCSOFTCHECKRET(rclc_publisher_init_best_effort(&mouse_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(holohover_msgs, msg, HolohoverMouse), "/drone/mouse"));
    RCSOFTCHECKRET(rclc_subscription_init_best_effort(&control_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(holohover_msgs, msg, HolohoverControl), "/drone/control"));

    // Ping and pong publisher and subscriber
    RCSOFTCHECKRET(rclc_publisher_init_best_effort(&pong_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/drone/pong"));
    RCSOFTCHECKRET(rclc_subscription_init_best_effort(&ping_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/drone/ping"));

    // Create a timer to sample imu
    RCSOFTCHECKRET(rclc_timer_init_default(&imu_timer, &support, RCL_MS_TO_NS(IMU_SAMPLE_TIME_MS), imu_timer_callback));

    // Create a timer to sample mouse
    RCSOFTCHECKRET(rclc_timer_init_default(&mouse_timer, &support, RCL_MS_TO_NS(MOUSE_SAMPLE_TIME_MS), mouse_timer_callback));

    ESP_LOGI(TAG, "Starting ros executor.");

    // Create executor
    RCSOFTCHECKRET(rclc_executor_init(&executor, &support.context, 5, &allocator));
    RCSOFTCHECKRET(rclc_executor_add_timer(&executor, &imu_timer));
    RCSOFTCHECKRET(rclc_executor_add_timer(&executor, &mouse_timer));
    RCSOFTCHECKRET(rclc_executor_add_subscription(&executor, &control_subscriber, &incoming_control, &motor_control_subscription_callback, ON_NEW_DATA));
    RCSOFTCHECKRET(rclc_executor_add_subscription(&executor, &ping_subscriber, &incoming_ping, &ping_subscription_callback, ON_NEW_DATA));

    return RCL_RET_OK;
}

void destroy_entities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_subscription_fini(&control_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&ping_subscriber, &node));

    RCSOFTCHECK(rcl_timer_fini(&imu_timer));
    RCSOFTCHECK(rcl_timer_fini(&mouse_timer));

    RCSOFTCHECK(rcl_publisher_fini(&imu_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&mouse_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&pong_publisher, &node));

    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
}

void micro_ros_task(void * arg)
{
    // set log level
#ifdef RMW_UXRCE_TRANSPORT_CUSTOM
    // Don't log with custom transport to not disturb serial communication
    esp_log_level_set(TAG, ESP_LOG_NONE);
#else
    esp_log_level_set(TAG, ESP_LOG_INFO);
#endif

    // Wait 2 sec for flight controller
    vTaskDelay((2000 / portTICK_RATE_MS));

    configure_led();

    ESP_LOGI(TAG, "Configure UART.");
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    // Install UART driver (we don't need an event queue here)
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 0, NULL, 0));
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_LOGI(TAG, "UART set pins, mode and install driver.");
    // Set UART pins
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TXD, UART_RXD, UART_RTS, UART_CTS));

    ESP_LOGI(TAG, "Configure SPI.");
    spi_bus_config_t buscfg = {
            .miso_io_num = SPI_MISO,
            .mosi_io_num = SPI_MOSI,
            .sclk_io_num = SPI_CLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1
    };
    // Initialize SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    pmw3389dm_config_t pmw3389dm_config = {
            .host = HSPI_HOST,
            .cs_io = SPI_CS,
            .rs_io = SPI_RS
    };
    ESP_LOGI(TAG, "Initialize PMW3389DM...");
    ESP_ERROR_CHECK(spi_pmw3389dm_init(&pmw3389dm_config, &pmw3389dm_handle));
    // power up and upload firmware
    ESP_ERROR_CHECK(spi_pmw3389dm_power_up_and_upload_firmware(pmw3389dm_handle));
    // set CPI to maximum (16000)
    ESP_ERROR_CHECK(spi_pmw3389dm_set_cpi(pmw3389dm_handle, 16000));
    // set lift cut off to 3mm
    ESP_ERROR_CHECK(spi_pmw3389dm_write(pmw3389dm_handle, 0x63, 0x03));
    // since we only burst read we can initialise it here
    ESP_ERROR_CHECK(spi_pmw3389dm_init_burst_read(pmw3389dm_handle));

    reset_motors();

    char incoming_ping_buffer[STRING_BUFFER_LEN];
    incoming_ping.frame_id.data = incoming_ping_buffer;
    incoming_ping.frame_id.capacity = STRING_BUFFER_LEN;

    allocator = rcl_get_default_allocator();

    init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    if (strcmp(CONFIG_MICRO_ROS_AGENT_IP, "0.0.0.0") == 0) {
        // autodiscover
        RCCHECK(rmw_uros_discover_agent(rmw_options));
    } else {
        // use static agent ip and port
        RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    }
#endif

    while(1) {
        switch (state) {
            case WAITING_AGENT:
                EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent_options(100, 1, rmw_options)) ? AGENT_AVAILABLE : WAITING_AGENT;);
                break;
            case AGENT_AVAILABLE:
                state = (RCL_RET_OK == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;

                if (state == AGENT_CONNECTED) {
                    state = (RMW_RET_OK == rmw_uros_sync_session(500)) ? AGENT_CONNECTED : WAITING_AGENT;
                }

                if (state == WAITING_AGENT) {
                    destroy_entities();
                }
                break;
            case AGENT_CONNECTED:
                ESP_LOGI(TAG, "Agent is connected, ros is now spinning.");

                if (uxr_millis() - last_control_time > MOTOR_WATCHDOG_TIMEOUT) {
                    reset_motors();
                    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 3)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
                }

                if (state == AGENT_CONNECTED) {
                    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
                }
                break;
            case AGENT_DISCONNECTED:
                reset_motors();
                destroy_entities();
                state = WAITING_AGENT;
                break;
            default:
                break;
        }

        if (state == AGENT_CONNECTED) {
            gpio_set_level(GPIO_NUM_LED, 1);
        } else {
            gpio_set_level(GPIO_NUM_LED, 0);
        }
    }
}

static size_t uart_port = UART_NUM_0;

void app_main(void)
{
#ifdef RMW_UXRCE_TRANSPORT_CUSTOM
//    esp_err_t ret = nvs_flash_init();
//    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//        ESP_ERROR_CHECK(nvs_flash_erase());
//        ret = nvs_flash_init();
//    }
//    ESP_ERROR_CHECK(ret);
//
//    ESP_ERROR_CHECK(rmw_uros_set_custom_transport(
//        true,
//        (void *) "ESP32",
//        esp32_bluetooth_serial_open,
//        esp32_bluetooth_serial_close,
//        esp32_bluetooth_serial_write,
//        esp32_bluetooth_serial_read
//    ));
    ESP_ERROR_CHECK(rmw_uros_set_custom_transport(
		true,
		(void *) &uart_port,
		esp32_serial_open,
		esp32_serial_close,
		esp32_serial_write,
		esp32_serial_read
	));
#elif defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#else
#error micro-ROS transports misconfigured
#endif

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}
