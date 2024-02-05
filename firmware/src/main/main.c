#include <string.h>
#include <stdio.h>
#include <math.h>

#include "neopixel.h"
#include "nvs-manager.h"
#include "wifi.h"
#include "config-server.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "msp.h"
#include "PMW3389DM/spi_pmw3389dm.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/header.h>
#include <holohover_msgs/msg/holohover_imu_stamped.h>
#include <holohover_msgs/msg/holohover_mouse_stamped.h>
#include <holohover_msgs/msg/holohover_control_stamped.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

static const char *TAG = "MAIN";

#define MSG_BUFFER_LEN 100

#define CHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != 1)){printf("Failed on line %d. Aborting.\n",__LINE__);vTaskDelete(NULL);}}
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECKRET(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);return temp_rc;}}

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

#define GPIO_NUM_LED GPIO_NUM_13

#define GPIO_NUM_NEOPIXEL       GPIO_NUM_0
#define GPIO_NUM_NEOPIXEL_POWER GPIO_NUM_2
#define NEOPIXEL_NR_LED         1
#define	NEOPIXEL_RMT_CHANNEL    RMT_CHANNEL_1

#define MOTOR_A_1  0
#define MOTOR_A_2  1
#define MOTOR_B_1  4
#define MOTOR_B_2  3
#define MOTOR_C_1  2
#define MOTOR_C_2  5

// Time for watchdog to turn off motors if no commands arrive: 100ms
#define MOTOR_WATCHDOG_TIMEOUT  100

#define UART_PORT          UART_NUM_2
#define UART_TXD           8
#define UART_RXD           7
#define UART_RTS           -1
#define UART_CTS           -1
#define UART_BAUD_RATE     1000000
#define UART_BUF_SIZE      1024
// Timeout of 100ms
#define UART_TIMEOUT  (100 / portTICK_RATE_MS)
// Sampling frequency of 100Hz
#define IMU_SAMPLE_TIME_MS 10

#define SPI_MISO 21
#define SPI_MOSI 19
#define SPI_CLK  5
#define SPI_CS   33
#define SPI_RS   27
// Sampling frequency of 100Hz
#define MOUSE_SAMPLE_TIME_MS 10

pixel_settings_t px;
uint32_t pixels[NEOPIXEL_NR_LED];

pmw3389dm_handle_t pmw3389dm_handle;
uint8_t pmw3389dm_burst_buffer[12];

struct msp_raw_imu_t imu;
struct msp_motor_t motors;

rcl_init_options_t init_options;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;

rcl_timer_t imu_timer;
rcl_publisher_t imu_publisher;
holohover_msgs__msg__HolohoverIMUStamped outgoing_imu_measurement_msg;
uint8_t outgoing_imu_measurement_msg_buffer[MSG_BUFFER_LEN];

rcl_timer_t mouse_timer;
rcl_publisher_t mouse_publisher;
holohover_msgs__msg__HolohoverMouseStamped outgoing_mouse_measurement_msg;
uint8_t outgoing_mouse_measurement_msg_buffer[MSG_BUFFER_LEN];

rcl_subscription_t control_subscriber;
holohover_msgs__msg__HolohoverControlStamped incoming_control_msg;
uint8_t incoming_control_msg_buffer[MSG_BUFFER_LEN];
int64_t last_control_time;

rcl_publisher_t pong_publisher;
rcl_subscription_t ping_subscriber;
std_msgs__msg__Header incoming_ping;
uint8_t incoming_ping_buffer[MSG_BUFFER_LEN];

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED_IDLE,
  AGENT_CONNECTED_ACTIVE,
  AGENT_DISCONNECTED
} state;

void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    if (timer != NULL) {

        int msp_raw_imu_result = msp_request(UART_PORT, MSP_RAW_IMU, &imu, sizeof(imu), UART_TIMEOUT);

        if (msp_raw_imu_result >= 0) {
            int64_t nanos = rmw_uros_epoch_nanos();
            outgoing_imu_measurement_msg.header.stamp.sec = nanos / 1000000000;
            outgoing_imu_measurement_msg.header.stamp.nanosec = nanos % 1000000000;

            outgoing_imu_measurement_msg.acc.x = (double) imu.acc[1] / 32767 * 16 * 4 * 9.81;
            outgoing_imu_measurement_msg.acc.y = -((double) imu.acc[0] / 32767 * 16 * 4 * 9.81);
            outgoing_imu_measurement_msg.acc.z = -((double) imu.acc[2] / 32767 * 16 * 4 * 9.81);

            outgoing_imu_measurement_msg.gyro.x = (double) imu.gyro[1] / 32767 * 2000 / 180 * M_PI;
            outgoing_imu_measurement_msg.gyro.y = -((double) imu.gyro[0] / 32767 * 2000 / 180 * M_PI);
            outgoing_imu_measurement_msg.gyro.z = (double) imu.gyro[2] / 32767 * 2000 / 180 * M_PI;

            RCSOFTCHECK(rcl_publish(&imu_publisher, (const void*) &outgoing_imu_measurement_msg, NULL));
        } else {
            if (msp_raw_imu_result < 0) {
                ESP_LOGW(TAG, "MSP raw imu request failed.");
            }
        }
    }
}

void mouse_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    if (timer != NULL) {

        int64_t start = esp_timer_get_time();
        esp_err_t err = spi_pmw3389dm_burst_read(pmw3389dm_handle, pmw3389dm_burst_buffer);
        int64_t end = esp_timer_get_time();
        int64_t diff = end - start;
        ESP_LOGI(TAG, "time mouse: %lld", diff);
        if (err == ESP_OK) {
            int64_t nanos = rmw_uros_epoch_nanos();
            outgoing_mouse_measurement_msg.header.stamp.sec = nanos / 1000000000;
            outgoing_mouse_measurement_msg.header.stamp.nanosec = nanos % 1000000000;

            bool motion = (pmw3389dm_burst_buffer[0] & 0x80) > 0;
            bool lifted = (pmw3389dm_burst_buffer[0] & 0x08) > 0;

            if (lifted) {
                ESP_LOGI(TAG, "Holohover ist lifted.");
                return;
            }

            if (motion) {
                // movement count since last call
                int16_t delta_x = (int16_t) ((pmw3389dm_burst_buffer[3] << 8) | pmw3389dm_burst_buffer[2]);
                int16_t delta_y = (int16_t) ((pmw3389dm_burst_buffer[5] << 8) | pmw3389dm_burst_buffer[4]);

                // last term convert inch to meters
                double dist_x = (double) delta_x / 16000 * 0.0254;
                double dist_y = (double) delta_y / 16000 * 0.0254;

                outgoing_mouse_measurement_msg.v_x = dist_x / (double) last_call_time * 1000 * 1000 * 1000;
                outgoing_mouse_measurement_msg.v_y = -dist_y / (double) last_call_time * 1000 * 1000 * 1000;
            } else {
                outgoing_mouse_measurement_msg.v_x = 0;
                outgoing_mouse_measurement_msg.v_y = 0;
            }

            RCSOFTCHECK(rcl_publish(&mouse_publisher, (const void*) &outgoing_mouse_measurement_msg, NULL));
        } else {
            ESP_LOGW(TAG, "Burst read failed.");
        }
    }
}

void motor_control_subscription_callback(const void * msgin)
{
    const holohover_msgs__msg__HolohoverControlStamped *msg = (const holohover_msgs__msg__HolohoverControlStamped*) msgin;

    motors.motor[MOTOR_A_1] = 1000 + (int)(1000 * msg->motor_a_1);
    motors.motor[MOTOR_A_2] = 1000 + (int)(1000 * msg->motor_a_2);
    motors.motor[MOTOR_B_1] = 1000 + (int)(1000 * msg->motor_b_1);
    motors.motor[MOTOR_B_2] = 1000 + (int)(1000 * msg->motor_b_2);
    motors.motor[MOTOR_C_1] = 1000 + (int)(1000 * msg->motor_c_1);
    motors.motor[MOTOR_C_2] = 1000 + (int)(1000 * msg->motor_c_2);

    if (msp_command(UART_PORT, MSP_SET_MOTOR, &motors, sizeof(motors), UART_TIMEOUT) < 0) {
        ESP_LOGW(TAG, "MSP set motor command failed.");
    }

    if (state != AGENT_CONNECTED_ACTIVE) {
        state = AGENT_CONNECTED_ACTIVE;

        np_set_pixel_rgbw(&px, 0, 0x00, 0x00, 0xFF, 0x00); // blue
        np_show(&px, NEOPIXEL_RMT_CHANNEL);
    }
    last_control_time = uxr_millis();
}

void ping_subscription_callback(const void * msgin)
{
    int64_t nanos = rmw_uros_epoch_nanos();
    incoming_ping.stamp.sec = nanos / 1000000000;
    incoming_ping.stamp.nanosec = nanos % 1000000000;

    RCSOFTCHECK(rcl_publish(&pong_publisher, (const void*) &incoming_ping, NULL));
}

void configure_led()
{
    gpio_reset_pin(GPIO_NUM_LED);
    // Set the GPIO as a push/pull output
    gpio_set_direction(GPIO_NUM_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_LED, 1);
}

void configure_neopixel()
{
    gpio_reset_pin(GPIO_NUM_NEOPIXEL_POWER);
    // Set the GPIO as a push/pull output
    gpio_set_direction(GPIO_NUM_NEOPIXEL_POWER, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_NEOPIXEL_POWER, 1);

    ESP_ERROR_CHECK(neopixel_init(GPIO_NUM_NEOPIXEL, NEOPIXEL_RMT_CHANNEL));

    for	(int i = 0; i < NEOPIXEL_NR_LED; i++) {
        pixels[i] = 0;
    }
    px.pixels = (uint8_t*) pixels;
    px.pixel_count = NEOPIXEL_NR_LED;
    strcpy(px.color_order, "GRB");

    memset(&px.timings, 0, sizeof(px.timings));
    px.timings.mark.level0 = 1;
    px.timings.space.level0 = 1;
    px.timings.mark.duration0 = 12;
    px.nbits = 24;
    px.timings.mark.duration1 = 14;
    px.timings.space.duration0 = 7;
    px.timings.space.duration1 = 16;
    px.timings.reset.duration0 = 600;
    px.timings.reset.duration1 = 600;

    px.brightness = 0x80;
    np_set_pixel_rgbw(&px, 0, 0xFF, 0x00, 0x00, 0x00); // red
    np_show(&px, NEOPIXEL_RMT_CHANNEL);
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

    // create node
    RCSOFTCHECKRET(rclc_node_init_default(&node, "holohover_firmware", "", &support));

    // Create publishers and subscribers for measurements and control commands
    RCSOFTCHECKRET(rclc_publisher_init_best_effort(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(holohover_msgs, msg, HolohoverIMUStamped), "/drone/imu"));
    RCSOFTCHECKRET(rclc_publisher_init_best_effort(&mouse_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(holohover_msgs, msg, HolohoverMouseStamped), "/drone/mouse"));
    RCSOFTCHECKRET(rclc_subscription_init_best_effort(&control_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(holohover_msgs, msg, HolohoverControlStamped), "/drone/control"));

    // Ping and pong publisher and subscriber
    RCSOFTCHECKRET(rclc_publisher_init_best_effort(&pong_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/drone/pong"));
    RCSOFTCHECKRET(rclc_subscription_init_best_effort(&ping_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/drone/ping"));

    // Create a timer to sample imu
    RCSOFTCHECKRET(rclc_timer_init_default(&imu_timer, &support, RCL_MS_TO_NS(IMU_SAMPLE_TIME_MS), imu_timer_callback));

    // Create a timer to sample mouse
    RCSOFTCHECKRET(rclc_timer_init_default(&mouse_timer, &support, RCL_MS_TO_NS(MOUSE_SAMPLE_TIME_MS), mouse_timer_callback));

    ESP_LOGI(TAG, "Starting ros executor.");

    // create executor
    RCSOFTCHECKRET(rclc_executor_init(&executor, &support.context, 4, &allocator));
    RCSOFTCHECKRET(rclc_executor_add_subscription(&executor, &ping_subscriber, &incoming_ping, &ping_subscription_callback, ON_NEW_DATA));
    RCSOFTCHECKRET(rclc_executor_add_subscription(&executor, &control_subscriber, &incoming_control_msg, &motor_control_subscription_callback, ON_NEW_DATA));
    RCSOFTCHECKRET(rclc_executor_add_timer(&executor, &imu_timer));
    RCSOFTCHECKRET(rclc_executor_add_timer(&executor, &mouse_timer));

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
    static micro_ros_utilities_memory_conf_t conf = {0};
    conf.max_string_capacity = 50;

    CHECK(micro_ros_utilities_create_static_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(holohover_msgs, msg, HolohoverIMUStamped),
        &outgoing_imu_measurement_msg,
        conf,
        outgoing_imu_measurement_msg_buffer,
        sizeof(outgoing_imu_measurement_msg_buffer)
    ));
    outgoing_imu_measurement_msg.header.frame_id = micro_ros_string_utilities_set(outgoing_imu_measurement_msg.header.frame_id, "body");

    CHECK(micro_ros_utilities_create_static_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(holohover_msgs, msg, HolohoverMouseStamped),
        &outgoing_mouse_measurement_msg,
        conf,
        outgoing_mouse_measurement_msg_buffer,
        sizeof(outgoing_mouse_measurement_msg_buffer)
    ));
    outgoing_mouse_measurement_msg.header.frame_id = micro_ros_string_utilities_set(outgoing_mouse_measurement_msg.header.frame_id, "body");

    CHECK(micro_ros_utilities_create_static_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(holohover_msgs, msg, HolohoverControlStamped),
        &incoming_control_msg,
        conf,
        incoming_control_msg_buffer,
        sizeof(incoming_control_msg_buffer)
    ));

    CHECK(micro_ros_utilities_create_static_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
        &incoming_ping,
        conf,
        incoming_ping_buffer,
        sizeof(incoming_ping_buffer)
    ));

    // Wait 2 sec for flight controller
    vTaskDelay(2000 / portTICK_RATE_MS);

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

    state = WAITING_AGENT;

    allocator = rcl_get_default_allocator();
    node = rcl_get_zero_initialized_node();

    init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    char agent_port[10];
    sprintf(agent_port, "%d", global_config.agent_port);
    RCCHECK(rmw_uros_options_set_udp_address(global_config.agent_ip, agent_port, rmw_options));

    while(1) {
        switch (state) {
            case WAITING_AGENT:
                EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent_options(100, 1, rmw_options)) ? AGENT_AVAILABLE : WAITING_AGENT;);
                break;
            case AGENT_AVAILABLE:
                state = (RCL_RET_OK == create_entities()) ? AGENT_CONNECTED_IDLE : WAITING_AGENT;

                if (state == AGENT_CONNECTED_IDLE) {
                    state = (RMW_RET_OK == rmw_uros_sync_session(500)) ? AGENT_CONNECTED_IDLE : WAITING_AGENT;
                }

                if (state == AGENT_CONNECTED_IDLE) {
                    np_set_pixel_rgbw(&px, 0, 0x00, 0xFF, 0x00, 0x00); // green
                    np_show(&px, NEOPIXEL_RMT_CHANNEL);
                }

                if (state == WAITING_AGENT) {
                    destroy_entities();
                }
                break;
            case AGENT_CONNECTED_ACTIVE:
                if (uxr_millis() - last_control_time > MOTOR_WATCHDOG_TIMEOUT) {
                    state = AGENT_CONNECTED_IDLE;

                    np_set_pixel_rgbw(&px, 0, 0x00, 0xFF, 0x00, 0x00); // green
                    np_show(&px, NEOPIXEL_RMT_CHANNEL);
                }
                break;
            case AGENT_CONNECTED_IDLE:
                EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 3)) ? AGENT_CONNECTED_IDLE : AGENT_DISCONNECTED;);
                break;
            case AGENT_DISCONNECTED:
                np_set_pixel_rgbw(&px, 0, 0xFF, 0x00, 0x00, 0x00); // red
                np_show(&px, NEOPIXEL_RMT_CHANNEL);

                destroy_entities();

                state = WAITING_AGENT;
                break;
            default:
                break;
        }

        if (state == AGENT_CONNECTED_IDLE || state == AGENT_CONNECTED_ACTIVE) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }

        if (state != AGENT_CONNECTED_ACTIVE) {
            // stop motors
            reset_motors();
        }
    }

    vTaskDelete(NULL);
}

void app_main(void)
{
    configure_led();
    configure_neopixel();
    nvs_init();
    nvs_load_config();

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(network_interface_initialize());
    // disable wifi power safe mode
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
#endif

    config_server_start();

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}
