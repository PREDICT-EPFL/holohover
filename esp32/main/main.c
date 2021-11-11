#include <string.h>
#include <stdio.h>
#include <unistd.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"

#include "msp.h"

#include <uros_network_interfaces.h>
#include <holohover_msgs/msg/drone_measurement.h>
#include <holohover_msgs/msg/motor_control.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define TAG "HOLOHOVER_ESP32"

#define SAMPLE_TIME_MS 10

#define MOTOR_A_1  0
#define MOTOR_A_2  1
#define MOTOR_B_1  2
#define MOTOR_B_2  3
#define MOTOR_C_1  4
#define MOTOR_C_2  5

#define UART_PORT          UART_NUM_2
#define UART_TXD           17
#define UART_RXD           16
#define UART_RTS           18
#define UART_CTS           19
#define UART_BAUD_RATE     115200
#define UART_BUF_SIZE      1024
// Timeout of 100ms
#define UART_TIMEOUT  (100 / portTICK_RATE_MS)

rcl_publisher_t drone_measurement_publisher;
rcl_subscription_t motor_control_subscriber;

holohover_msgs__msg__DroneMeasurement outgoing_measurement;
holohover_msgs__msg__MotorControl incoming_motor_control;

struct msp_attitude_t attitude;
struct msp_raw_imu_t imu;
struct msp_motor_t motors;

void measurement_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);

	if (timer != NULL) {

        if (msp_request(UART_PORT, MSP_ATTITUDE, &attitude, sizeof(attitude), UART_TIMEOUT) >= 0
         && msp_request(UART_PORT, MSP_RAW_IMU, &imu, sizeof(imu), UART_TIMEOUT) >= 0) {

            outgoing_measurement.atti.roll  = (float) attitude.roll;
            outgoing_measurement.atti.pitch = (float) attitude.pitch;
            outgoing_measurement.atti.yaw   = (float) attitude.yaw;

            outgoing_measurement.acc.x = imu.acc[0];
            outgoing_measurement.acc.y = imu.acc[1];
            outgoing_measurement.acc.z = imu.acc[2];

            outgoing_measurement.gyro.x = imu.gyro[0];
            outgoing_measurement.gyro.y = imu.gyro[1];
            outgoing_measurement.gyro.z = imu.gyro[2];

            outgoing_measurement.mag.x = imu.mag[0];
            outgoing_measurement.mag.y = imu.mag[1];
            outgoing_measurement.mag.z = imu.mag[2];

            RCSOFTCHECK(rcl_publish(&drone_measurement_publisher, (const void*) &outgoing_measurement, NULL));
        }
	}
}

void motor_control_subscription_callback(const void * msgin)
{
    const holohover_msgs__msg__MotorControl *msg = (const holohover_msgs__msg__MotorControl*) msgin;

    motors.motor[MOTOR_A_1] = 1000 + (int)(1000 * msg->motor_a_1);
    motors.motor[MOTOR_A_2] = 1000 + (int)(1000 * msg->motor_a_2);
    motors.motor[MOTOR_B_1] = 1000 + (int)(1000 * msg->motor_b_1);
    motors.motor[MOTOR_B_2] = 1000 + (int)(1000 * msg->motor_b_2);
    motors.motor[MOTOR_C_1] = 1000 + (int)(1000 * msg->motor_c_1);
    motors.motor[MOTOR_C_2] = 1000 + (int)(1000 * msg->motor_c_2);

    msp_command(UART_PORT, MSP_SET_MOTOR, &motors, sizeof(motors), UART_TIMEOUT);
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "Start RS485 application test and configure UART.");

    // Install UART driver (we don't need an event queue here)
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 0, NULL, 0));

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));

    ESP_LOGI(TAG, "UART set pins, mode and install driver.");

    // Set UART pins
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TXD, UART_RXD, UART_RTS, UART_CTS));

    // init motors
    // 1000 = 0%, 2000 = 100%
    for (int i = 0; i < MSP_MAX_SUPPORTED_MOTORS; ++i) {
        motors.motor[i] = 1000;
    }
    msp_command(UART_PORT, MSP_SET_MOTOR, &motors, sizeof(motors), UART_TIMEOUT);

#if 0
	// create init_options
	//RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
#endif

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodiscovery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

	// Create a best effort pong publisher
	RCCHECK(rclc_publisher_init_best_effort(&drone_measurement_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(holohover_msgs, msg, DroneMeasurement), "/drone/measurement"));

	// Create a best effort ping subscriber
	RCCHECK(rclc_subscription_init_best_effort(&motor_control_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(holohover_msgs, msg, MotorControl), "/drone/motor_control"));

	// Create a timer to sample measurements
	rcl_timer_t timer;
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(SAMPLE_TIME_MS), measurement_timer_callback));

	// Create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &motor_control_subscriber, &incoming_motor_control,
		&motor_control_subscription_callback, ON_NEW_DATA));

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		usleep(1000);
	}

	// Free resources
	RCCHECK(rcl_publisher_fini(&drone_measurement_publisher, &node));
	RCCHECK(rcl_subscription_fini(&motor_control_subscriber, &node));
	RCCHECK(rcl_node_fini(&node));
}


void app_main(void)
{
#ifdef CONFIG_MICRO_ROS_ESP_NETIF_WLAN || CONFIG_MICRO_ROS_ESP_NETIF_ENET
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}
