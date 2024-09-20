#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "audio_pipeline.h"
#include "audio_element.h"
#include "audio_mem.h"
#include "audio_sys.h"
#include "audio_common.h"
#include "audio_event_iface.h"
#include "i2s_stream.h"
#include "esp_peripherals.h"
#include "board.h"
#include "driver/i2s_std.h"
#include "amr_decoder.h"
#include "amrwb_encoder.h"
#include "periph_button.h"
#include "input_key_service.h"
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <std_msgs/msg/byte_multi_array.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// I2S配置
#define PLAY_RATE         16000
#define PLAY_CHANNEL      1
#define PLAY_BITS         16

#define AMRWB_HEADER_INFO        "#!AMR-WB\n"  
#define AMRWB_HEADER_SIZE        (9)
#define RECV_RB_SIZE             1024 * 5
static const char *TAG = "Audio_com";

// Define publisher & subscriber
static rcl_subscription_t subscriber;
static rcl_publisher_t publisher;
// Define send & recv message
static std_msgs__msg__ByteMultiArray recv_msg;
static std_msgs__msg__ByteMultiArray send_msg;
audio_element_handle_t i2s_stream_reader, amrnb_encoder_el, amrwb_encoder_el;
audio_pipeline_handle_t pipeline_in = NULL;
audio_pipeline_handle_t pipeline_out = NULL;
rclc_executor_t executor;
static bool send_flag = false;
static int res_header = 0;
static int receive_ID = 0;
static ringbuf_handle_t *recv_rb;
static char *rbuf = NULL;
// #define SEND_LEN    61    // AMRWB_ENC_BITRATE_MD2385 61
// #define SEND_LEN    16    // AMRWB_ENC_BITRATE_MD66   16
#define SEND_LEN    48       // AMRWB_ENC_BITRATE_MD885  24
#define RECV_LEN    48       // recv msg length need match with target msg length

static audio_element_handle_t create_amr_decoder()
{
    amr_decoder_cfg_t amr_cfg = DEFAULT_AMR_DECODER_CONFIG();
    // amr_cfg.task_prio = 12;
    return amr_decoder_init(&amr_cfg);
}

static audio_element_handle_t create_amrwb_encoder()
{
    amrwb_encoder_cfg_t amrwb_cfg = DEFAULT_AMRWB_ENCODER_CONFIG();
    amrwb_cfg.task_core = 1;
    amrwb_cfg.bitrate_mode = AMRWB_ENC_BITRATE_MD885; 
    return amrwb_encoder_init(&amrwb_cfg);
}

static audio_element_handle_t create_i2s_stream(int sample_rates, int bits, int channels, audio_stream_type_t type)
{
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT_WITH_PARA(CODEC_ADC_I2S_PORT, sample_rates, bits, type);
    i2s_cfg.std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_MONO;
    i2s_cfg.std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    i2s_cfg.task_core = 1;
    audio_element_handle_t i2s_stream = i2s_stream_init(&i2s_cfg);
    mem_assert(i2s_stream);
    audio_element_set_music_info(i2s_stream, sample_rates, channels, bits);
    return i2s_stream;
}

//print heap information
void print_heap_info() {
    ESP_LOGI(TAG, "CPU cli");
    audio_mem_print(TAG, __LINE__, __func__);
    audio_sys_get_real_time_stats();
    ESP_LOGI(TAG, "Internal free heap size: %ld bytes", esp_get_free_internal_heap_size());
    ESP_LOGI(TAG, "PSRAM    free heap size: %ld bytes", esp_get_free_heap_size() - esp_get_free_internal_heap_size());
    ESP_LOGI(TAG, "Total    free heap size: %ld bytes", esp_get_free_heap_size());
}

//timer callback 
void microros_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
        // print_heap_info();
        // 自定义内容
	}
}

// receive audio  
void subscription_callback(const void * msgin)
{
	const std_msgs__msg__ByteMultiArray * msg = (const std_msgs__msg__ByteMultiArray *)msgin;
    recv_msg.data.size = msg->data.size;
    memcpy(recv_msg.data.data, msg->data.data, recv_msg.data.size);
    if(msg == NULL){
		printf("Callback : msg NULL \r\n");
	}
    if (rb_bytes_available(recv_rb) > recv_msg.data.size) {
        // ESP_LOGI(TAG, "recv_rb available: %d, recv_rb size: %d \r\n", 
                // rb_bytes_available(recv_rb), RECV_RB_SIZE);
        if (rb_write(recv_rb, (void *)recv_msg.data.data, recv_msg.data.size, portMAX_DELAY) <= 0) {
            ESP_LOGW(TAG, "recv_rb write timeout");
        }
    } else {
        ESP_LOGI(TAG, "recv_rb is filled");
    }
}

static int amr_read_cb(audio_element_handle_t self, char *buffer, int len, TickType_t ticks_to_wait, void *context)
{
    int read_bytes = 0;
    unsigned char head[] = {
        0x23, 0x21, 0x41, 0x4D, 0x52, 0x2D, 0x57, 0x42, 0x0A
    }; // AMRWB header
    char header[1];
    if (receive_ID == 0) {
        if (len <= AMRWB_HEADER_SIZE) {
            res_header = AMRWB_HEADER_SIZE - len;
            memcpy(buffer, AMRWB_HEADER_INFO, len);
        } else {
            memcpy(buffer, AMRWB_HEADER_INFO, AMRWB_HEADER_SIZE);
            read_bytes = len;
        }
        ESP_LOGI(TAG, "Write AMRWB_HEADER_INFO: %s", buffer);
        read_bytes = len;
    } else {
        if (res_header) {
            memcpy(buffer, AMRWB_HEADER_INFO + AMRWB_HEADER_SIZE - res_header, res_header);
            res_header = 0;
            ESP_LOGI(TAG, "Write AMRWB_HEADER_INFO: %s", buffer);
            read_bytes = len;
        } else {
            if (send_flag && (RECV_RB_SIZE - rb_bytes_available(recv_rb)) >= len) {
                *header = 0x00;
                if (len == 1) {
                    while (*header != 0x0c) read_bytes = rb_read(recv_rb, header, len, portMAX_DELAY);
                    memcpy(buffer, header, len);
                } else {
                    read_bytes = rb_read(recv_rb, buffer, len, portMAX_DELAY);
                }
            } else {
                if(send_flag != 0) {
                    ESP_LOGI(TAG, "Ringbuf is no data!");
                    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
                }
                if (len == 1) {
                    memset(buffer, 0x0c, len);
                } else {
                    memset(buffer, 0x00, len);
                }
                read_bytes = len;
            }
        }
    }
    receive_ID++;
    return read_bytes;
}

// send audio 
static int amrnb_write_cb(audio_element_handle_t self, char *buffer, int len, TickType_t ticks_to_wait, void *context)
{
    if (send_flag) {
            send_msg.data.size += len;
            if (send_msg.data.size < SEND_LEN) {
                memcpy(send_msg.data.data + send_msg.data.size - len, buffer, len);
            }else if (send_msg.data.size == SEND_LEN) {
                memcpy(send_msg.data.data + send_msg.data.size - len, buffer, len);
                RCSOFTCHECK(rcl_publish(&publisher, &send_msg, NULL));
                memset(send_msg.data.data, 0x00, send_msg.data.size);
                send_msg.data.size = 0;
                return len;
            }else if (send_msg.data.size > SEND_LEN) {
                ESP_LOGW(TAG, "send_msg if overflow!");
                send_msg.data.size -= len;
                RCSOFTCHECK(rcl_publish(&publisher, &send_msg, NULL));
                memset(send_msg.data.data, 0x00, send_msg.data.size);
                send_msg.data.size = 0;
            }
    }
    return len;
}

static esp_err_t input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx)
{
    /* Handle touch pad events
           to start, pause, resume, finish current song and adjust volume
        */
    audio_board_handle_t board_handle = (audio_board_handle_t) ctx;
    int player_volume;
    audio_hal_get_volume(board_handle->audio_hal, &player_volume);
    if (evt->type == INPUT_KEY_SERVICE_ACTION_CLICK_RELEASE) {
        switch ((int)evt->data) {
            case INPUT_KEY_USER_ID_PLAY:
                ESP_LOGI(TAG, "[ * ] [Play] input key event");
                send_flag = true;
                ESP_LOGW(TAG, "START recording");
                break;
            case INPUT_KEY_USER_ID_SET:
                ESP_LOGI(TAG, "[ * ] [Set] input key event");
                send_flag = false;
                ESP_LOGW(TAG, "STOP recording");
                break;
            case INPUT_KEY_USER_ID_VOLUP:
                ESP_LOGI(TAG, "[ * ] [Vol+] input key event");
                player_volume += 10;
                if (player_volume > 100) {
                    player_volume = 100;
                }
                audio_hal_set_volume(board_handle->audio_hal, player_volume);
                ESP_LOGI(TAG, "[ * ] Volume set to %d %%", player_volume);
                break;
            case INPUT_KEY_USER_ID_VOLDOWN:
                ESP_LOGI(TAG, "[ * ] [Vol-] input key event");
                player_volume -= 10;
                if (player_volume < 0) {
                    player_volume = 0;
                }
                audio_hal_set_volume(board_handle->audio_hal, player_volume);
                ESP_LOGI(TAG, "[ * ] Volume set to %d %%", player_volume);
                break;
        }
    }
    return ESP_OK;
}

void app_main(void)
{    
    uint8_t *recv_buf = (uint8_t *)audio_calloc(1, (SEND_LEN) * sizeof(uint8_t));
    uint8_t *send_buf = (uint8_t *)audio_calloc(1, (SEND_LEN) * sizeof(uint8_t));
    // Initialize send_msg
    send_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension *)audio_calloc(1, sizeof(std_msgs__msg__MultiArrayDimension));
    send_msg.layout.data_offset = 0;
    send_msg.layout.dim.size = 0;
    send_msg.layout.dim.capacity = SEND_LEN;
    send_msg.data.capacity = SEND_LEN;
    send_msg.data.data = (uint8_t *)audio_calloc(1, (SEND_LEN) * sizeof(uint8_t));
    send_msg.data.size = 0;

    //Initialize recv_msg
    recv_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension *)audio_calloc(1, sizeof(std_msgs__msg__MultiArrayDimension));
    recv_msg.layout.data_offset = 0;
    recv_msg.layout.dim.size = 0;
    recv_msg.layout.dim.capacity = RECV_LEN;
    recv_msg.data.capacity = RECV_LEN;
    recv_msg.data.data = (uint8_t *)audio_calloc(1, (RECV_LEN) * sizeof(uint8_t));
    recv_msg.data.size = 0;

    // Initialize peripherals and audio pipeline
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    ESP_LOGI(TAG, "[ 1 ] Initialize peripherals");
    esp_periph_set_handle_t set;
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    set = esp_periph_set_init(&periph_cfg);
    audio_board_key_init(set);

    ESP_LOGI(TAG, "[ 2 ] Start codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);
    
    ESP_LOGI(TAG, "[ 3 ] Create and start input key service");
    input_key_service_info_t input_key_info[] = INPUT_KEY_DEFAULT_INFO();
    input_key_service_cfg_t input_cfg = INPUT_KEY_SERVICE_DEFAULT_CONFIG();
    input_cfg.handle = set;
    periph_service_handle_t input_ser = input_key_service_create(&input_cfg);
    input_key_service_add_key(input_ser, input_key_info, INPUT_KEY_NUM);
    periph_service_set_callback(input_ser, input_key_service_cb, (void *)board_handle);

    ESP_LOGI(TAG, "[ 3 ] Create audio pipeline for playback");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline_in = audio_pipeline_init(&pipeline_cfg);
    pipeline_out = audio_pipeline_init(&pipeline_cfg);
    
    ESP_LOGI(TAG, "[3.1] Create i2s stream to write data to codec chip and read data from codec chip");
    i2s_stream_reader = create_i2s_stream(PLAY_RATE, PLAY_BITS, PLAY_CHANNEL, AUDIO_STREAM_READER);
    amrwb_encoder_el = create_amrwb_encoder();
    
    audio_element_handle_t amr_decoder_el = create_amr_decoder();
    audio_element_handle_t i2s_stream_writer = create_i2s_stream(PLAY_RATE, PLAY_BITS, PLAY_CHANNEL, AUDIO_STREAM_WRITER);
    
    recv_rb = rb_create(RECV_RB_SIZE, 1);
    rbuf = (char *)audio_calloc(1, 120);
    // Set amr callbcak function
    audio_element_set_write_cb(amrwb_encoder_el, amrnb_write_cb, (void *)NULL);
    audio_element_set_read_cb(amr_decoder_el, amr_read_cb, (void *)NULL);

    ESP_LOGI(TAG, "[3.2] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline_out, i2s_stream_reader,    "i2s_reader");
    audio_pipeline_register(pipeline_in, i2s_stream_writer,     "i2s_writer");

    audio_pipeline_register(pipeline_out, amrwb_encoder_el,     "amr_encoder");
    audio_pipeline_register(pipeline_in, amr_decoder_el,        "amr_decoder");

    ESP_LOGI(TAG, "[3.3] Link it together [Micro-ros]-->i2s_stream_writer-->amr_decoder-->[codec_chip]\
     [codec_chip]-->i2s_stream_reader-->amrnb_encoder-->[Micro-ros]");
    const char *link_out[2] = {"i2s_reader","amr_encoder"};
    audio_pipeline_link(pipeline_out, &link_out[0], 2);
    const char *link_in[2] = {"amr_decoder", "i2s_writer"};    
    audio_pipeline_link(pipeline_in, &link_in[0], 2);

    ESP_LOGI(TAG, "[ 4 ] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);
    // // audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);

    ESP_LOGI(TAG, "[5] Start audio_pipeline");
    audio_pipeline_set_listener(pipeline_out, evt);
    audio_pipeline_set_listener(pipeline_in, evt);

    ESP_LOGW(TAG, "[ 6 ] Press the keys to communicate:");
    ESP_LOGW(TAG, "      [Play] to start.");

    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
        ESP_ERROR_CHECK(uros_network_interface_initialize());
    #endif

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Create init_options.
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif
    
    // create init_options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Initialize micro-ROS node
    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "Micro_ROS_audio_com_node", "", &support));

    // Initialize publisher    
    RCCHECK(rclc_publisher_init_best_effort(
        &publisher, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ByteMultiArray), 
        "Esp32_audio_data_1"));

    // Initialize subscriber
    RCCHECK(rclc_subscription_init_best_effort(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ByteMultiArray),
        "Esp32_audio_data_2"));

	// Create publish timer.
	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 10;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		microros_timer_callback));

    // Create executor. 
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); // 第三个参数为最大任务数
    unsigned int rcl_wait_timeout = 1;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    // Add timer and subscriber to executor.
	// RCCHECK(rclc_executor_add_timer(&executor, &timer));          //自定义定时器内容
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));

    i2s_stream_set_clk(i2s_stream_writer, PLAY_RATE, PLAY_BITS, PLAY_CHANNEL);
    audio_pipeline_run(pipeline_out);
    audio_pipeline_run(pipeline_in);

    while (1) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, 1);
        // Spin once
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
        usleep(1000);  //microseconds
    }

    ESP_LOGI(TAG, "[ 6 ] Pipeline stopped");
    // Terminate the pipeline
    audio_pipeline_stop(pipeline_in);
    audio_pipeline_wait_for_stop(pipeline_in);
    audio_pipeline_terminate(pipeline_in);
    audio_pipeline_stop(pipeline_out);
    audio_pipeline_wait_for_stop(pipeline_out);
    audio_pipeline_terminate(pipeline_out);

    // Unregister and release elements
    audio_pipeline_unregister(pipeline_in, i2s_stream_reader);
    audio_pipeline_unregister(pipeline_out, i2s_stream_writer);


    /* Terminate the pipeline before removing the listener */    
    audio_pipeline_remove_listener(pipeline_in);
    audio_pipeline_remove_listener(pipeline_out);

    /* Stop all peripherals before removing the listener */
    esp_periph_set_stop_all(set);
    audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);

    /* Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface */
    audio_event_iface_destroy(evt);

    /* Release all resources */
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));
    audio_pipeline_deinit(pipeline_in);
    audio_pipeline_deinit(pipeline_out);
    audio_element_deinit(i2s_stream_reader);
    audio_element_deinit(i2s_stream_writer);
    esp_periph_set_destroy(set);
}
