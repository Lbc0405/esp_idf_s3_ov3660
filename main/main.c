#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_camera.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"

static const char *TAG = "CAM";

// ======= 双 ROI 巡线参数 =======
#define GRAY_THRESHOLD  80          // 灰度阈值，小于此值视为黑色赛道
#define ROI1_Y_START    120         // ROI1 起始行
#define ROI1_HEIGHT     60          // ROI1 高度
#define ROI2_Y_START    300         // ROI2 起始行
#define ROI2_HEIGHT     60          // ROI2 高度
// ROI 宽度均为整幅图像宽度，因为赛道线可能出现在任何水平位置
// =================================

// --- 计算 ROI 内黑色像素的平均列坐标（重心）---
// 返回 -1 表示没有检测到黑色像素
static int calc_black_center(const uint8_t *gray_img, int img_width, int img_height,
                             int roi_y_start, int roi_height)
{
    long sum_x = 0;
    int count = 0;
    int y_end = roi_y_start + roi_height;
    if (y_end > img_height) y_end = img_height;
    if (roi_y_start < 0) roi_y_start = 0;

    for (int y = roi_y_start; y < y_end; y++) {
        for (int x = 0; x < img_width; x++) {
            if (gray_img[y * img_width + x] < GRAY_THRESHOLD) {
                sum_x += x;
                count++;
            }
        }
    }

    if (count == 0) {
        return -1;  // 未检测到赛道
    }
    return (int)(sum_x / count);
}

// ====================================================

#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    15

#define CAM_PIN_SIOD    4
#define CAM_PIN_SIOC    5

#define CAM_PIN_D7      16
#define CAM_PIN_D6      17
#define CAM_PIN_D5      18
#define CAM_PIN_D4      12
#define CAM_PIN_D3      10
#define CAM_PIN_D2      8
#define CAM_PIN_D1      9
#define CAM_PIN_D0      11

#define CAM_PIN_VSYNC   6
#define CAM_PIN_HREF    7
#define CAM_PIN_PCLK    13

// ========== MPU6050 I2C (new driver_ng API) ==========
#include "driver/i2c_master.h"

#define MPU6050_ADDR         0x68
#define MPU6050_SDA_GPIO     21
#define MPU6050_SCL_GPIO     47
#define I2C_MASTER_FREQ_HZ   400000

// MPU6050 寄存器地址
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_TEMP_OUT_H   0x41
#define MPU6050_PWR_MGMT_1   0x6B

typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t temp;
    int16_t gyro_x, gyro_y, gyro_z;
} mpu6050_data_t;

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

static void i2c_master_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,          // I2C_NUM_1 已被摄像头 SCCB 占用
        .sda_io_num = MPU6050_SDA_GPIO,
        .scl_io_num = MPU6050_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
}

static esp_err_t mpu6050_write_byte(uint8_t reg, uint8_t val)
{
    uint8_t write_buf[2] = { reg, val };
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), pdMS_TO_TICKS(100));
}

static esp_err_t mpu6050_read_bytes(uint8_t reg_start, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_start, 1, data, len, pdMS_TO_TICKS(100));
}

static esp_err_t mpu6050_read_all(mpu6050_data_t *out)
{
    uint8_t buf[14];
    esp_err_t ret = mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H, buf, 14);
    if (ret != ESP_OK) return ret;

    out->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    out->accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    out->accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    out->temp    = (int16_t)((buf[6] << 8) | buf[7]);
    out->gyro_x  = (int16_t)((buf[8] << 8) | buf[9]);
    out->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    out->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);

    return ESP_OK;
}

// ========== UART 打包发送 ==========
#include "driver/uart.h"

#define UART_TX_GPIO    1
#define UART_RX_GPIO    2
#define UART_PORT       UART_NUM_1
#define UART_BAUD_RATE  115200

#define PACKET_HEADER   0xAA
#define PACKET_LEN      (1 + sizeof(mpu6050_data_t) + 1)

static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_TX_GPIO, UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT, 256, 0, 0, NULL, 0);
}

static void __attribute__((unused)) uart_send_mpu6050_packet(const mpu6050_data_t *data)
{
    uint8_t packet[PACKET_LEN];
    int idx = 0;

    packet[idx++] = PACKET_HEADER;

    uint8_t *raw = (uint8_t *)data;
    for (int i = 0; i < sizeof(mpu6050_data_t); i++) {
        packet[idx++] = raw[i];
    }

    uint8_t checksum = 0;
    for (int i = 1; i < idx; i++) {
        checksum += packet[i];
    }
    packet[idx++] = checksum;

    uart_write_bytes(UART_PORT, packet, idx);
}

void app_main(void)
{
    // 检查 PSRAM 是否可用（VGA 灰度需要 ~600KB 帧缓冲）
    if (!esp_psram_is_initialized()) {
        ESP_LOGE(TAG, "PSRAM is not initialized!");
        esp_restart();
    }

    camera_config_t config = {
        .pin_pwdn       = CAM_PIN_PWDN,
        .pin_reset      = CAM_PIN_RESET,
        .pin_xclk       = CAM_PIN_XCLK,
        .pin_sccb_sda   = CAM_PIN_SIOD,
        .pin_sccb_scl   = CAM_PIN_SIOC,
        .pin_d7         = CAM_PIN_D7,
        .pin_d6         = CAM_PIN_D6,
        .pin_d5         = CAM_PIN_D5,
        .pin_d4         = CAM_PIN_D4,
        .pin_d3         = CAM_PIN_D3,
        .pin_d2         = CAM_PIN_D2,
        .pin_d1         = CAM_PIN_D1,
        .pin_d0         = CAM_PIN_D0,
        .pin_vsync      = CAM_PIN_VSYNC,
        .pin_href       = CAM_PIN_HREF,
        .pin_pclk       = CAM_PIN_PCLK,
        .xclk_freq_hz   = 20000000,
        .ledc_timer     = LEDC_TIMER_0,
        .ledc_channel   = LEDC_CHANNEL_0,
        .pixel_format   = PIXFORMAT_GRAYSCALE,
        .frame_size     = FRAMESIZE_VGA,          // 640 x 480
        .fb_count       = 2,
        .grab_mode      = CAMERA_GRAB_WHEN_EMPTY
    };

    esp_err_t err = esp_camera_init(&config);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed: 0x%x", err);
        esp_restart();
    }
    ESP_LOGI(TAG, "Camera Init OK, resolution: 640x480, grayscale");

    sensor_t *s = esp_camera_sensor_get();
    ESP_LOGI(TAG, "Sensor PID: 0x%02X", s->id.PID);

    // ---- 初始化 I2C 和 UART ----
    i2c_master_init();
    // 唤醒 MPU6050（退出休眠模式）
    if (mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x00) != ESP_OK) {
        ESP_LOGW(TAG, "MPU6050 wake-up failed");
    }
    uart_init();

    // 注册当前任务到看门狗，超时设为 5 秒
    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "Entering main loop");

    while (1) {
        esp_task_wdt_reset();  // 喂狗
        camera_fb_t *fb = esp_camera_fb_get();
        if(!fb) {
            ESP_LOGE(TAG, "Capture Failed");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (fb->format != PIXFORMAT_GRAYSCALE) {
            ESP_LOGW(TAG, "Unexpected format: %d", fb->format);
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        const uint8_t *gray = fb->buf;
        int width  = fb->width;
        int height = fb->height;

        int center1 = calc_black_center(gray, width, height, ROI1_Y_START, ROI1_HEIGHT);
        int center2 = calc_black_center(gray, width, height, ROI2_Y_START, ROI2_HEIGHT);
        int deviation1 = (center1 >= 0) ? (center1 - 320) : 9999;
        int deviation2 = (center2 >= 0) ? (center2 - 320) : 9999;

        // 巡线结果（已有日志）
        ESP_LOGI(TAG,
                "Frame: %dx%d | ROI1 center=%d (dev=%d) | ROI2 center=%d (dev=%d)",
                width, height,
                center1, deviation1,
                center2, deviation2);

        // ---- 读取 MPU6050 并输出到终端 ----
        mpu6050_data_t mpu_data;
        if (mpu6050_read_all(&mpu_data) == ESP_OK) {
            ESP_LOGI(TAG,
                    "MPU6050: accel=(%d,%d,%d) gyro=(%d,%d,%d) temp=%d",
                    mpu_data.accel_x, mpu_data.accel_y, mpu_data.accel_z,
                    mpu_data.gyro_x, mpu_data.gyro_y, mpu_data.gyro_z,
                    mpu_data.temp);
            // 如果以后需要二进制 UART 发送，取消下面注释：
            // uart_send_mpu6050_packet(&mpu_data);
        } else {
            ESP_LOGW(TAG, "MPU6050 read fail");
        }

        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(50)); // 20 FPS
    }
}
