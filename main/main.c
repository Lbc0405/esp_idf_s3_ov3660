#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_camera.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
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

// ========== MPU6050 偏航角计算 (快速高精度标定版) ==========
#include "driver/i2c_master.h"

#define MPU6050_ADDR         0x68
#define MPU6050_SDA_GPIO     47
#define MPU6050_SCL_GPIO     21
#define I2C_MASTER_FREQ_HZ   400000

// MPU6050 寄存器地址
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_TEMP_OUT_H   0x41
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_WHO_AM_I      0x75

// 量程转换系数 (±2g, ±250°/s)
#define ACCEL_SCALE         16384.0f
#define GYRO_SCALE          131.0f

#define SAMPLE_PERIOD_MS    20      // 偏航角计算周期

// 标定参数
#define CALIB_MAX_SAMPLES   100     // 最大采样数（上限保护）
#define CALIB_MIN_SAMPLES   25      // 最少采样数（中值滤波窗口）
#define CALIB_DELAY_MS      4       // 采样间隔 4ms
#define STABLE_STD_GZ       40.0f   // gz 标准差阈值（原始值，约 0.3°/s）
#define STABLE_STD_AX       80.0f   // ax 标准差阈值
#define STABLE_STD_AY       80.0f   // ay 标准差阈值
#define STABLE_STD_AZ       80.0f   // az 标准差阈值

// ==================== MPU6050 数据结构 ====================

typedef struct {
    // 陀螺仪零偏
    float gx_bias, gy_bias, gz_bias;
    // 重力方向向量（倾斜安装时的重力分量）
    float grav_ax, grav_ay, grav_az;
    // 重力模长
    float grav_mag;
    // 标定是否成功
    bool is_calibrated;
    // 实际采样次数
    int actual_samples;
    // 标定耗时 (ms)
    int calib_time_ms;
} MPU_Calibrator_t;

typedef struct {
    MPU_Calibrator_t *calib;

    // 当前角速度 (°/s)
    float gx, gy, gz;

    // 偏航角 (°)，积分得到
    float yaw_angle;

    // 俯仰角、横滚角（辅助参考）
    float pitch;
    float roll;

    // 上一次更新时间
    int64_t last_update_us;
} Yaw_Calculator_t;

// ==================== I2C 句柄 ====================

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

static MPU_Calibrator_t calibrator;
static Yaw_Calculator_t *yaw_calc = NULL;

// ==================== I2C 底层 ====================

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

static int16_t combine_bytes(uint8_t h, uint8_t l)
{
    return (int16_t)((h << 8) | l);
}

// ==================== MPU6050 基础功能 ====================

static bool mpu6050_detect(void)
{
    uint8_t whoami;
    esp_err_t ret = mpu6050_read_bytes(MPU6050_WHO_AM_I, &whoami, 1);
    if (ret != ESP_OK || whoami != 0x68) {
        ESP_LOGE(TAG, "MPU6050 未检测到 (WHO_AM_I=0x%02X, ret=%d)", whoami, ret);
        return false;
    }
    ESP_LOGI(TAG, "MPU6050 检测成功");
    return true;
}

static void mpu6050_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                             int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t data[14];
    mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H, data, 14);

    *ax = combine_bytes(data[0], data[1]);
    *ay = combine_bytes(data[2], data[3]);
    *az = combine_bytes(data[4], data[5]);
    *gx = combine_bytes(data[8], data[9]);
    *gy = combine_bytes(data[10], data[11]);
    *gz = combine_bytes(data[12], data[13]);
}

// ==================== 工具函数 ====================

/*
 * 快速选择中值（Hoare 选择算法）
 * O(n) 复杂度，比排序快
 */
static float quick_select_median(int16_t *arr, int n)
{
    int16_t temp[CALIB_MAX_SAMPLES];
    memcpy(temp, arr, n * sizeof(int16_t));

    int low = 0, high = n - 1;
    int k = n / 2;

    while (low <= high) {
        int16_t pivot = temp[high];
        int i = low - 1;
        for (int j = low; j < high; j++) {
            if (temp[j] <= pivot) {
                i++;
                int16_t t = temp[i]; temp[i] = temp[j]; temp[j] = t;
            }
        }
        int16_t t = temp[i + 1]; temp[i + 1] = temp[high]; temp[high] = t;
        int pi = i + 1;

        if (pi == k) return (float)temp[pi];
        else if (pi < k) low = pi + 1;
        else high = pi - 1;
    }
    return (float)temp[0];
}

/*
 * 计算标准差
 */
static float calculate_stddev(float *arr, int n)
{
    if (n <= 1) return 9999.0f;

    float sum = 0.0f;
    for (int i = 0; i < n; i++) sum += arr[i];
    float mean = sum / n;

    float variance = 0.0f;
    for (int i = 0; i < n; i++) {
        float diff = arr[i] - mean;
        variance += diff * diff;
    }
    variance /= (n - 1);  // 样本方差

    return sqrtf(variance);
}

// ==================== 快速高精度标定 ====================

/*
 * 标定策略：
 * 1. 持续采样，同时维护滑动窗口
 * 2. 达到最小样本后，检查窗口内数据稳定性（标准差）
 * 3. 数据稳定 → 提前结束，节省时间
 * 4. 数据不稳定 → 继续采样到最大次数
 * 5. 最终用所有采集数据做中值滤波，去除野点
 */
static bool mpu_calibrate_fast(MPU_Calibrator_t *calib)
{
    int64_t start_time = esp_timer_get_time();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "【快速标定】保持静止...");
    ESP_LOGI(TAG, "最少%d次，最多%d次，间隔%dms",
             CALIB_MIN_SAMPLES, CALIB_MAX_SAMPLES, CALIB_DELAY_MS);
    ESP_LOGI(TAG, "========================================");

    // 原始数据缓冲区
    int16_t buf_ax[CALIB_MAX_SAMPLES];
    int16_t buf_ay[CALIB_MAX_SAMPLES];
    int16_t buf_az[CALIB_MAX_SAMPLES];
    int16_t buf_gx[CALIB_MAX_SAMPLES];
    int16_t buf_gy[CALIB_MAX_SAMPLES];
    int16_t buf_gz[CALIB_MAX_SAMPLES];

    // 滑动窗口（用于稳定性判断）
    float win_gz[CALIB_MIN_SAMPLES];
    float win_ax[CALIB_MIN_SAMPLES];
    float win_ay[CALIB_MIN_SAMPLES];
    float win_az[CALIB_MIN_SAMPLES];
    int win_idx = 0;

    int i;
    for (i = 0; i < CALIB_MAX_SAMPLES; i++) {
        mpu6050_read_raw(&buf_ax[i], &buf_ay[i], &buf_az[i],
                         &buf_gx[i], &buf_gy[i], &buf_gz[i]);

        // 更新滑动窗口
        win_gz[win_idx] = (float)buf_gz[i];
        win_ax[win_idx] = (float)buf_ax[i];
        win_ay[win_idx] = (float)buf_ay[i];
        win_az[win_idx] = (float)buf_az[i];
        win_idx = (win_idx + 1) % CALIB_MIN_SAMPLES;

        // 达到最小样本数后，检查稳定性
        if (i >= CALIB_MIN_SAMPLES - 1) {
            float std_gz = calculate_stddev(win_gz, CALIB_MIN_SAMPLES);
            float std_ax = calculate_stddev(win_ax, CALIB_MIN_SAMPLES);
            float std_ay = calculate_stddev(win_ay, CALIB_MIN_SAMPLES);
            float std_az = calculate_stddev(win_az, CALIB_MIN_SAMPLES);

            // 所有轴都稳定 → 提前结束
            if (std_gz < STABLE_STD_GZ &&
                std_ax < STABLE_STD_AX &&
                std_ay < STABLE_STD_AY &&
                std_az < STABLE_STD_AZ) {

                int actual = i + 1;
                int elapsed_ms = (int)((esp_timer_get_time() - start_time) / 1000);

                ESP_LOGI(TAG, "✓ 数据已稳定，提前结束");
                ESP_LOGI(TAG, "  采样次数: %d (最少要求%d)", actual, CALIB_MIN_SAMPLES);
                ESP_LOGI(TAG, "  耗时: %d ms", elapsed_ms);
                ESP_LOGI(TAG, "  标准差: gz=%.1f ax=%.1f ay=%.1f az=%.1f",
                         std_gz, std_ax, std_ay, std_az);

                calib->actual_samples = actual;
                calib->calib_time_ms = elapsed_ms;
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CALIB_DELAY_MS));
    }

    // 如果达到最大次数仍未稳定
    if (i >= CALIB_MAX_SAMPLES) {
        int elapsed_ms = (int)((esp_timer_get_time() - start_time) / 1000);
        ESP_LOGW(TAG, "⚠ 达到最大采样数，数据未完全稳定");
        ESP_LOGI(TAG, "  采样次数: %d", CALIB_MAX_SAMPLES);
        ESP_LOGI(TAG, "  耗时: %d ms", elapsed_ms);
        calib->actual_samples = CALIB_MAX_SAMPLES;
        calib->calib_time_ms = elapsed_ms;
    }

    int n = calib->actual_samples;

    // ========== 中值滤波去除野点 ==========
    ESP_LOGI(TAG, "进行中值滤波...");

    calib->grav_ax = quick_select_median(buf_ax, n);
    calib->grav_ay = quick_select_median(buf_ay, n);
    calib->grav_az = quick_select_median(buf_az, n);
    calib->gx_bias = quick_select_median(buf_gx, n);
    calib->gy_bias = quick_select_median(buf_gy, n);
    calib->gz_bias = quick_select_median(buf_gz, n);

    calib->grav_mag = sqrtf(calib->grav_ax * calib->grav_ax +
                            calib->grav_ay * calib->grav_ay +
                            calib->grav_az * calib->grav_az);

    // 验证重力模长
    ESP_LOGI(TAG, "标定结果:");
    ESP_LOGI(TAG, "  重力分量: ax=%.1f, ay=%.1f, az=%.1f",
             calib->grav_ax, calib->grav_ay, calib->grav_az);
    ESP_LOGI(TAG, "  重力模长: %.1f (应接近 16384)", calib->grav_mag);
    ESP_LOGI(TAG, "  陀螺零偏: gx=%.1f, gy=%.1f, gz=%.1f",
             calib->gx_bias, calib->gy_bias, calib->gz_bias);

    if (fabsf(calib->grav_mag - 16384.0f) > 3000.0f) {
        ESP_LOGW(TAG, "⚠ 警告：重力模长偏离过大，请检查模块是否晃动");
        calib->is_calibrated = false;
    } else {
        ESP_LOGI(TAG, "✓ 标定成功");
        calib->is_calibrated = true;
    }

    // 静止验证
    ESP_LOGI(TAG, "静止验证:");
    for (int j = 0; j < 5; j++) {
        float ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps;
        int16_t ax, ay, az, gx, gy, gz;
        mpu6050_read_raw(&ax, &ay, &az, &gx, &gy, &gz);
        ax_g = (ax - calib->grav_ax) / ACCEL_SCALE;
        ay_g = (ay - calib->grav_ay) / ACCEL_SCALE;
        az_g = (az - calib->grav_az) / ACCEL_SCALE;
        gx_dps = (gx - calib->gx_bias) / GYRO_SCALE;
        gy_dps = (gy - calib->gy_bias) / GYRO_SCALE;
        gz_dps = (gz - calib->gz_bias) / GYRO_SCALE;

        ESP_LOGI(TAG, "  A:%6.3fg %6.3fg %6.3fg | G:%7.2f°/s %7.2f°/s %7.2f°/s",
                 ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    return calib->is_calibrated;
}

// ==================== 偏航角计算 ====================

static Yaw_Calculator_t* yaw_create(MPU_Calibrator_t *calib)
{
    Yaw_Calculator_t *calc = (Yaw_Calculator_t*)malloc(sizeof(Yaw_Calculator_t));
    if (!calc) return NULL;

    memset(calc, 0, sizeof(Yaw_Calculator_t));
    calc->calib = calib;
    calc->last_update_us = esp_timer_get_time();

    return calc;
}

/*
 * 获取标定补偿后的角速度和加速度
 */
static void get_calibrated_data(Yaw_Calculator_t *calc,
                                 float *ax_g, float *ay_g, float *az_g,
                                 float *gx_dps, float *gy_dps, float *gz_dps)
{
    int16_t ax, ay, az, gx, gy, gz;
    mpu6050_read_raw(&ax, &ay, &az, &gx, &gy, &gz);

    MPU_Calibrator_t *c = calc->calib;

    // 扣除重力分量，转换为 g
    *ax_g = (ax - c->grav_ax) / ACCEL_SCALE;
    *ay_g = (ay - c->grav_ay) / ACCEL_SCALE;
    *az_g = (az - c->grav_az) / ACCEL_SCALE;

    // 扣除零偏，转换为 °/s
    *gx_dps = (gx - c->gx_bias) / GYRO_SCALE;
    *gy_dps = (gy - c->gy_bias) / GYRO_SCALE;
    *gz_dps = (gz - c->gz_bias) / GYRO_SCALE;
}

/*
 * 更新偏航角
 */
static void yaw_update(Yaw_Calculator_t *calc)
{
    int64_t now_us = esp_timer_get_time();
    float dt = (now_us - calc->last_update_us) / 1000000.0f;
    calc->last_update_us = now_us;

    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.02f;
    }

    float ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps;
    get_calibrated_data(calc, &ax_g, &ay_g, &az_g, &gx_dps, &gy_dps, &gz_dps);

    calc->gx = gx_dps;
    calc->gy = gy_dps;
    calc->gz = gz_dps;

    // 计算俯仰角和横滚角（用原始加速度）
    int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
    mpu6050_read_raw(&ax_r, &ay_r, &az_r, &gx_r, &gy_r, &gz_r);

    float ax_f = (float)ax_r;
    float ay_f = (float)ay_r;
    float az_f = (float)az_r;

    float norm = sqrtf(ax_f * ax_f + ay_f * ay_f + az_f * az_f);
    if (norm > 0.0f) {
        calc->pitch = asinf(-ax_f / norm) * 180.0f / M_PI;
        calc->roll  = atan2f(ay_f, az_f) * 180.0f / M_PI;
    }

    // 偏航角 = 对 gz 积分
    calc->yaw_angle += gz_dps * dt;

    // 归一化到 [-180, 180]
    while (calc->yaw_angle > 180.0f) calc->yaw_angle -= 360.0f;
    while (calc->yaw_angle < -180.0f) calc->yaw_angle += 360.0f;
}

static void yaw_reset(Yaw_Calculator_t *calc)
{
    calc->yaw_angle = 0.0f;
    calc->last_update_us = esp_timer_get_time();
}

static float yaw_get_angle(Yaw_Calculator_t *calc)
{
    return calc->yaw_angle;
}

// ========== UART 初始化（预留） ==========
#include "driver/uart.h"

// Strapping 引脚注意事项：
//  GPIO45 控制 VDD_SPI 电压，启动时必须浮空/高（UART TX 空闲态=高，兼容）
//  GPIO46 控制启动日志路由，启动时外设勿拉低
#define UART_TX_GPIO    45
#define UART_RX_GPIO    46
#define UART_PORT       UART_NUM_1
#define UART_BAUD_RATE  115200

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

    // ---- 初始化 I2C 和 MPU6050 ----
    i2c_master_init();
    if (!mpu6050_detect()) {
        ESP_LOGE(TAG, "MPU6050 初始化失败");
        esp_restart();
    }
    // 唤醒 MPU6050（退出休眠模式）
    mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));

    // 快速标定
    if (!mpu_calibrate_fast(&calibrator)) {
        ESP_LOGW(TAG, "标定未通过，但继续运行...");
    }

    yaw_calc = yaw_create(&calibrator);
    if (!yaw_calc) {
        ESP_LOGE(TAG, "内存分配失败");
        esp_restart();
    }

    uart_init();

    // 注册当前任务到看门狗，超时设为 5 秒
    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "偏航角计算已启动 | 标定耗时: %d ms, 采样: %d 次",
             calibrator.calib_time_ms, calibrator.actual_samples);
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

        // 加权合并: 近端70% + 远端30%
        int dev_weighted;
        if (deviation1 == 9999 && deviation2 == 9999) {
            dev_weighted = 9999;
        } else if (deviation1 == 9999) {
            dev_weighted = deviation2;
        } else if (deviation2 == 9999) {
            dev_weighted = deviation1;
        } else {
            dev_weighted = (int)(deviation1 * 0.7f + deviation2 * 0.3f);
        }

        // 巡线结果
        ESP_LOGI(TAG,
                "Frame: %dx%d | ROI1 dev=%d ROI2 dev=%d | weighted=%d",
                width, height,
                deviation1, deviation2, dev_weighted);

        // ---- 更新偏航角 ----
        yaw_update(yaw_calc);

        // ---- UART 发送: 加权偏差 + 偏航角 ----
        // 协议: 0xAA 0x55 | dev(2B) | yaw(2B, 0.01°) | xor | \n
        int16_t yaw_send = (int16_t)(yaw_calc->yaw_angle * 100.0f);
        uint8_t pkt[8];
        pkt[0] = 0xAA;
        pkt[1] = 0x55;
        pkt[2] = (uint8_t)(dev_weighted & 0xFF);
        pkt[3] = (uint8_t)((dev_weighted >> 8) & 0xFF);
        pkt[4] = (uint8_t)(yaw_send & 0xFF);
        pkt[5] = (uint8_t)((yaw_send >> 8) & 0xFF);
        pkt[6] = pkt[2] ^ pkt[3] ^ pkt[4] ^ pkt[5];
        pkt[7] = '\n';
        uart_write_bytes(UART_PORT, pkt, sizeof(pkt));

        ESP_LOGI(TAG,
                "Yaw:%7.2f° Pitch:%6.2f° Roll:%6.2f° | gz:%7.2f°/s",
                yaw_calc->yaw_angle,
                yaw_calc->pitch,
                yaw_calc->roll,
                yaw_calc->gz);

        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(50)); // 20 FPS
    }
}
