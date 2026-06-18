#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_camera.h"
#include "esp_log.h"

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

void app_main(void)
{
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
        return;
    }
    ESP_LOGI(TAG, "Camera Init OK, resolution: 640x480, grayscale");

    sensor_t *s = esp_camera_sensor_get();
    ESP_LOGI(TAG, "Sensor PID: 0x%02X", s->id.PID);

    // 设置双 ROI 起始行，假设图像高度 480
    int roi1_y = ROI1_Y_START;
    int roi1_h = ROI1_HEIGHT;
    int roi2_y = ROI2_Y_START;
    int roi2_h = ROI2_HEIGHT;
    int img_w = 640;
    int img_h = 480;

    while(1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if(!fb) {
            ESP_LOGE(TAG, "Capture Failed");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // 确保是灰度图
        if (fb->format != PIXFORMAT_GRAYSCALE) {
            ESP_LOGW(TAG, "Unexpected format: %d", fb->format);
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        const uint8_t *gray = fb->buf;
        int width  = fb->width;
        int height = fb->height;

        // 计算两个 ROI 的赛道重心
        int center1 = calc_black_center(gray, width, height, roi1_y, roi1_h);
        int center2 = calc_black_center(gray, width, height, roi2_y, roi2_h);

        // 计算偏移（相对于图像水平中心 320）
        int deviation1 = (center1 >= 0) ? (center1 - 320) : 9999;  // 9999 表示没检测到
        int deviation2 = (center2 >= 0) ? (center2 - 320) : 9999;

        ESP_LOGI(TAG,
                 "Frame: %dx%d | ROI1 center=%d (dev=%d) | ROI2 center=%d (dev=%d)",
                 width, height,
                 center1, deviation1,
                 center2, deviation2);

        // 在此处可以将偏差值发送给电机控制任务（例如通过队列或全局变量）
        // ...

        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(50)); // 20 FPS
    }
}