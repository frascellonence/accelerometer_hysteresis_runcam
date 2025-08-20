#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "mc36xx/m_drv_mc36xx.h"
#include "mc36xx/m_drv_interface.h"

#include "motion_detect.h"

#define WINDOW_SIZE 30
#define SAMPLE_RATE_HZ 54

static float walk_freq_min = 1.0f;
static float walk_freq_max = 2.5f;
static float run_freq_min  = 2.5f;
static float walk_amp_thr  = 1.20f;
static float run_amp_thr   = 2.0f;
static float run_sensitivity = 1.0f;

typedef struct {
    float x, y, z;
} sample_t;

static sample_t ring[WINDOW_SIZE];
static int ring_count = 0;
static float last_x = 0.0f, last_y = 0.0f, last_z = 0.0f;
static float dbg_last_freq = 0.0f, dbg_last_amp = 0.0f, dbg_last_thr = 0.0f;
static int dbg_last_peaks = 0;

static float calculate_centered_magnitude(float out_centered[WINDOW_SIZE], int len, float *out_pp_amp) {
    if (len <= 0) { if (out_pp_amp) *out_pp_amp = 0.0f; return 0.0f; }
    float mean_x = 0.0f, mean_y = 0.0f, mean_z = 0.0f;
    for (int i = 0; i < len; i++) { mean_x += ring[i].x; mean_y += ring[i].y; mean_z += ring[i].z; }
    mean_x /= len; mean_y /= len; mean_z /= len;
    float min_v = 1e9f, max_v = -1e9f;
    for (int i = 0; i < len; i++) {
        float cx = ring[i].x - mean_x;
        float cy = ring[i].y - mean_y;
        float cz = ring[i].z - mean_z;
        float m = sqrtf(cx*cx + cy*cy + cz*cz);
        out_centered[i] = m;
        if (m < min_v) min_v = m;
        if (m > max_v) max_v = m;
    }
    if (out_pp_amp) *out_pp_amp = (max_v - min_v);
    return max_v - min_v;
}

static int count_peaks_with_refractory(const float *data, int length, float threshold, int min_distance_samples) {
    if (length < 3) return 0;
    int count = 0; int last_peak_index = -min_distance_samples;
    for (int i = 1; i < length - 1; i++) {
        if (i - last_peak_index < min_distance_samples) continue;
        if (data[i] > threshold && data[i] > data[i-1] && data[i] > data[i+1]) {
            last_peak_index = i; count++;
        }
    }
    return count;
}

bool motion_init(void) {
    m_drv_i2c_init();
    // Allow sensor power to stabilize
    sleep_ms(10);
    // Simple I2C bus scan to aid debugging
    // Optional: comment out bus scan in production
    // Quick probe both possible I2C addresses
    uint8_t addr_candidates[2] = {0x4C, 0x6C};
    bool identified = false;
    for (int i = 0; i < 2 && !identified; i++) {
        uint8_t addr = addr_candidates[i];
        // Try reading PROD register 0x18
        uint8_t reg = 0x18, val = 0x00;
        int w = i2c_write_timeout_us(i2c0, addr, &reg, 1, true, 1000);
        int r = (w == 1) ? i2c_read_timeout_us(i2c0, addr, &val, 1, false, 1000) : -1;
        if (w == 1 && r == 1) {
            printf("MC36xx PROD (0x18) at 0x%02X = 0x%02X\n", addr, val);
            identified = true;
        }
    }
    // Try initialization a few times in case of first-read timing issues
    for (int attempt = 0; attempt < 3; attempt++) {
        if (M_DRV_MC36XX_Init() == M_DRV_MC36XX_RETCODE_SUCCESS) {
            break;
        }
        sleep_ms(5);
        if (attempt == 2) {
            printf("MC36xx init failed\n");
            return false;
        }
    }
    M_DRV_MC36XX_ConfigRegRngResCtrl(E_M_DRV_MC36XX_RANGE_4G, E_M_DRV_MC36XX_RES_12BIT);
    M_DRV_MC36XX_SetSampleRate(E_M_DRV_MC36XX_CWAKE_SR_LP_54Hz, E_M_DRV_MC36XX_SNIFF_SR_6Hz);
    M_DRV_MC36XX_SetMode(E_M_DRV_MC36XX_MODE_CWAKE);
    return true;
}

void motion_set_sensitivity(float s) {
    if (s < 0.5f) s = 0.5f; if (s > 2.0f) s = 2.0f; run_sensitivity = s;
    run_amp_thr = 2.5f / run_sensitivity;
    run_freq_min = 2.5f - 0.3f * (run_sensitivity - 1.0f);
    if (run_freq_min < 2.0f) run_freq_min = 2.0f;
}

activity_state_t motion_classify_tick(void) {
    float data[M_DRV_MC36XX_FIFO_DEPTH][M_DRV_MC36XX_AXES_NUM] = {0};
    int n = M_DRV_MC36XX_ReadData(data, 1);
    if (n <= 0) return ACTIVITY_IDLE;
    last_x = data[0][M_DRV_MC36XX_AXIS_X];
    last_y = data[0][M_DRV_MC36XX_AXIS_Y];
    last_z = data[0][M_DRV_MC36XX_AXIS_Z];
    if (ring_count >= WINDOW_SIZE) {
        for (int i = 0; i < WINDOW_SIZE - 1; i++) ring[i] = ring[i + 1];
        ring_count = WINDOW_SIZE - 1;
    }
    ring[ring_count].x = data[0][M_DRV_MC36XX_AXIS_X];
    ring[ring_count].y = data[0][M_DRV_MC36XX_AXIS_Y];
    ring[ring_count].z = data[0][M_DRV_MC36XX_AXIS_Z];
    ring_count++;

    if (ring_count < WINDOW_SIZE) return ACTIVITY_IDLE;

    float centered[WINDOW_SIZE];
    float pp_amp = 0.0f;
    calculate_centered_magnitude(centered, ring_count, &pp_amp);
    dbg_last_amp = pp_amp;

    if (pp_amp < 0.10f) return ACTIVITY_IDLE;

    float min_v = centered[0], max_v = centered[0];
    for (int i = 1; i < ring_count; i++) { if (centered[i] < min_v) min_v = centered[i]; if (centered[i] > max_v) max_v = centered[i]; }
    float amp = max_v - min_v;
    float thr = min_v + 0.35f * amp;
    int min_dist = SAMPLE_RATE_HZ / 4; if (min_dist < 2) min_dist = 2;
    int peak_count = count_peaks_with_refractory(centered, ring_count, thr, min_dist);
    dbg_last_peaks = peak_count; dbg_last_thr = thr;
    float window_sec = (float)ring_count / SAMPLE_RATE_HZ;
    float freq = (window_sec > 0.0f) ? ((float)peak_count / window_sec) : 0.0f;
    dbg_last_freq = freq;

    if (amp < 0.08f) return ACTIVITY_IDLE;
    if (freq >= run_freq_min && amp >= run_amp_thr) return ACTIVITY_RUNNING;
    if (freq >= walk_freq_min && freq <= walk_freq_max && amp >= walk_amp_thr) return ACTIVITY_WALKING;
    if (amp >= 0.12f && freq >= 0.8f) return ACTIVITY_WALKING;
    return ACTIVITY_IDLE;
}

void motion_get_last_sample(float *x, float *y, float *z) {
    if (x) *x = last_x;
    if (y) *y = last_y;
    if (z) *z = last_z;
}

void motion_get_debug(motion_debug_t *out) {
    if (!out) return;
    out->last_x = last_x;
    out->last_y = last_y;
    out->last_z = last_z;
    out->last_frequency_hz = dbg_last_freq;
    out->last_amp_pp = dbg_last_amp;
    out->last_peak_count = dbg_last_peaks;
    out->last_peak_threshold = dbg_last_thr;
}


