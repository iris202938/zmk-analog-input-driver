/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_analog_input

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zmk/events/input_event.h>
#include <zmk/endpoints.h>

/* input_processor.h と event_manager.h は v0.3 では不要なため削除しました */

LOG_MODULE_REGISTER(zmk_analog_input, CONFIG_ZMK_INPUT_LOG_LEVEL);

struct analog_input_channel_config {
    uint8_t idx;
    int32_t mv_mid;
    int32_t mv_min_max;
    int32_t mv_deadzone;
    int32_t scale_multiplier;
    int32_t scale_divisor;
    bool invert;
    uint8_t evt_type;
    uint16_t input_code;
    bool report_on_change_only;
};

struct analog_input_channel_data {
    int32_t mv_mid;
    int16_t last_value;
};

struct analog_input_config {
    const struct device *adc_dev;
    struct adc_channel_cfg adc_ch_cfg[2];
    uint16_t sampling_hz;
    struct analog_input_channel_config x_ch_cfg;
    struct analog_input_channel_config y_ch_cfg;
};

struct analog_input_data {
    const struct device *dev;
    struct analog_input_channel_data x_ch_data;
    struct analog_input_channel_data y_ch_data;
    struct k_timer timer;
    struct k_work work;
};

// ADCからmVを読み取るヘルパー関数
static int read_adc_mv(const struct device *dev, const struct adc_channel_cfg *ch_cfg, int32_t *mv) {
    int16_t buf;
    struct adc_sequence seq = {
        .channels = BIT(ch_cfg->channel_id),
        .buffer = &buf,
        .buffer_size = sizeof(buf),
        .resolution = 12,
    };

    int ret = adc_read(dev, &seq);
    if (ret < 0) return ret;

    int32_t val = buf;
    adc_raw_to_mv(adc_ref_internal(dev), ch_cfg->gain, seq.resolution, &val);
    *mv = val;
    return 0;
}

// 起動時のオートキャリブレーション関数
static int32_t calibrate_channel(const struct device *adc_dev, const struct adc_channel_cfg *ch_cfg) {
    int32_t sum = 0;
    int samples = 16; // 16回サンプリングして精度を高める
    int32_t val;

    LOG_INF("Calibrating channel %d...", ch_cfg->channel_id);
    for (int i = 0; i < samples; i++) {
        if (read_adc_mv(adc_dev, ch_cfg, &val) == 0) {
            sum += val;
        }
        k_msleep(5); // 電圧安定のための微小な待ち時間
    }
    return sum / samples;
}

static int analog_input_process_channel(const struct device *dev,
                                        const struct analog_input_channel_config *ch_cfg,
                                        struct analog_input_channel_data *ch_data,
                                        const struct adc_channel_cfg *adc_ch_cfg) {
    int32_t mv;
    if (read_adc_mv(DT_INST_PHANDLE(0, io_channels), adc_ch_cfg, &mv) < 0) {
        return -EIO;
    }

    // 中心値からの差分を計算
    int32_t rel_mv = mv - ch_data->mv_mid;
    if (ch_cfg->invert) rel_mv = -rel_mv;

    // デッドゾーン判定
    if (rel_mv > -ch_cfg->mv_deadzone && rel_mv < ch_cfg->mv_deadzone) {
        rel_mv = 0;
    } else {
        rel_mv = (rel_mv > 0) ? (rel_mv - ch_cfg->mv_deadzone) : (rel_mv + ch_cfg->mv_deadzone);
    }

    // スケーリング
    int16_t value = (rel_mv * ch_cfg->scale_multiplier) / ch_cfg->scale_divisor;

    if (ch_cfg->report_on_change_only && value == ch_data->last_value) {
        return 0;
    }

    ch_data->last_value = value;

    return zmk_input_report_event(ch_cfg->evt_type, ch_cfg->input_code, value, (ch_cfg->idx == 0));
}

static void analog_input_work_handler(struct k_work *work) {
    struct analog_input_data *data = CONTAINER_OF(work, struct analog_input_data, work);
    const struct device *dev = data->dev;
    const struct analog_input_config *config = dev->config;

    analog_input_process_channel(dev, &config->x_ch_cfg, &data->x_ch_data, &config->adc_ch_cfg[0]);
    analog_input_process_channel(dev, &config->y_ch_cfg, &data->y_ch_data, &config->adc_ch_cfg[1]);
}

static void analog_input_timer_handler(struct k_timer *timer) {
    struct analog_input_data *data = CONTAINER_OF(timer, struct analog_input_data, timer);
    k_work_submit(&data->work);
}

static int analog_input_init(const struct device *dev) {
    const struct analog_input_config *config = dev->config;
    struct analog_input_data *data = dev->data;

    data->dev = dev;

    if (!device_is_ready(config->adc_dev)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV;
    }

    // 各チャンネルの初期化
    for (int i = 0; i < 2; i++) {
        adc_channel_setup(config->adc_dev, &config->adc_ch_cfg[i]);
    }

    // Xチャンネルのキャリブレーション
    if (config->x_ch_cfg.mv_mid == 0) {
        data->x_ch_data.mv_mid = calibrate_channel(config->adc_dev, &config->adc_ch_cfg[0]);
        LOG_INF("X-Channel Auto-Calibrated to: %d mV", data->x_ch_data.mv_mid);
    } else {
        data->x_ch_data.mv_mid = config->x_ch_cfg.mv_mid;
    }

    // Yチャンネルのキャリブレーション
    if (config->y_ch_cfg.mv_mid == 0) {
        data->y_ch_data.mv_mid = calibrate_channel(config->adc_dev, &config->adc_ch_cfg[1]);
        LOG_INF("Y-Channel Auto-Calibrated to: %d mV", data->y_ch_data.mv_mid);
    } else {
        data->y_ch_data.mv_mid = config->y_ch_cfg.mv_mid;
    }

    k_work_init(&data->work, analog_input_work_handler);
    k_timer_init(&data->timer, analog_input_timer_handler, NULL);
    k_timer_start(&data->timer, K_MSEC(1000 / config->sampling_hz), K_MSEC(1000 / config->sampling_hz));

    return 0;
}

#define ANALOG_INPUT_CHANNEL_CFG(idx, ch_name) \
    .ch_name##_ch_cfg = { \
        .idx = idx, \
        .mv_mid = DT_INST_PROP_OR(0, ch_name##_ch_mv_mid, 0), \
        .mv_min_max = DT_INST_PROP(0, ch_name##_ch_mv_min_max), \
        .mv_deadzone = DT_INST_PROP(0, ch_name##_ch_mv_deadzone), \
        .scale_multiplier = DT_INST_PROP(0, ch_name##_ch_scale_multiplier), \
        .scale_divisor = DT_INST_PROP(0, ch_name##_ch_scale_divisor), \
        .invert = DT_INST_PROP_OR(0, ch_name##_ch_invert, false), \
        .evt_type = DT_INST_PROP(0, ch_name##_ch_evt_type), \
        .input_code = DT_INST_PROP(0, ch_name##_ch_input_code), \
        .report_on_change_only = DT_INST_NODE_HAS_PROP(0, ch_name##_ch_report_on_change_only), \
    }

static const struct analog_input_config analog_input_config_0 = {
    .adc_dev = DEVICE_DT_GET(DT_INST_IO_CHANNELS_CTLR(0)),
    .adc_ch_cfg = {
        {
            .gain = ADC_GAIN_1_6,
            .reference = ADC_REF_INTERNAL,
            .acquisition_time = ADC_ACQ_TIME_DEFAULT,
            .channel_id = DT_INST_IO_CHANNELS_INPUT_BY_IDX(0, 0),
        },
        {
            .gain = ADC_GAIN_1_6,
            .reference = ADC_REF_INTERNAL,
            .acquisition_time = ADC_ACQ_TIME_DEFAULT,
            .channel_id = DT_INST_IO_CHANNELS_INPUT_BY_IDX(0, 1),
        },
    },
    .sampling_hz = DT_INST_PROP(0, sampling_hz),
    ANALOG_INPUT_CHANNEL_CFG(0, x),
    ANALOG_INPUT_CHANNEL_CFG(1, y),
};

static struct analog_input_data analog_input_data_0;

DEVICE_DT_INST_DEFINE(0, analog_input_init, NULL, &analog_input_data_0, &analog_input_config_0, POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);
