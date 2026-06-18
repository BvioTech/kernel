#include "sw42000_touchscreen.h"
#include <linux/stdarg.h>

/* 调试开关 */
#define SW42000_DEBUG 0

/* SW42000协议定义 */
#define SW42000_I2C_ADDR            0x28
#define SW42000_MAX_I2C_BUFFER      256

#pragma pack(1)
struct sw42000_touch_data {
    u32 track_id : 5;
    u32 tool_type : 3;
    u32 angle : 8;
    u32 event : 2;
    u32 x : 14;
    u32 y : 14;
    u32 pressure : 8;
    u32 reserve1 : 10;
    u32 reserve2 : 4;
    u32 width_major : 14;
    u32 width_minor : 14;
};

struct sw42000_touch_info {
    u32 ic_status;
    u32 tc_status;
    u32 wakeup_type : 8;
    u32 touch_cnt : 5;
    u32 button_cnt : 3;
    u32 self_recal : 2;
    u32 mutual_recal : 2;
    u32 abnormal1_recal : 2;
    u32 abnormal2_recal : 2;
    u32 current_mode : 1;
    u32 rmdiff : 7;
    struct sw42000_touch_data data[10];
    /* debug info */
};
#pragma pack()

/* 触摸数据读取命令*/
static u8 cmd_get_irq_info[] = { 0x26, 0x00 };

/* 调试宏 */
#if SW42000_DEBUG
#define sw42000_dbg(dev, fmt, ...) dev_info(dev, "[DEBUG] " fmt, ##__VA_ARGS__)
#else
#define sw42000_dbg(dev, fmt, ...) do { } while (0)
#endif

/* I2C通信函数 */
static int sw42000_i2c_write_reg(struct sw42000_data *ts, u8 *data, int len)
{
    struct i2c_msg msg = {
        .addr = ts->client->addr,
        .flags = 0,
        .len = len,
        .buf = data,
    };

    sw42000_dbg(&ts->client->dev, "I2C Write: len=%d, data=%*ph\n", len, min(len, 8), data);

    int ret = i2c_transfer(ts->client->adapter, &msg, 1);
    if (ret != 1) {
        dev_err(&ts->client->dev, "SW42000 write failed: len=%d, ret=%d\n", len, ret);
        return ret < 0 ? ret : -EIO;
    }

    return 0;
}

static int sw42000_i2c_write_read(struct sw42000_data *ts, u8 *write_data, int write_len, 
                                  u8 *read_data, int read_len)
{
    struct i2c_msg msgs[] = {
        {
            .addr = ts->client->addr,
            .flags = 0,
            .len = write_len,
            .buf = write_data,
        },
        {
            .addr = ts->client->addr,
            .flags = I2C_M_RD,
            .len = read_len,
            .buf = read_data,
        },
    };

    int ret = i2c_transfer(ts->client->adapter, msgs, 2);
    
    sw42000_dbg(&ts->client->dev, "I2C WriteRead: write_len=%d, read_len=%d, ret=%d\n", 
                write_len, read_len, ret);
    
    if (ret != 2) {
        dev_err(&ts->client->dev, "SW42000 write-read failed: ret=%d\n", ret);
        return ret < 0 ? ret : -EIO;
    }

    return 0;
}

/* I2C写入 */
static int WA(struct sw42000_data *ts, u8 arg1, ...)
{
    u8 cmd[6];
    va_list args;
    int i = 0;
    
    va_start(args, arg1);
    cmd[i++] = arg1;
    
    /* 根据第一个参数决定有多少个参数 */
    int param_count = (arg1 == 0x40 || arg1 == 0x4f || arg1 == 0x4c) ? 5 : 1;
    
    for (int j = 1; j < param_count + 1 && i < 6; j++) {
        cmd[i++] = (u8)va_arg(args, int);
    }
    va_end(args);
    
    return sw42000_i2c_write_reg(ts, cmd, i);
}

/* 完整初始化序列 */
static int sw42000_device_init(struct sw42000_data *ts)
{
    int ret;

    dev_info(&ts->client->dev, "Initializing SW42000 with Windows driver sequence\n");

    /* 硬件复位 */
    sw42000_dbg(&ts->client->dev, "Performing hardware reset...\n");
    gpio_set_value(ts->reset_gpio, 0);
    msleep(50);
    gpio_set_value(ts->reset_gpio, 1);
    msleep(200);

    /* 初始化序列 */
    ret = WA(ts, 0x40, 0x1c, 0x5a, 0x5a, 0x5a, 0x5a); if (ret < 0) return ret;
    ret = WA(ts, 0x00, 0x1c); if (ret < 0) return ret;
    ret = WA(ts, 0x40, 0x1c, 0xa5, 0xa5, 0xa5, 0xa5); if (ret < 0) return ret;
    ret = WA(ts, 0x00, 0x1c); if (ret < 0) return ret;
    ret = WA(ts, 0x40, 0x1c, 0xf0, 0xf0, 0xf0, 0xf0); if (ret < 0) return ret;
    ret = WA(ts, 0x00, 0x1c); if (ret < 0) return ret;
    ret = WA(ts, 0x40, 0x1c, 0x0f, 0x0f, 0x0f, 0x0f); if (ret < 0) return ret;
    ret = WA(ts, 0x00, 0x1c); if (ret < 0) return ret;
    ret = WA(ts, 0x40, 0x1c, 0x00, 0xff, 0x00, 0xff); if (ret < 0) return ret;
    ret = WA(ts, 0x00, 0x1c); if (ret < 0) return ret;
    ret = WA(ts, 0x40, 0x1c, 0xff, 0x00, 0xff, 0x00); if (ret < 0) return ret;
    ret = WA(ts, 0x00, 0x1c); if (ret < 0) return ret;
    ret = WA(ts, 0x40, 0x1c, 0x00, 0x00, 0xff, 0xff); if (ret < 0) return ret;
    ret = WA(ts, 0x00, 0x1c); if (ret < 0) return ret;
    ret = WA(ts, 0x40, 0x1c, 0xff, 0xff, 0x00, 0x00); if (ret < 0) return ret;
    ret = WA(ts, 0x00, 0x1c); if (ret < 0) return ret;
    ret = WA(ts, 0x40, 0x1c, 0xff, 0xff, 0xff, 0xff); if (ret < 0) return ret;
    ret = WA(ts, 0x00, 0x1c); if (ret < 0) return ret;
    ret = WA(ts, 0x40, 0x1c, 0x00, 0x00, 0x00, 0x00); if (ret < 0) return ret;
    ret = WA(ts, 0x00, 0x1c); if (ret < 0) return ret;
    ret = WA(ts, 0x06, 0x42); if (ret < 0) return ret;
    ret = WA(ts, 0x26, 0x44); if (ret < 0) return ret;
    ret = WA(ts, 0x26, 0x5b); if (ret < 0) return ret;
    ret = WA(ts, 0x00, 0x5d); if (ret < 0) return ret;
    ret = WA(ts, 0x00, 0x5e); if (ret < 0) return ret;
    ret = WA(ts, 0x00, 0x1b); if (ret < 0) return ret;
    ret = WA(ts, 0x4f, 0xe4, 0x00, 0x00, 0x00, 0x00); if (ret < 0) return ret;
    ret = WA(ts, 0x4c, 0x00, 0x01, 0x00, 0x00, 0x00); if (ret < 0) return ret;
    ret = WA(ts, 0x4c, 0x01, 0x01, 0x00, 0x00, 0x00); if (ret < 0) return ret;
    ret = WA(ts, 0x4f, 0x6a, 0x01, 0x00, 0x00, 0x00); if (ret < 0) return ret;
    ret = WA(ts, 0x4f, 0x6d, 0x00, 0x00, 0x00, 0x00); if (ret < 0) return ret;
    ret = WA(ts, 0x4f, 0x6f, 0x00, 0x00, 0x00, 0x00); if (ret < 0) return ret;
    ret = WA(ts, 0x4f, 0x67, 0x00, 0x00, 0x00, 0x00); if (ret < 0) return ret;
    ret = WA(ts, 0x4f, 0x30, 0x00, 0x00, 0x00, 0x00); if (ret < 0) return ret;
    ret = WA(ts, 0x4c, 0x03, 0x11, 0x03, 0x00, 0x00); if (ret < 0) return ret;
    ret = WA(ts, 0x06, 0x00); if (ret < 0) return ret;
    ret = WA(ts, 0x06, 0x01); if (ret < 0) return ret;
    ret = WA(ts, 0x0c, 0x00); if (ret < 0) return ret;
    ret = WA(ts, 0x4f, 0x30, 0x00, 0x00, 0x00, 0x00); if (ret < 0) return ret;
    ret = WA(ts, 0x4c, 0x03, 0x11, 0x03, 0x00, 0x00); if (ret < 0) return ret;
    ret = WA(ts, 0x06, 0x00); if (ret < 0) return ret;
    ret = WA(ts, 0x06, 0x01); if (ret < 0) return ret;
    ret = WA(ts, 0x0c, 0x00); if (ret < 0) return ret;
    ret = WA(ts, 0x4f, 0x6f, 0x01, 0x00, 0x00, 0x00); if (ret < 0) return ret;

    dev_info(&ts->client->dev, "SW42000 initialization completed successfully\n");
    return 0;
}

/* 触摸数据读取 */
int sw42000_read_touch_data(struct sw42000_data *ts)
{
    struct sw42000_touch_info touch_info;
    int ret, i;
    int x, y;

    /* 读取触摸数据 */
    ret = sw42000_i2c_write_read(ts, cmd_get_irq_info, sizeof(cmd_get_irq_info),
                                 (u8*)&touch_info, sizeof(touch_info));
    if (ret < 0) {
        dev_err(&ts->client->dev, "Failed to read touch data: %d\n", ret);
        return ret;
    }

    sw42000_dbg(&ts->client->dev, "Touch count: %d\n", touch_info.touch_cnt);
    sw42000_dbg(&ts->client->dev, "IC status: 0x%08x, TC status: 0x%08x\n", 
                touch_info.ic_status, touch_info.tc_status);

    /* 清除之前的触摸状态 */
    ts->touch_count = 0;
    for (i = 0; i < SW42000_MAX_TOUCH_POINTS; i++) {
        ts->touch_points[i].status = 0;
    }

    /* 解析触摸点数据 */
    ts->touch_count = touch_info.touch_cnt;
    
    if (ts->touch_count > ts->pdata->max_id) {
        sw42000_dbg(&ts->client->dev, "Invalid touch count: %d\n", ts->touch_count);
        ts->touch_count = 0;
        return 0;
    }

    for (i = 0; i < ts->touch_count && i < ts->pdata->max_id; i++) {
        struct sw42000_touch_data *touch_data = &touch_info.data[i];
        
        x = touch_data->x;
        y = touch_data->y;

        /* 坐标范围检查 */
        if (x > ts->pdata->max_x || y > ts->pdata->max_y) {
            dev_warn(&ts->client->dev, "Invalid coordinates: x=%d, y=%d (max_x=%d, max_y=%d)\n", 
                     x, y, ts->pdata->max_x, ts->pdata->max_y);
            continue;
        }

        /* 设置触摸点数据 */
        ts->touch_points[i].id = touch_data->track_id;
        ts->touch_points[i].x = x;
        ts->touch_points[i].y = y;
        ts->touch_points[i].pressure = touch_data->pressure ? touch_data->pressure : 100;
        
        /* 事件类型3表示离开，其他表示触摸 */
        if (touch_data->event == 3) {
            ts->touch_points[i].status = 0; // 离开
            // dev_info(&ts->client->dev, "Touch %d: RELEASE id=%d, x=%d, y=%d\n",
            //          i, touch_data->track_id, x, y);
        } else {
            ts->touch_points[i].status = 1; // 触摸
            // dev_info(&ts->client->dev, "Touch %d: PRESS id=%d, x=%d, y=%d, pressure=%d, event=%d\n",
            //          i, touch_data->track_id, x, y, touch_data->pressure, touch_data->event);
        }
    }

    return 0;
}

/* 完整的多点触摸数据报告函数 - 支持十指触控 */
void sw42000_report_touch_data(struct sw42000_data *ts)
{
    int i;
    bool any_active_touch = false;
    static bool last_any_touch = false;

    /* 统计活跃触摸点 */
    for (i = 0; i < ts->touch_count; i++) {
        if (ts->touch_points[i].status == 1) {
            any_active_touch = true;
            break;
        }
    }

    /* 报告所有触摸点的多点触摸事件 */
    for (i = 0; i < SW42000_MAX_TOUCH_POINTS; i++) {
        input_mt_slot(ts->input_dev, i);
        
        if (i < ts->touch_count && ts->touch_points[i].status == 1) {
            /* 触摸按下或移动 */
            struct sw42000_touch_point *point = &ts->touch_points[i];
            
            input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
            input_report_abs(ts->input_dev, ABS_MT_POSITION_X, point->x);
            input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, point->y);
            input_report_abs(ts->input_dev, ABS_MT_PRESSURE, point->pressure);
            input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, point->id);
            
            // dev_info(&ts->client->dev, "MT Touch: slot=%d, id=%d, x=%d, y=%d, pressure=%d\n",
            //          i, point->id, point->x, point->y, point->pressure);
        } else {
            /* 触摸释放 */
            input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
        }
    }

    /* 报告触摸状态变化以控制鼠标指针显示/隐藏 */
    if (any_active_touch != last_any_touch) {
        if (any_active_touch) {
            /* 有触摸时发送特殊事件隐藏鼠标指针 */
            input_report_key(ts->input_dev, BTN_TOOL_FINGER, 1);
            // dev_info(&ts->client->dev, "Multi-touch: TOUCH START - Hide cursor\n");
        } else {
            /* 无触摸时发送特殊事件显示鼠标指针 */
            input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
            // dev_info(&ts->client->dev, "Multi-touch: TOUCH END - Show cursor\n");
        }
        last_any_touch = any_active_touch;
    }

    /* 同步多点触摸帧 */
    input_mt_sync_frame(ts->input_dev);
    
    /* 同步所有事件 */
    input_sync(ts->input_dev);
}

/* 兼容性函数 */
int sw42000_i2c_read(struct sw42000_data *ts, u16 reg, u8 *data, int len)
{
    u8 reg_buf[2] = { (u8)(reg >> 8), (u8)(reg & 0xFF) };
    return sw42000_i2c_write_read(ts, reg_buf, 2, data, len);
}

int sw42000_i2c_write(struct sw42000_data *ts, u16 reg, u8 *data, int len)
{
    u8 *buf;
    int ret;
    
    buf = kzalloc(len + 2, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;
        
    buf[0] = (u8)(reg >> 8);
    buf[1] = (u8)(reg & 0xFF);
    memcpy(&buf[2], data, len);
    
    ret = sw42000_i2c_write_reg(ts, buf, len + 2);
    
    kfree(buf);
    return ret;
}

int sw42000_i2c_read_simple(struct sw42000_data *ts, u8 reg, u8 *data, int len)
{
    u8 reg_buf[1] = { reg };
    return sw42000_i2c_write_read(ts, reg_buf, 1, data, len);
}

irqreturn_t sw42000_irq_handler(int irq, void *dev_id)
{
    struct sw42000_data *ts = dev_id;
    
    sw42000_dbg(&ts->client->dev, "IRQ triggered: %d\n", irq);

    if (!ts->enabled)
        return IRQ_HANDLED;

    queue_work(ts->workqueue, &ts->work);
    return IRQ_HANDLED;
}

void sw42000_work_func(struct work_struct *work)
{
    struct sw42000_data *ts = container_of(work, struct sw42000_data, work);
    int ret;

    mutex_lock(&ts->lock);

    if (!ts->enabled) {
        mutex_unlock(&ts->lock);
        return;
    }

    ret = sw42000_read_touch_data(ts);
    if (ret < 0) {
        dev_err(&ts->client->dev, "Failed to read touch data: %d\n", ret);
        goto out;
    }

    sw42000_report_touch_data(ts);

out:
    mutex_unlock(&ts->lock);
}

void sw42000_polling_work(struct work_struct *work)
{
    struct sw42000_data *ts = container_of(work, struct sw42000_data, polling_work.work);
    int ret;

    if (!ts->enabled)
        return;

    mutex_lock(&ts->lock);
    
    ret = sw42000_read_touch_data(ts);
    if (ret >= 0) {
        sw42000_report_touch_data(ts);
    }
    
    mutex_unlock(&ts->lock);

    if (ts->enabled && ts->polling_mode)
        schedule_delayed_work(&ts->polling_work, msecs_to_jiffies(16)); // 60fps
}

int sw42000_hw_reset(struct sw42000_data *ts)
{
    if (!gpio_is_valid(ts->reset_gpio))
        return -EINVAL;

    dev_info(&ts->client->dev, "Performing hardware reset\n");
    
    gpio_set_value(ts->reset_gpio, 0);
    msleep(ts->pdata->hw_reset_delay);
    gpio_set_value(ts->reset_gpio, 1);
    msleep(ts->pdata->hw_reset_delay * 2);

    return 0;
}

int sw42000_sw_reset(struct sw42000_data *ts)
{
    return sw42000_device_init(ts);
}

int sw42000_set_power_mode(struct sw42000_data *ts, u8 mode)
{
    /* 电源模式控制 - 可根据需要实现具体命令 */
    return 0;
}

struct sw42000_platform_data *sw42000_parse_dt(struct device *dev)
{
    struct sw42000_platform_data *pdata;
    struct device_node *np = dev->of_node;
    int ret;

    pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
    if (!pdata)
        return ERR_PTR(-ENOMEM);

    pdata->reset_gpio = of_get_named_gpio(np, "reset-gpios", 0);
    if (!gpio_is_valid(pdata->reset_gpio)) {
        dev_err(dev, "Invalid reset GPIO\n");
        return ERR_PTR(-EINVAL);
    }

    pdata->irq_gpio = of_get_named_gpio(np, "irq-gpios", 0);
    if (!gpio_is_valid(pdata->irq_gpio)) {
        dev_err(dev, "Invalid IRQ GPIO\n");
        return ERR_PTR(-EINVAL);
    }

    ret = of_property_read_u32(np, "max_x", &pdata->max_x);
    if (ret) pdata->max_x = 1080;

    ret = of_property_read_u32(np, "max_y", &pdata->max_y);
    if (ret) pdata->max_y = 1240;

    ret = of_property_read_u32(np, "max_pressure", &pdata->max_pressure);
    if (ret) pdata->max_pressure = 255;

    ret = of_property_read_u32(np, "max_width_major", &pdata->max_width_major);
    if (ret) pdata->max_width_major = 1080;

    ret = of_property_read_u32(np, "max_width_minor", &pdata->max_width_minor);
    if (ret) pdata->max_width_minor = 1240;

    ret = of_property_read_u32(np, "max_orientation", &pdata->max_orientation);
    if (ret) pdata->max_orientation = 90;

    ret = of_property_read_u32(np, "max_id", &pdata->max_id);
    if (ret) pdata->max_id = 10;

    ret = of_property_read_u32(np, "hw_reset_delay", &pdata->hw_reset_delay);
    if (ret) pdata->hw_reset_delay = 40;

    ret = of_property_read_u32(np, "sw_reset_delay", &pdata->sw_reset_delay);
    if (ret) pdata->sw_reset_delay = 40;

    pdata->wakeup_source = of_property_read_bool(np, "wakeup-source");

    return pdata;
}

static int sw42000_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
    struct sw42000_data *ts;
    struct sw42000_platform_data *pdata;
    int ret;

    dev_info(&client->dev, "SW42000 touchscreen driver probing (Multi-Touch v7)\n");

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "I2C functionality not supported\n");
        return -EIO;
    }

    pdata = sw42000_parse_dt(&client->dev);
    if (IS_ERR(pdata))
        return PTR_ERR(pdata);

    ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
    if (!ts)
        return -ENOMEM;

    ts->client = client;
    ts->pdata = pdata;
    ts->reset_gpio = pdata->reset_gpio;
    ts->irq_gpio = pdata->irq_gpio;
    ts->polling_mode = false;

    mutex_init(&ts->lock);
    i2c_set_clientdata(client, ts);

    /* Request GPIOs */
    ret = devm_gpio_request_one(&client->dev, ts->reset_gpio,
                               GPIOF_OUT_INIT_LOW, "sw42000-reset");
    if (ret) {
        dev_err(&client->dev, "Failed to request reset GPIO: %d\n", ret);
        return ret;
    }

    ret = devm_gpio_request_one(&client->dev, ts->irq_gpio,
                               GPIOF_IN, "sw42000-irq");
    if (ret) {
        dev_err(&client->dev, "Failed to request IRQ GPIO: %d\n", ret);
        return ret;
    }

    /* Hardware reset */
    ret = sw42000_hw_reset(ts);
    if (ret) {
        dev_err(&client->dev, "Hardware reset failed: %d\n", ret);
        return ret;
    }

    /* 序列初始化设备 */
    ret = sw42000_device_init(ts);
    if (ret) {
        dev_err(&client->dev, "Device initialization failed: %d\n", ret);
        return ret;
    }

    /* Create input device - 专业多点触摸屏设备 */
    ts->input_dev = devm_input_allocate_device(&client->dev);
    if (!ts->input_dev) {
        dev_err(&client->dev, "Failed to allocate input device\n");
        return -ENOMEM;
    }

    ts->input_dev->name = "SW42000 Multi-Touch Touchscreen";
    ts->input_dev->id.bustype = BUS_I2C;
    ts->input_dev->id.vendor = 0x1004;   // LG vendor ID
    ts->input_dev->id.product = 0x4200; // SW42000 product ID
    ts->input_dev->id.version = 0x0700;  // v7.0 - Multi-Touch Mode
    ts->input_dev->dev.parent = &client->dev;

    /* 设置设备类型 - 触摸屏设备 */
    __set_bit(EV_ABS, ts->input_dev->evbit);
    __set_bit(EV_KEY, ts->input_dev->evbit);
    __set_bit(EV_SYN, ts->input_dev->evbit);

    /* 设置触摸屏按键 - 用于指示触摸状态 */
    __set_bit(BTN_TOOL_FINGER, ts->input_dev->keybit);
    __set_bit(BTN_TOUCH, ts->input_dev->keybit);

    /* 设置触摸屏设备属性 */
    __set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

    /* 设置多点触摸参数 - 支持十指触控 */
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, pdata->max_x, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, pdata->max_y, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, pdata->max_pressure, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, pdata->max_id - 1, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, pdata->max_width_major, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MINOR, 0, pdata->max_width_minor, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_ORIENTATION, 0, pdata->max_orientation, 0, 0);

    /* 初始化多点触摸 - 支持十指且优化性能 */
    ret = input_mt_init_slots(ts->input_dev, pdata->max_id, 
                             INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED | INPUT_MT_TRACK);
    if (ret) {
        dev_err(&ts->client->dev, "Failed to initialize MT slots: %d\n", ret);
        return ret;
    }

    /* 注册输入设备 */
    ret = input_register_device(ts->input_dev);
    if (ret) {
        dev_err(&ts->client->dev, "Failed to register input device: %d\n", ret);
        return ret;
    }

    /* Create workqueue */
    ts->workqueue = create_singlethread_workqueue("sw42000_wq");
    if (!ts->workqueue) {
        dev_err(&ts->client->dev, "Failed to create workqueue\n");
        return -ENOMEM;
    }

    INIT_WORK(&ts->work, sw42000_work_func);
    INIT_DELAYED_WORK(&ts->polling_work, sw42000_polling_work);

    /* Setup IRQ */
    ts->irq = gpio_to_irq(ts->irq_gpio);
    if (ts->irq >= 0) {
        ret = devm_request_threaded_irq(&client->dev, ts->irq, NULL,
                                       sw42000_irq_handler,
                                       IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                       SW42000_DRIVER_NAME, ts);
        if (ret) {
            dev_warn(&client->dev, "Failed to request IRQ %d: %d, enabling polling mode\n", ts->irq, ret);
            ts->irq = -1;
            ts->polling_mode = true;
            schedule_delayed_work(&ts->polling_work, msecs_to_jiffies(100));
        } else {
            dev_info(&client->dev, "IRQ %d registered successfully\n", ts->irq);
        }
    } else {
        dev_info(&client->dev, "No valid IRQ, using polling mode\n");
        ts->polling_mode = true;
        schedule_delayed_work(&ts->polling_work, msecs_to_jiffies(100));
    }

    ts->enabled = true;

    if (pdata->wakeup_source)
        device_init_wakeup(&client->dev, true);

    dev_info(&client->dev, "SW42000 multi-touch touchscreen driver loaded successfully\n");
    return 0;
}

static void sw42000_remove(struct i2c_client *client)
{
    struct sw42000_data *ts = i2c_get_clientdata(client);

    ts->enabled = false;
    ts->polling_mode = false;
    
    if (ts->workqueue) {
        cancel_work_sync(&ts->work);
        cancel_delayed_work_sync(&ts->polling_work);
        destroy_workqueue(ts->workqueue);
    }

    device_init_wakeup(&client->dev, false);
    dev_info(&client->dev, "SW42000 touchscreen driver removed\n");
}

static int sw42000_suspend(struct device *dev)
{
    struct sw42000_data *ts = dev_get_drvdata(dev);

    mutex_lock(&ts->lock);
    
    if (ts->enabled) {
        if (ts->irq >= 0)
            disable_irq(ts->irq);
        ts->enabled = false;
        if (ts->polling_mode)
            cancel_delayed_work_sync(&ts->polling_work);
    }
    
    ts->suspended = true;
    mutex_unlock(&ts->lock);

    return 0;
}

static int sw42000_resume(struct device *dev)
{
    struct sw42000_data *ts = dev_get_drvdata(dev);

    mutex_lock(&ts->lock);
    
    if (ts->suspended) {
        sw42000_hw_reset(ts);
        sw42000_device_init(ts);
        ts->enabled = true;
        ts->suspended = false;
        if (ts->irq >= 0)
            enable_irq(ts->irq);
        if (ts->polling_mode)
            schedule_delayed_work(&ts->polling_work, msecs_to_jiffies(100));
    }
    
    mutex_unlock(&ts->lock);

    return 0;
}

static const struct dev_pm_ops sw42000_pm_ops = {
    .suspend = sw42000_suspend,
    .resume = sw42000_resume,
};

static const struct i2c_device_id sw42000_id[] = {
    { SW42000_DRIVER_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, sw42000_id);

static const struct of_device_id sw42000_of_match[] = {
    { .compatible = "lge_sub,sw42000_sub" },
    { }
};
MODULE_DEVICE_TABLE(of, sw42000_of_match);

static struct i2c_driver sw42000_driver = {
    .driver = {
        .name = SW42000_DRIVER_NAME,
        .of_match_table = sw42000_of_match,
        .pm = &sw42000_pm_ops,
    },
    .probe = sw42000_probe,
    .remove = sw42000_remove,
    .id_table = sw42000_id,
};

module_i2c_driver(sw42000_driver);

MODULE_AUTHOR("Rebecca Team");
MODULE_DESCRIPTION("SW42000 Multi-Touch Touchscreen Driver for Raspberry Pi 5");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("7.0");