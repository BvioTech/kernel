#ifndef _SW42000_TOUCHSCREEN_H_
#define _SW42000_TOUCHSCREEN_H_

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>

#define SW42000_DRIVER_NAME "sw42000_touchscreen"
#define SW42000_DEVICE_NAME "sw42000_sub"

/* Register addresses - 统一使用16位地址 */
#define SW42000_REG_DEVICE_STS      0x0200  // 设备状态寄存器
#define SW42000_REG_TOUCH_COUNT     0x0201  // 触摸点数量
#define SW42000_REG_TOUCH_DATA      0x0202  // 触摸数据起始地址
#define SW42000_REG_DEVICE_INFO     0x0001  // 设备信息
#define SW42000_REG_POWER_MODE      0x0003  // 电源模式

/* Touch data structure */
#define SW42000_MAX_TOUCH_POINTS    10
#define SW42000_TOUCH_DATA_SIZE     6

/* Power modes */
#define SW42000_POWER_ACTIVE        0x00
#define SW42000_POWER_SLEEP         0x01
#define SW42000_POWER_DEEP_SLEEP    0x02

/* Touch point structure */
struct sw42000_touch_point {
    u8 id;
    u8 status;
    u16 x;
    u16 y;
    u8 pressure;
};

/* Platform data structure */
struct sw42000_platform_data {
    u32 irq_gpio;
    u32 reset_gpio;
    u32 max_x;
    u32 max_y;
    u32 max_pressure;
    u32 max_width_major;
    u32 max_width_minor;
    u32 max_orientation;
    u32 max_id;
    u32 hw_reset_delay;
    u32 sw_reset_delay;
    bool wakeup_source;
};

/* Main device structure */
struct sw42000_data {
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct sw42000_platform_data *pdata;
    struct workqueue_struct *workqueue;
    struct work_struct work;
    struct delayed_work polling_work;
    struct mutex lock;
    
    int irq;
    int irq_gpio;
    int reset_gpio;
    
    bool enabled;
    bool suspended;
    bool polling_mode;
    
    int touch_count;
    struct sw42000_touch_point touch_points[SW42000_MAX_TOUCH_POINTS];
};

/* Function declarations*/
int sw42000_i2c_read(struct sw42000_data *ts, u16 reg, u8 *data, int len);
int sw42000_i2c_write(struct sw42000_data *ts, u16 reg, u8 *data, int len);
int sw42000_i2c_read_simple(struct sw42000_data *ts, u8 reg, u8 *data, int len);
int sw42000_read_touch_data(struct sw42000_data *ts);
void sw42000_report_touch_data(struct sw42000_data *ts);
irqreturn_t sw42000_irq_handler(int irq, void *dev_id);
void sw42000_work_func(struct work_struct *work);
void sw42000_polling_work(struct work_struct *work);
int sw42000_hw_reset(struct sw42000_data *ts);
int sw42000_sw_reset(struct sw42000_data *ts);
int sw42000_set_power_mode(struct sw42000_data *ts, u8 mode);
struct sw42000_platform_data *sw42000_parse_dt(struct device *dev);

#endif