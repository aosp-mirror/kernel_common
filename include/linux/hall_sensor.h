/*for Hall sensor common header file*/
#ifndef __LINUX_HALL_SENSOR_H
#define __LINUX_HALL_SENSOR_H

extern struct blocking_notifier_head hallsensor_notifier_list;

extern int register_notifier_by_hallsensor(struct notifier_block *nb);
extern int unregister_notifier_by_hallsensor(struct notifier_block *nb);
int hallsensor_enable_by_hover_driver(int on);
#endif

