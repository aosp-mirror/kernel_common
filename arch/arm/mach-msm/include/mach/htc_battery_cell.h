/*
 * Copyright (C) 2007 HTC Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __HTC_BATTERY_CELL_H__
#define __HTC_BATTERY_CELL_H__

enum htc_battery_cell_type {
	HTC_BATTERY_CELL_TYPE_NORMAL,
	HTC_BATTERY_CELL_TYPE_HV,
};

#define HTC_BATTERY_CELL_ID_UNKNOWN	(255)
#define HTC_BATTERY_CELL_ID_DEVELOP	(254)

struct htc_battery_cell {
	const char *model_name;
	int	capacity;
	int	id;
	int	id_raw_min;
	int	id_raw_max;
	int	type;	/* HV or normal */
	const int	voltage_max;
	const int	voltage_min;
	void *chg_param;	/* charger client param */
	void *gauge_param;	/* gauge client param */
};


/* htc_battery_cell_init: init battery cells data
 * RETURN VALUE:
 * return 1 if input ary is NULL or ary_size is 0. It will use default
 * cell data instead. else return 0
 */
int htc_battery_cell_init(struct htc_battery_cell *ary, int ary_aize);

/* htc_battery_cell_find
 *  return idnetified battery cell ptr.
 */
struct htc_battery_cell *htc_battery_cell_find(int id_raw);

/* htc_battery_cell_find_and_set_id_auto
 *  set cur_cell as idnetified battery cell and return id.
 *  auto means: if id changes to a valid one, it will be set to
 *  cur_cell immediately. But if it changes to UNKNOWN id, it
 *  needs pass a successively checking cycle (ex: 3 times) to
 *  switch cur_cell to unknown.
 */
int htc_battery_cell_find_and_set_id_auto(int id_raw);

/* htc_battery_cell_get_cur_cell
 *  return current use battery cell ptr.
 */
struct htc_battery_cell *htc_battery_cell_get_cur_cell(void);

/* htc_battery_cell_set_cur_cell
 *  return current use battery cell ptr.
 */
struct htc_battery_cell *htc_battery_cell_set_cur_cell(
				struct htc_battery_cell *cell);

/* htc_battery_cell_set_cur_cell_by_id
 *  return 0: success, 1: fail
 */
int htc_battery_cell_set_cur_cell_by_id(int id);

/* htc_battery_cell_get_cur_cell_charger_cdata
 *  return current use battery cell chg_param ptr.
 */
void *htc_battery_cell_get_cur_cell_charger_cdata(void);

/* htc_battery_cell_get_cur_cell_guage_cdata
 *  return current use battery cell gauge_param ptr.
 */
void *htc_battery_cell_get_cur_cell_gauge_cdata(void);

/* htc_battery_cell_hv_authenticated
 *  return whether HV authentication pass or not.
 */
int htc_battery_cell_hv_authenticated(void);

/* htc_battery_cell_register_change_notify
 */
/* int htc_battery_cell_register_change_notify(void); */
#endif
