/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018 Xilnx, Inc
 */

#ifndef __XLNX_LOGS_H__
#define __XLNX_LOGS_H__

#define PMD_INIT_LOG(level, fmt, args...) \
	rte_log(RTE_LOG_ ## level, xlnx_net_logtype_init, \
			"%s(): " fmt "\n", __func__, ## args)

#define PMD_INIT_FUNC_TRACE() PMD_INIT_LOG(DEBUG, ">>")

#define PMD_DRV_LOG(level, fmt, args...) \
	rte_log(RTE_LOG_ ## level, xlnx_net_logtype_driver, \
			"%s(): " fmt "\n", __func__, ## args)

#define xlnx_log_err(s, ...) PMD_INIT_LOG(ERR, s, ##__VA_ARGS__)
#define xlnx_log_info(s, ...) PMD_INIT_LOG(INFO, s, ##__VA_ARGS__)
#define xlnx_log_dbg(s, ...) PMD_DRV_LOG(DEBUG, s, ##__VA_ARGS__)

extern int xlnx_net_logtype_init;
extern int xlnx_net_logtype_driver;

#endif /* __XLNX_LOGS_H__*/
