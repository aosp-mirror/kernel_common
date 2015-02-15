#if !defined(HTCPLATFORM_H)
#define HTC_PLATFORM_H
#include <mach/board.h>

void si_wakeup_mhl(void);
void si_d2_to_d3(void);
#ifdef CONFIG_HTC_MHL_DETECTION
int mhl_detect_register_notifier(struct t_mhl_status_notifier *notifier);
#endif

#endif /* if !defined(HTC_PLATFORM_H) */
