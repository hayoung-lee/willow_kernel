
#ifndef __ASM_ARCH_GPIO_WILLOW_H
#define __ASM_ARCH_GPIO_WILLOW_H __FILE__

#include "gpio.h"

/* WILLOW KEY MAP */
#define WILLOW_POWER_KEY		EXYNOS4_GPX1(3)
#define WILLOW_VOLUM_UP			EXYNOS4_GPX3(3)
#define WILLOW_VOLUM_DOWN		EXYNOS4_GPX3(4)

/* DOCK DET */
#define DOCK_DET_N				EXYNOS4_GPX1(7)

/* BACKLIGHT */
#define BACKLIGHT_PWM_GPIO		EXYNOS4_GPD0(1)

#endif /* __ASM_ARCH_GPIO_WILLOW_H */