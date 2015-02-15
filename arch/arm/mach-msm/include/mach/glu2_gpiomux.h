/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __GLU2_GPIOMUX_H_
#define __GLU2_GPIOMUX_H_

enum board_glu {
        GLU_WLU = 256,
        GLU_U = 264,

        GLU2_U = 282,
        GLU2_UL,
        GLU2_WLJ,
};

void __init glu2_init_gpiomux(void);
#endif
