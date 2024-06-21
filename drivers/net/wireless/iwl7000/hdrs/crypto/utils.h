/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Cryptographic utilities
 *
 * Copyright (c) 2023 Herbert Xu <herbert@gondor.apana.org.au>
 */
#ifndef _BACKPORT_CRYPTO_UTILS_H
#define _BACKPORT_CRYPTO_UTILS_H

#if LINUX_VERSION_IS_GEQ(6,4,0)
#include_next <crypto/utils.h>
#else
#include <crypto/algapi.h>
#endif /* < 6.4.0 */

#endif	/* _BACKPORT_CRYPTO_UTILS_H */
