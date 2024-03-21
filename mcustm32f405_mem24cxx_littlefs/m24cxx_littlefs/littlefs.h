/*
 * littlefs.h
 *
 *  Created on: Mar 3, 2022
 *      Author: lth
 */

#ifndef INC_LITTLEFS_H_
#define INC_LITTLEFS_H_
#include "lfs.h"

#ifdef DEBUG
#define LFS_DBG(...)    fprintf(stderr, __VA_ARGS__);
#else
#define LFS_DBG(...)
#endif

extern lfs_t littlefs;

int littlefs_init(M24CXX_HandleTypeDef *m24cxx_init, void *key);

lfs_ssize_t littlefs_size(const struct lfs_config *c);
lfs_ssize_t littlefs_du(const struct lfs_config *c);

#endif /* INC_LITTLEFS_H_ */
