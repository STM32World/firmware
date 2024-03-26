/**
 ******************************************************************************
 * @file           : littlefs.h
 * @brief          : Littlefs m24cxx driver header
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 Lars Boegild Thomsen <lth@stm32world.com>.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
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

lfs_ssize_t littlefs_size();
lfs_ssize_t littlefs_du();

#endif /* INC_LITTLEFS_H_ */
