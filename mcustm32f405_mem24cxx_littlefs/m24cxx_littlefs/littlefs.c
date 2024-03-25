/*
 * littlefs.c
 *
 *  Created on: Mar 3, 2022
 *      Author: lth
 */

#include "main.h"
#include "stdio.h"
#include "string.h"
//#include "fatfs.h"
//#include "fs.h"
#include "lfs.h"
#include "m24cxx.h"
#include "littlefs.h"

#define LFS_ENC 1

int littlefs_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
int littlefs_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
int littlefs_erase(const struct lfs_config *c, lfs_block_t block);
int littlefs_sync(const struct lfs_config *c);

const struct lfs_config littlefs_config = {
//struct lfs_config littlefs_config = {
    // block device operations
    .read  = littlefs_read,
    .prog  = littlefs_prog,
    .erase = littlefs_erase,
    .sync  = littlefs_sync,

    // block device configuration
    .read_size = 16,
    .prog_size = 16,
    .block_size = 256,
    .block_count = 4 * 512,
    .cache_size = 16,
    .lookahead_size = 16,
    .block_cycles = 100,
};

uint8_t *encryption_key = NULL;

M24CXX_HandleTypeDef *m24cxx_handle = NULL;

// variables used by the filesystem
lfs_t littlefs;
//lfs_file_t file;

int littlefs_init(M24CXX_HandleTypeDef *m24cxx_init, void *key) {
    LFS_DBG("LittleFS Init");
    m24cxx_handle = m24cxx_init;

    if (key) encryption_key = key;

//    LFS_DBG("Erasing");
//    for (uint32_t i = 0; i < littlefs_config.block_count; ++i) {
//        littlefs_erase(&littlefs_config, i);
//    }

    int err = lfs_mount(&littlefs, &littlefs_config);

    //err = 1;

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err) {
        lfs_format(&littlefs, &littlefs_config);
        lfs_mount(&littlefs, &littlefs_config);
    }

    //fs_init(&littlefs_config);

    return 0;

}

int littlefs_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) {
    LFS_DBG("LittleFS Read b = 0x%04lx o = 0x%04lx s = 0x%04lx", block, off, size);

    uint8_t buf[size];

    if (m24cxx_read(m24cxx_handle, block * littlefs_config.block_size + off, (void *)&buf, size) != M24CXX_Ok)
        return -1;

    for (uint32_t i = 0; i < size; ++i) {
        ((uint8_t *)buffer)[i] = encryption_key ? buf[i] ^ encryption_key[off + i] : buf[i];
    }

    return 0;
}

int littlefs_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {
    LFS_DBG("LittleFS Prog b = 0x%04lx o = 0x%04lx s = 0x%04lx", block, off, size);

    uint8_t buf[size];

    for (uint32_t i = 0; i < size; ++i) {
        buf[i] = encryption_key ? ((uint8_t *)buffer)[i] ^ encryption_key[off + i] : ((uint8_t *)buffer)[i];
    }

    if (m24cxx_write(m24cxx_handle, block * littlefs_config.block_size + off, (void *)&buf, size) != M24CXX_Ok)
        return -1;

    return 0;
}

int littlefs_erase(const struct lfs_config *c, lfs_block_t block) {
    LFS_DBG("LittleFS Erase b = 0x%04lx", block);

    uint8_t buf[littlefs_config.block_size];

    memset(buf, 0xff, sizeof(buf));

    if (encryption_key) {
        for (uint32_t i = 0; i < littlefs_config.block_size; ++i) {
            buf[i] = buf[i] ^ encryption_key[i];
        }
    }

    if (m24cxx_write(m24cxx_handle, block * littlefs_config.block_size, (void *)&buf, littlefs_config.block_size) != M24CXX_Ok)
        return -1;

    return 0;
}

int littlefs_sync(const struct lfs_config *c) {
    LFS_DBG("LittleFS Sync");
    return 0;
}

lfs_ssize_t littlefs_size() {
    return littlefs.cfg->block_size * littlefs.cfg->block_count;
    //return c->block_size * c->block_count;
}

lfs_ssize_t littlefs_du() {
    return lfs_fs_size(&littlefs) * littlefs.cfg->block_size;
}
