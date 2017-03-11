/**
 * @file spiffs.c
 *
 * @date Dec 13, 2016
 * @author jupiter
 * @description
 */


#include <spiffs.h>

static spiffs fs;

#define LOG_PAGE_SIZE       256

static u8_t spiffs_work_buf[LOG_PAGE_SIZE*2];
static u8_t spiffs_fds[32*4];
static u8_t spiffs_cache_buf[(LOG_PAGE_SIZE+32)*4];


void my_spi_read(int addr, int size, char *buf)
{

}
void my_spi_write(int addr, int size, char *buf)
{

}
void my_spi_erase(int addr, int size)
{

}


void my_spiffs_mount() {
    spiffs_config cfg;
    cfg.phys_size = 2*1024*1024; // use all spi flash
    cfg.phys_addr = 0; // start spiffs at start of spi flash
    cfg.phys_erase_block = 65536; // according to datasheet
    cfg.log_block_size = 65536; // let us not complicate things
    cfg.log_page_size = LOG_PAGE_SIZE; // as we said

    cfg.hal_read_f = my_spi_read;
    cfg.hal_write_f = my_spi_write;
    cfg.hal_erase_f = my_spi_erase;


    int res = SPIFFS_mount(&fs,
      &cfg,
      spiffs_work_buf,
      spiffs_fds,
      sizeof(spiffs_fds),
      spiffs_cache_buf,
      sizeof(spiffs_cache_buf),
      0);
    printf("mount res: %i\n", res);
  }


static s32_t my_spiffs_read(u32_t addr, u32_t size, u8_t *dst) {
    my_spi_read(addr, size, dst);
    return SPIFFS_OK;
  }

  static s32_t my_spiffs_write(u32_t addr, u32_t size, u8_t *src) {
    my_spi_write(addr, size, src);
    return SPIFFS_OK;
  }

  static s32_t my_spiffs_erase(u32_t addr, u32_t size) {
    my_spi_erase(addr, size);
    return SPIFFS_OK;
  }
