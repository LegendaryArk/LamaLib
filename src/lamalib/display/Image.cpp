#include "image.hpp"
#include "display/lv_core/lv_obj.h"
#include "display/lv_objx/lv_img.h"

using namespace lamaLib;
Image::Image(lv_coord_t ix, lv_coord_t iy, lv_coord_t iwidth,
             lv_coord_t iheight, lv_obj_t *iparent, const void *srcImg,
             char letter) {
  lv_fs_drv_t pcfs_drv;                      /*A driver descriptor*/
  memset(&pcfs_drv, 0, sizeof(lv_fs_drv_t)); /*Initialization*/

  pcfs_drv.file_size = sizeof(pc_file_t); /*Set up fields...*/
  pcfs_drv.letter = letter;
  pcfs_drv.open = pcfs_open;
  pcfs_drv.close = pcfs_close;
  pcfs_drv.read = pcfs_read;
  pcfs_drv.seek = pcfs_seek;
  pcfs_drv.tell = pcfs_tell;
  lv_fs_add_drv(&pcfs_drv);

  image = lv_img_create(iparent, NULL);
  lv_img_set_src(image, srcImg);
  lv_obj_set_pos(image, ix, iy);
  lv_obj_set_size(image, iwidth, iheight);
}
lv_fs_res_t Image::pcfs_open(void *file_p, const char *fn, lv_fs_mode_t mode) {
  errno = 0;
  const char *flags = "";
  if (mode == LV_FS_MODE_WR)
    flags = "wb";
  else if (mode == LV_FS_MODE_RD)
    flags = "rb";
  else if (mode == (LV_FS_MODE_WR | LV_FS_MODE_RD))
    flags = "a+";

  char buf[256];
  sprintf(buf, "/%s", fn);
  pc_file_t f = fopen(buf, flags);

  if (f == NULL)
    return LV_FS_RES_UNKNOWN;
  else {
    fseek(f, 0, SEEK_SET);
    pc_file_t *fp = (pc_file_t *)file_p;
    *fp = f;
  }

  return LV_FS_RES_OK;
}
lv_fs_res_t Image::pcfs_close(void *file_p) {
  pc_file_t *fp = (pc_file_t *)file_p;
  fclose(*fp);
  return LV_FS_RES_OK;
}
lv_fs_res_t Image::pcfs_read(void *file_p, void *buf, uint32_t btr,
                             uint32_t *br) {
  pc_file_t *fp = (pc_file_t *)file_p;
  *br = fread(buf, 1, btr, *fp);
  return LV_FS_RES_OK;
}
lv_fs_res_t Image::pcfs_seek(void *file_p, uint32_t pos) {
  pc_file_t *fp = (pc_file_t *)file_p;
  fseek(*fp, pos, SEEK_SET);
  return LV_FS_RES_OK;
}
lv_fs_res_t Image::pcfs_tell(void *file_p, uint32_t *pos_p) {
  pc_file_t *fp = (pc_file_t *)file_p;
  *pos_p = ftell(*fp);
  return LV_FS_RES_OK;
}