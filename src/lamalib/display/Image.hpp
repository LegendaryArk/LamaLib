#include "display/lv_core/lv_obj.h"
#include "pros/apix.h"

namespace lamaLib {
class Image {
public:
  Image(lv_coord_t ix, lv_coord_t iy, lv_coord_t iwidth, lv_coord_t iheight,
        lv_obj_t * iparent, const void *srcImg, char letter);
  /**
   * @brief Driver open function
   * 
   * @param file_p 
   * @param fn 
   * @param mode 
   * @return lv_fs_res_t 
   */
  static lv_fs_res_t pcfs_open(void *file_p, const char *fn, lv_fs_mode_t mode);
  /**
   * @brief Driver close function
   * 
   * @param file_p 
   * @return lv_fs_res_t 
   */
  static lv_fs_res_t pcfs_close(void *file_p);
  /**
   * @brief Driver read function
   * 
   * @param file_p 
   * @param buf 
   * @param btr 
   * @param br 
   * @return lv_fs_res_t 
   */
  static lv_fs_res_t pcfs_read(void *file_p, void *buf, uint32_t btr,
                               uint32_t *br);
  /**
   * @brief Driver file search function
   * 
   * @param file_p 
   * @param pos 
   * @return lv_fs_res_t 
   */
  static lv_fs_res_t pcfs_seek(void *file_p, uint32_t pos);
  /**
   * @brief Driver write function
   * 
   * @param file_p 
   * @param pos_p 
   * @return lv_fs_res_t 
   */
  static lv_fs_res_t pcfs_tell(void *file_p, uint32_t *pos_p);

private:
  typedef FILE *pc_file_t;
  lv_obj_t * image;
};
} // namespace lamaLib