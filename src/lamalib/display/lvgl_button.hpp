#include "pros/apix.h"

namespace lamaLib{
    /**
     * @brief Create a Btn object
     * 
     * @param copy 
     * @param parent 
     * @param x 
     * @param y 
     * @param width 
     * @param height 
     * @param id 
     * @param title 
     * @return lv_obj_t* 
     */
lv_obj_t *createBtn(lv_obj_t *copy, lv_obj_t *parent, lv_coord_t x,
                lv_coord_t y, lv_coord_t width, lv_coord_t height, int id,
                const char *title);

    /**
     * @brief Copies a previously made button
     * 
     * @param parent 
     * @param copy 
     * @param id 
     * @return lv_obj_t* 
     */
lv_obj_t *copyBtn(lv_obj_t *parent, lv_obj_t *copy, int id);

    /**
     * @brief Create a Btn Style object
     * 
     * @param copy 
     * @param rel 
     * @param pr 
     * @param tglRel 
     * @param tglPr 
     * @param tglBorder 
     * @param textColor 
     * @param btn 
     * @return lv_style_t* 
     */
lv_style_t *createBtnStyle(lv_style_t *copy, lv_color_t rel, lv_color_t pr,
                        lv_color_t tglRel, lv_color_t tglPr,
                        lv_color_t tglBorder, lv_color_t textColor,
                        lv_obj_t *btn);

    /**
     * @brief Set the Btn Style object
     * 
     * @param btnStyle 
     * @param btn 
     */
void setBtnStyle(lv_style_t *btnStyle, lv_obj_t *btn);

    /**
     * @brief Create a Label object
     * 
     * @param parent 
     * @param x 
     * @param y 
     * @param width 
     * @param height 
     * @param text 
     * @return lv_obj_t* 
     */
lv_obj_t *createLabel(lv_obj_t *parent, lv_coord_t x, lv_coord_t y,
                    lv_coord_t width, lv_coord_t height, const char *text);

    /**
     * @brief Create a Blank Btn
     * 
     * @param copy 
     * @param parent 
     * @param x 
     * @param y 
     * @param width 
     * @param height 
     * @param id 
     * @return lv_obj_t* 
     */
lv_obj_t *createBlankBtn(lv_obj_t *copy, lv_obj_t *parent, lv_coord_t x,
                        lv_coord_t y, lv_coord_t width, lv_coord_t height,
                        int id);

    /**
     * @brief Create a Btn Label
     * 
     * @param btnParent 
     * @param title 
     * @return lv_obj_t* 
     */
lv_obj_t *createBtnLabel(lv_obj_t *btnParent, const char *title);
}