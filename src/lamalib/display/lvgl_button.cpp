#include "lvgl_button.hpp"

using namespace lamaLib;

lv_obj_t * lamaLib::createBtn(lv_obj_t *copy, lv_obj_t *parent, lv_coord_t x,
                    lv_coord_t y, lv_coord_t width, lv_coord_t height, int id,
                    const char *title) {
  lv_obj_t *btn = lv_btn_create(parent, copy);
  lv_obj_set_pos(btn, x, y);
  lv_obj_set_size(btn, width, height);
  lv_obj_set_free_num(btn, id);

  lv_obj_t *label = lv_label_create(btn, NULL);
  lv_label_set_text(label, title);
  lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 5);

  return btn;
}

lv_obj_t * lamaLib::copyBtn(lv_obj_t *parent, lv_obj_t *copy, int id) {
  lv_obj_t *btn = lv_btn_create(parent, copy);
  lv_obj_set_free_num(btn, id);

  return btn;
}

lv_style_t * lamaLib::createBtnStyle(lv_style_t *copy, lv_color_t rel, lv_color_t pr,
                           lv_color_t tglRel, lv_color_t tglPr,
                           lv_color_t tglBorder, lv_color_t textColor,
                           lv_obj_t *btn) {
  lv_style_t *btnStyle = (lv_style_t *)malloc(sizeof(lv_style_t) * 4);

  for (int i = 0; i < 4; i++)
    lv_style_copy(&btnStyle[i], copy);

  btnStyle[0].body.main_color = rel;
  btnStyle[0].body.grad_color = rel;
  btnStyle[0].text.color = textColor;

  btnStyle[1].body.main_color = pr;
  btnStyle[1].body.grad_color = pr;
  btnStyle[1].text.color = textColor;

  btnStyle[2].body.main_color = tglRel;
  btnStyle[2].body.grad_color = tglRel;
  btnStyle[2].body.border.width = 2;
  btnStyle[2].body.border.color = tglBorder;
  btnStyle[2].text.color = textColor;

  btnStyle[3].body.main_color = tglPr;
  btnStyle[3].body.grad_color = tglPr;
  btnStyle[3].body.border.width = 2;
  btnStyle[3].body.border.color = tglBorder;
  btnStyle[3].text.color = textColor;

  lv_btn_set_style(btn, LV_BTN_STYLE_REL, &btnStyle[0]);
  lv_btn_set_style(btn, LV_BTN_STYLE_PR, &btnStyle[1]);
  lv_btn_set_style(btn, LV_BTN_STYLE_TGL_REL, &btnStyle[2]);
  lv_btn_set_style(btn, LV_BTN_STYLE_TGL_PR, &btnStyle[3]);

  return btnStyle;
}

void lamaLib::setBtnStyle(lv_style_t *btnStyle, lv_obj_t *btn) {
  lv_btn_set_style(btn, LV_BTN_STYLE_REL, &btnStyle[0]);
  lv_btn_set_style(btn, LV_BTN_STYLE_PR, &btnStyle[1]);
  lv_btn_set_style(btn, LV_BTN_STYLE_TGL_REL, &btnStyle[2]);
  lv_btn_set_style(btn, LV_BTN_STYLE_TGL_PR, &btnStyle[3]);
}

lv_obj_t * lamaLib::createLabel(lv_obj_t *parent, lv_coord_t x, lv_coord_t y,
                      lv_coord_t width, lv_coord_t height, const char *text) {
  lv_obj_t *label = lv_label_create(parent, NULL);
  lv_obj_set_pos(label, x, y);
  lv_obj_set_size(label, width, height);
  lv_label_set_text(label, text);

  return label;
}

lv_obj_t *createBlankBtn(lv_obj_t *copy, lv_obj_t *parent, lv_coord_t x,
                         lv_coord_t y, lv_coord_t width, lv_coord_t height,
                         int id) {
  lv_obj_t *btn = lv_btn_create(parent, copy);
  lv_obj_set_pos(btn, x, y);
  lv_obj_set_size(btn, width, height);
  lv_obj_set_free_num(btn, id);
  return btn;
}

lv_obj_t *createBtnLabel(lv_obj_t *btnParent, const char *title) {
  lv_obj_t *label = lv_label_create(btnParent, NULL);
  lv_label_set_text(label, title);
  lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 5);

  return label;
}