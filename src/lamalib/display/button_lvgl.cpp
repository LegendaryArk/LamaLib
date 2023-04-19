// #include "button_lvgl.hpp"
// #include "display/lv_core/lv_obj.h"

// using namespace lamaLib;

// Button_lgvl::Button_lgvl(lv_obj_t * iparent){
//     parent = iparent;
// }

// lv_obj_t * Button_lgvl::createBtn(lv_coord_t x, lv_coord_t y,
//                              lv_coord_t width, lv_coord_t height, int id,
//                              const char *title) {
//   lv_obj_t * btn = lv_btn_create(lv_scr_act(), NULL);
//   lv_obj_set_pos(btn, x, y);
//   lv_obj_set_size(btn, width, height);
//   lv_obj_set_free_num(btn, id);
//   lv_btn_set_action(btn, LV_BTN_ACTION_CLICK, (*btnFuncList[id])());

//   lv_obj_t * label = lv_label_create(btn, NULL);
//   lv_label_set_text(label, title);
//   lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 5);

//   //addBtn(btn, id);
  
//   return btn;
// }

// lv_style_t * Button_lgvl::createBtnStyle(lv_style_t * copy, lv_color_t rel, lv_color_t pr,
//     lv_color_t tglRel, lv_color_t tglPr, lv_color_t tglBorder, lv_color_t textColor, lv_obj_t * btn)
// {
//     lv_style_t * btnStyle = (lv_style_t *)malloc(sizeof(lv_style_t) * 4);

//     for(int i = 0; i < 4; i++) lv_style_copy(&btnStyle[i], copy);

//     btnStyle[0].body.main_color = rel;
//     btnStyle[0].body.grad_color = rel;
//     btnStyle[0].text.color = textColor;

//     btnStyle[1].body.main_color = pr;
//     btnStyle[1].body.grad_color = pr;
//     btnStyle[1].text.color = textColor;

//     btnStyle[2].body.main_color = tglRel;
//     btnStyle[2].body.grad_color = tglRel;
//     btnStyle[2].body.border.width = 2;
//     btnStyle[2].body.border.color = tglBorder;
//     btnStyle[2].text.color = textColor;

//     btnStyle[3].body.main_color = tglPr;
//     btnStyle[3].body.grad_color = tglPr;
//     btnStyle[3].body.border.width = 2;
//     btnStyle[3].body.border.color = tglBorder;
//     btnStyle[3].text.color = textColor;

//     lv_btn_set_style(btn, LV_BTN_STYLE_REL, &btnStyle[0]);
//     lv_btn_set_style(btn, LV_BTN_STYLE_PR, &btnStyle[1]);
//     lv_btn_set_style(btn, LV_BTN_STYLE_TGL_REL, &btnStyle[2]);
//     lv_btn_set_style(btn, LV_BTN_STYLE_TGL_PR, &btnStyle[3]);

//     return btnStyle;
// }

// void Button_lgvl::setBtnStyle(lv_style_t *btnStyle, lv_obj_t *btn) {
//   lv_btn_set_style(btn, LV_BTN_STYLE_REL, &btnStyle[0]);
//   lv_btn_set_style(btn, LV_BTN_STYLE_PR, &btnStyle[1]);
//   lv_btn_set_style(btn, LV_BTN_STYLE_TGL_REL, &btnStyle[2]);
//   lv_btn_set_style(btn, LV_BTN_STYLE_TGL_PR, &btnStyle[3]);
// }

// void Button_lgvl::setAction(int id, lv_action_t action()){
//     btnFuncList[id] = action;
// }

// // lv_res_t Button_lgvl::(lv_obj_t * btn){
// //     uint8_t id = lv_obj_get_free_num(btn);
// //     (*btnFuncList[id])();
// //     return LV_RES_OK;
// // }