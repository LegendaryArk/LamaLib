// #include "display/lv_core/lv_obj.h"
// #include "pros/apix.h"
// #include <functional>

// namespace lamaLib {
// class Button_lgvl {
// public:
//   typedef void (*button_actions)();
//   Button_lgvl(lv_obj_t *iparent);

//   static lv_res_t btnClickAction(lv_obj_t *btn);

//   static lv_obj_t *createBtn(lv_coord_t x, lv_coord_t y, lv_coord_t width,
//                       lv_coord_t height, int id, const char *title);

//   static lv_style_t *createBtnStyle(lv_style_t *copy, lv_color_t rel, lv_color_t pr,
//                              lv_color_t tglRel, lv_color_t tglPr,
//                              lv_color_t tglBorder, lv_color_t textColor,
//                              lv_obj_t *btn);

//   static void setBtnStyle(lv_style_t *btnStyle, lv_obj_t *btn);

//   static void setAction(int id, lv_action_t action());

//   // lv_res_t doAction(lv_obj_t * btn);

// private:
//   lv_obj_t *parent;
//   lv_action_t (*btnFuncList[])();
//   // std::function<void()> buttonActions[];
// };
// } // namespace lamaLib