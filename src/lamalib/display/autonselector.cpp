#include "autonselector.hpp"

using namespace std;
using namespace lamaLib;

AutonSelector *AutonSelector::getAutonSelector() {
  if (!autonSelector)
    autonSelector = new AutonSelector();
  return autonSelector;
}

AutonSelector::AutonSelector() {}

AutonSelector *AutonSelector::autonSelector = nullptr;

  lv_obj_t * AutonSelector::tabview = nullptr;

	lv_obj_t * AutonSelector::soloLeftRed = nullptr;
	lv_obj_t * AutonSelector::soloRightRed = nullptr;
	lv_obj_t * AutonSelector::leftAWPRed = nullptr;
	lv_obj_t * AutonSelector::rightAWPRed = nullptr;

	lv_obj_t * AutonSelector::soloLeftBlue = nullptr;
	lv_obj_t * AutonSelector::soloRightBlue = nullptr;
	lv_obj_t * AutonSelector::leftAWPBlue = nullptr;
	lv_obj_t * AutonSelector::rightAWPBlue = nullptr;

	lv_obj_t * AutonSelector::confirmRed = nullptr;
	lv_obj_t * AutonSelector::confirmBlue = nullptr;

	lv_obj_t * AutonSelector::holder = nullptr;

	lv_obj_t * AutonSelector::redTab = nullptr;
	lv_obj_t * AutonSelector::blueTab = nullptr;

	//lv_style_t * AutonSelector::unselected;
	lv_style_t * AutonSelector::unselectedRed = nullptr;
	lv_style_t * AutonSelector::unselectedBlue = nullptr;
	lv_style_t * AutonSelector::green = nullptr;
	lv_style_t * AutonSelector::selected = nullptr;
	lv_style_t * AutonSelector::confirmStyle = nullptr;

  std::string AutonSelector::labels[] = {"placeholder", "placeholder", "placeholder", "placeholder", "placeholder", "placeholder", "placeholder", "placeholder"};

	int AutonSelector::current = 0;
	int AutonSelector::confirmed = 0;

void AutonSelector::init() {
  holder = createBtn(NULL, lv_scr_act(), -100, -50, 1, 1, 110, "help");

  // unselected = createBtnStyle(&lv_style_plain, LV_COLOR_BLACK, LV_COLOR_GRAY,
  //                             LV_COLOR_WHITE, LV_COLOR_CYAN, LV_COLOR_BLACK,
  //                             LV_COLOR_WHITE, holder);
  unselectedRed = createBtnStyle(
      &lv_style_plain, LV_COLOR_MAKE(157, 49, 47), LV_COLOR_MAKE(189, 59, 59),
      LV_COLOR_WHITE, LV_COLOR_WHITE, LV_COLOR_WHITE, LV_COLOR_WHITE, holder);
  unselectedBlue = createBtnStyle(
      &lv_style_plain, LV_COLOR_MAKE(57, 92, 120), LV_COLOR_MAKE(91, 122, 140),
      LV_COLOR_WHITE, LV_COLOR_WHITE, LV_COLOR_WHITE, LV_COLOR_WHITE, holder);
  selected = createBtnStyle(
      &lv_style_plain, LV_COLOR_YELLOW, LV_COLOR_MAKE(204, 173, 0),
      LV_COLOR_WHITE, LV_COLOR_CYAN, LV_COLOR_BLACK, LV_COLOR_BLACK, holder);
  green = createBtnStyle(&lv_style_plain, LV_COLOR_MAKE(32, 246, 0),
                         LV_COLOR_MAKE(34, 145, 0), LV_COLOR_WHITE,
                         LV_COLOR_CYAN, LV_COLOR_BLACK, LV_COLOR_BLACK, holder);
  confirmStyle = createBtnStyle(&lv_style_plain, LV_COLOR_BLACK, LV_COLOR_GRAY,
                                LV_COLOR_WHITE, LV_COLOR_CYAN, LV_COLOR_BLACK,
                                LV_COLOR_WHITE, holder);

  // create a tab view object
  tabview = lv_tabview_create(lv_scr_act(), NULL);

  // add 2 tabs (the tabs are page (lv_page) and can be scrolled
  lv_obj_t *redTab = lv_tabview_add_tab(tabview, "Red");
  lv_obj_t *blueTab = lv_tabview_add_tab(tabview, "Blue");

  soloLeftRed = createBtn(NULL, redTab, 10, 30, 130, 40, 0, labels[0].c_str());
  soloLeftBlue = createBtn(NULL, blueTab, 10, 30, 130, 40, 4, labels[4].c_str());

  confirmRed = createBtn(NULL, redTab, 10, 90, 130, 40, 20, "Confirm");
  confirmBlue = createBtn(NULL, blueTab, 10, 90, 130, 40, 20, "Confirm");

  Image RedLogo(160, 90, 130, 27, redTab, "G:usd/CtLogo.bin", 'G');
  Image BlueLogo(160, 90, 130, 27, blueTab, "G:usd/CtLogo.bin", 'G');

  setBtnStyle(unselectedRed, soloLeftRed);
  setBtnStyle(unselectedBlue, soloLeftBlue);
  setBtnStyle(confirmStyle, confirmRed);
  setBtnStyle(confirmStyle, confirmBlue);

  soloRightRed =
      createBtn(soloLeftRed, redTab, 160, 10, 130, 40, 1, labels[1].c_str());
  rightAWPRed =
      createBtn(soloLeftRed, redTab, 310, 10, 130, 40, 2, labels[2].c_str());
  leftAWPRed = 
  	  createBtn(soloLeftRed, redTab, 310, 80, 130, 40, 3, labels[3].c_str());

  soloRightBlue =
      createBtn(soloLeftBlue, blueTab, 160, 10, 130, 40, 5, labels[5].c_str());
  rightAWPBlue =
      createBtn(soloLeftBlue, blueTab, 310, 10, 130, 40, 6, labels[6].c_str());
  leftAWPBlue =
      createBtn(soloLeftBlue, blueTab, 310, 80, 130, 40, 7, labels[7].c_str());

  setBtnStyle(green, soloLeftRed);

  lv_btn_set_action(soloLeftRed, LV_BTN_ACTION_CLICK, buttonHighlight);
  lv_btn_set_action(soloRightRed, LV_BTN_ACTION_CLICK, buttonHighlight);
  lv_btn_set_action(rightAWPRed, LV_BTN_ACTION_CLICK, buttonHighlight);
  lv_btn_set_action(leftAWPRed, LV_BTN_ACTION_CLICK, buttonHighlight);

  lv_btn_set_action(soloLeftBlue, LV_BTN_ACTION_CLICK, buttonHighlight);
  lv_btn_set_action(soloRightBlue, LV_BTN_ACTION_CLICK, buttonHighlight);
  lv_btn_set_action(rightAWPBlue, LV_BTN_ACTION_CLICK, buttonHighlight);
  lv_btn_set_action(leftAWPBlue, LV_BTN_ACTION_CLICK, buttonHighlight);

  lv_btn_set_action(confirmRed, LV_BTN_ACTION_CLICK, buttonHighlight);
  lv_btn_set_action(confirmBlue, LV_BTN_ACTION_CLICK, buttonHighlight);
}

void AutonSelector::addRoute(Alliance ialliance, string ikey, int id,
                             void (*iroute)()) {
  switch (ialliance) {
  case Alliance::RED:
    labels[id] = ikey;
    redRoutes.insert_or_assign(ikey, iroute);
    redIds.insert_or_assign(id, ikey);
    break;
  case Alliance::BLUE:
    labels[id] = ikey;
    blueRoutes.insert_or_assign(ikey, iroute);
    blueIds.insert_or_assign(id, ikey);
    break;
  }
}
void AutonSelector::removeRoute(Alliance ialliance, string ikey, int id) {
  switch (ialliance) {
  case Alliance::RED:
    labels[id] = "placeholder";
    redIds.erase(id);
    redRoutes.erase(ikey);
    break;
  case Alliance::BLUE:
    labels[id] = "placeholder";
    blueIds.erase(id);
    blueRoutes.erase(ikey);
    break;
  }
}

void AutonSelector::runSelectedRoute() {
  if (confirmed <= 3) {
    loaded = redRoutes.at(redIds.at(confirmed));
    loaded();
  } else if (confirmed > 3) {
    loaded = blueRoutes.at(blueIds.at(confirmed));
    loaded();
  }
}

string AutonSelector::getSelectedKey() {
	return redIds.at(confirmed);
}

lv_res_t AutonSelector::buttonHighlight(lv_obj_t *button) {
  uint8_t id = lv_obj_get_free_num(button);
  if (id == 0) {
    if (confirmed != 0) setBtnStyle(selected, soloLeftRed);
    if (confirmed != 1) setBtnStyle(unselectedRed, soloRightRed);
    if (confirmed != 2) setBtnStyle(unselectedRed, rightAWPRed);
    if (confirmed != 3) setBtnStyle(unselectedRed, leftAWPRed);
    if (confirmed != 4) setBtnStyle(unselectedBlue, soloLeftBlue);
    if (confirmed != 5) setBtnStyle(unselectedBlue, soloRightBlue);
    if (confirmed != 6) setBtnStyle(unselectedBlue, rightAWPBlue);
    if (confirmed != 7) setBtnStyle(unselectedBlue, leftAWPBlue);
    current = 0;
  }
  if (id == 1) {
    if (confirmed != 0) setBtnStyle(unselectedRed, soloLeftRed);
    if (confirmed != 1) setBtnStyle(selected, soloRightRed);
    if (confirmed != 2) setBtnStyle(unselectedRed, rightAWPRed);
    if (confirmed != 3) setBtnStyle(unselectedRed, leftAWPRed);
    if (confirmed != 4) setBtnStyle(unselectedBlue, soloLeftBlue);
    if (confirmed != 5) setBtnStyle(unselectedBlue, soloRightBlue);
    if (confirmed != 6) setBtnStyle(unselectedBlue, rightAWPBlue);
    if (confirmed != 7) setBtnStyle(unselectedBlue, leftAWPBlue);
    current = 1;
  }
  if (id == 2) {
    if (confirmed != 0) setBtnStyle(unselectedRed, soloLeftRed);
    if (confirmed != 1) setBtnStyle(unselectedRed, soloRightRed);
    if (confirmed != 2) setBtnStyle(selected, rightAWPRed);
    if (confirmed != 3) setBtnStyle(unselectedRed, leftAWPRed);
    if (confirmed != 4) setBtnStyle(unselectedBlue, soloLeftBlue);
    if (confirmed != 5) setBtnStyle(unselectedBlue, soloRightBlue);
    if (confirmed != 6) setBtnStyle(unselectedBlue, rightAWPBlue);
    if (confirmed != 7) setBtnStyle(unselectedBlue, leftAWPBlue);
    current = 2;
  }
  if (id == 3) {
    if (confirmed != 0) setBtnStyle(unselectedRed, soloLeftRed);
    if (confirmed != 1) setBtnStyle(unselectedRed, soloRightRed);
    if (confirmed != 2) setBtnStyle(unselectedRed, rightAWPRed);
    if (confirmed != 3) setBtnStyle(selected, leftAWPRed);
    if (confirmed != 4) setBtnStyle(unselectedBlue, soloLeftBlue);
    if (confirmed != 5) setBtnStyle(unselectedBlue, soloRightBlue);
    if (confirmed != 6) setBtnStyle(unselectedBlue, rightAWPBlue);
    if (confirmed != 7) setBtnStyle(unselectedBlue, leftAWPBlue);
    current = 3;
  }
  if (id == 4) {
    if (confirmed != 0) setBtnStyle(unselectedRed, soloLeftRed);
    if (confirmed != 1) setBtnStyle(unselectedRed, soloRightRed);
    if (confirmed != 2) setBtnStyle(unselectedRed, rightAWPRed);
    if (confirmed != 3) setBtnStyle(unselectedRed, leftAWPRed);
    if (confirmed != 4) setBtnStyle(selected, soloLeftBlue);
    if (confirmed != 5) setBtnStyle(unselectedBlue, soloRightBlue);
    if (confirmed != 6) setBtnStyle(unselectedBlue, rightAWPBlue);
    if (confirmed != 7) setBtnStyle(unselectedBlue, leftAWPBlue);
    current = 4;
  }
  if (id == 5) {
    if (confirmed != 0) setBtnStyle(unselectedRed, soloLeftRed);
    if (confirmed != 1) setBtnStyle(unselectedRed, soloRightRed);
    if (confirmed != 2) setBtnStyle(unselectedRed, rightAWPRed);
    if (confirmed != 3) setBtnStyle(unselectedRed, leftAWPRed);
    if (confirmed != 4) setBtnStyle(unselectedBlue, soloLeftBlue);
    if (confirmed != 5) setBtnStyle(selected, soloRightBlue);
    if (confirmed != 6) setBtnStyle(unselectedBlue, rightAWPBlue);
    if (confirmed != 7) setBtnStyle(unselectedBlue, leftAWPBlue);
    current = 5;
  }
  if (id == 6) {
    if (confirmed != 0) setBtnStyle(unselectedRed, soloLeftRed);
    if (confirmed != 1) setBtnStyle(unselectedRed, soloRightRed);
    if (confirmed != 2) setBtnStyle(unselectedRed, rightAWPRed);
    if (confirmed != 3) setBtnStyle(unselectedRed, leftAWPRed);
    if (confirmed != 4) setBtnStyle(unselectedBlue, soloLeftBlue);
    if (confirmed != 5) setBtnStyle(unselectedBlue, soloRightBlue);
    if (confirmed != 6) setBtnStyle(selected, rightAWPBlue);
    if (confirmed != 7) setBtnStyle(unselectedBlue, leftAWPBlue);
    current = 6;
  }
  if (id == 7) {
    if (confirmed != 0) setBtnStyle(unselectedRed, soloLeftRed);
    if (confirmed != 1) setBtnStyle(unselectedRed, soloRightRed);
    if (confirmed != 2) setBtnStyle(unselectedRed, rightAWPRed);
    if (confirmed != 3) setBtnStyle(unselectedRed, leftAWPRed);
    if (confirmed != 4) setBtnStyle(unselectedBlue, soloLeftBlue);
    if (confirmed != 5) setBtnStyle(unselectedBlue, soloRightBlue);
    if (confirmed != 6) setBtnStyle(unselectedBlue, rightAWPBlue);
    if (confirmed != 7) setBtnStyle(selected, leftAWPBlue);
    current = 7;
  }
  if (id == 20) {
    switch (current) {
    case 0:
      setBtnStyle(green, soloLeftRed);            // 0
      setBtnStyle(unselectedRed, soloRightRed);   // 1
      setBtnStyle(unselectedRed, rightAWPRed);    // 2
      setBtnStyle(unselectedRed, leftAWPRed);     // 3
      setBtnStyle(unselectedBlue, soloLeftBlue);  // 4
      setBtnStyle(unselectedBlue, soloRightBlue); // 5
      setBtnStyle(unselectedBlue, rightAWPBlue);  // 6
      setBtnStyle(unselectedBlue, leftAWPBlue);   // 7
      confirmed = 0;
      break;
    case 1:
      setBtnStyle(unselectedRed, soloLeftRed);    // 0
      setBtnStyle(green, soloRightRed);           // 1
      setBtnStyle(unselectedRed, rightAWPRed);    // 2
      setBtnStyle(unselectedRed, leftAWPRed);     // 3
      setBtnStyle(unselectedBlue, soloLeftBlue);  // 4
      setBtnStyle(unselectedBlue, soloRightBlue); // 5
      setBtnStyle(unselectedBlue, rightAWPBlue);  // 6
      setBtnStyle(unselectedBlue, leftAWPBlue);   // 7
      confirmed = 1;
      break;
    case 2:
      setBtnStyle(unselectedRed, soloLeftRed);    // 0
      setBtnStyle(unselectedRed, soloRightRed);   // 1
      setBtnStyle(green, rightAWPRed);            // 2
      setBtnStyle(unselectedRed, leftAWPRed);     // 3
      setBtnStyle(unselectedBlue, soloLeftBlue);  // 4
      setBtnStyle(unselectedBlue, soloRightBlue); // 5
      setBtnStyle(unselectedBlue, rightAWPBlue);  // 6
      setBtnStyle(unselectedBlue, leftAWPBlue);   // 7
      confirmed = 2;
      break;
    case 3:
      setBtnStyle(unselectedRed, soloLeftRed);    // 0
      setBtnStyle(unselectedRed, soloRightRed);   // 1
      setBtnStyle(unselectedRed, rightAWPRed);    // 2
      setBtnStyle(green, leftAWPRed);             // 3
      setBtnStyle(unselectedBlue, soloLeftBlue);  // 4
      setBtnStyle(unselectedBlue, soloRightBlue); // 5
      setBtnStyle(unselectedBlue, rightAWPBlue);  // 6
      setBtnStyle(unselectedBlue, leftAWPBlue);   // 7
      confirmed = 3;
      break;
    case 4:
      setBtnStyle(unselectedRed, soloLeftRed);    // 0
      setBtnStyle(unselectedRed, soloRightRed);   // 1
      setBtnStyle(unselectedRed, rightAWPRed);    // 2
      setBtnStyle(unselectedRed, leftAWPRed);     // 3
      setBtnStyle(green, soloLeftBlue);           // 4
      setBtnStyle(unselectedBlue, soloRightBlue); // 5
      setBtnStyle(unselectedBlue, rightAWPBlue);  // 6
      setBtnStyle(unselectedBlue, leftAWPBlue);   // 7
      confirmed = 4;
      break;
    case 5:
      setBtnStyle(unselectedRed, soloLeftRed);   // 0
      setBtnStyle(unselectedRed, soloRightRed);  // 1
      setBtnStyle(unselectedRed, rightAWPRed);   // 2
      setBtnStyle(unselectedRed, leftAWPRed);    // 3
      setBtnStyle(unselectedBlue, soloLeftBlue); // 4
      setBtnStyle(green, soloRightBlue);         // 5
      setBtnStyle(unselectedBlue, rightAWPBlue); // 6
      setBtnStyle(unselectedBlue, leftAWPBlue);  // 7
      confirmed = 5;
      break;
    case 6:
      setBtnStyle(unselectedRed, soloLeftRed);    // 0
      setBtnStyle(unselectedRed, soloRightRed);   // 1
      setBtnStyle(unselectedRed, rightAWPRed);    // 2
      setBtnStyle(unselectedRed, leftAWPRed);     // 3
      setBtnStyle(unselectedBlue, soloLeftBlue);  // 4
      setBtnStyle(unselectedBlue, soloRightBlue); // 5
      setBtnStyle(green, rightAWPBlue);           // 6
      setBtnStyle(unselectedBlue, leftAWPBlue);   // 7
      confirmed = 6;
      break;
    case 7:
      setBtnStyle(unselectedRed, soloLeftRed);    // 0
      setBtnStyle(unselectedRed, soloRightRed);   // 1
      setBtnStyle(unselectedRed, rightAWPRed);    // 2
      setBtnStyle(unselectedRed, leftAWPRed);     // 3
      setBtnStyle(unselectedBlue, soloLeftBlue);  // 4
      setBtnStyle(unselectedBlue, soloRightBlue); // 5
      setBtnStyle(unselectedBlue, rightAWPBlue);  // 6
      setBtnStyle(green, leftAWPBlue);            // 7
      confirmed = 7;
      break;
    default:
      break;
    }
  }
  return LV_RES_OK;
}