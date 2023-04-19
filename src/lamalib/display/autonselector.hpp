// #include "Button_lvgl.hpp"
#include "pros/apix.h"
#include "lvgl_button.hpp"
#include "Image.hpp"
#include <string>
#include <unordered_map>

namespace lamaLib {
enum class Alliance { BLUE = 0, RED };

class AutonSelector {
public:
	static AutonSelector* getAutonSelector();

private:
	AutonSelector();

	static AutonSelector* autonSelector;

public:
	/**
	 * @brief Builds the auton selector
	 * @brief DO THIS LAST AFTER YOU INPUT ALL YOUR ROUTES
	 */
	static void init();

	/**
	 * @brief Adds route
	 * 
	 * @param ialliance 
	 * @param ikey 
	 * @param id 
	 * @param iroute 
	 */
	void addRoute(Alliance ialliance, std::string ikey, int id, void (*iroute)());
	/**
	 * @brief Removes route
	 * 
	 * @param ialliance 
	 * @param ikey 
	 * @param id 
	 */
	void removeRoute(Alliance ialliance, std::string ikey, int id);

	/**
	 * @brief Runs route selected on auton selector
	 * 
	 */
	void runSelectedRoute();

	/**
	 * @brief Returns the selected auton route
	 * 
	 * @return std::string 
	 */
	std::string getSelectedKey();

	private:
	/**
	 * @brief Button actions
	 * 
	 * @param button 
	 * @return lv_res_t 
	 */
    static lv_res_t buttonHighlight(lv_obj_t *button);

	std::unordered_map<std::string, void (*)()> redRoutes;
	std::unordered_map<std::string, void (*)()> blueRoutes;
	std::unordered_map<int, std::string> redIds;
	std::unordered_map<int, std::string> blueIds;
	
	static lv_obj_t * tabview;

	static lv_obj_t * soloLeftRed;
	static lv_obj_t * soloRightRed;
	static lv_obj_t * leftAWPRed;
	static lv_obj_t * rightAWPRed;

	static lv_obj_t * soloLeftBlue;
	static lv_obj_t * soloRightBlue;
	static lv_obj_t * leftAWPBlue;
	static lv_obj_t * rightAWPBlue;

	static lv_obj_t * confirmRed;
	static lv_obj_t * confirmBlue;

	static lv_obj_t * holder;

	static lv_obj_t * redTab;
	static lv_obj_t * blueTab;

	//static lv_style_t * unselected;
	static lv_style_t * unselectedRed;
	static lv_style_t * unselectedBlue;
	static lv_style_t * green;
	static lv_style_t * selected;
	static lv_style_t * confirmStyle;

	static std::string labels[8];

	static int current;
	static int confirmed;
	void (*loaded)();
};
} // namespace lamaLib