#pragma once

#include "api.h"

namespace lamaLib {
class Vision : public pros::Vision {
    std::unordered_map<std::string, pros::vision_signature_s_t> signatures;
	
    public:
    /**
    * @brief The vision sensor is a low quality camera that can see "blobs" of colour specified through colour signatures, calibrated with the V5 Vision Utlitiy
    *
	* To calibrate with the V5 Vision Utility, see: https://kb.vex.com/hc/en-us/articles/360035951911-Using-the-Vision-Utility-with-the-V5-Vision-Sensor
	* 
    * @param port The port of the vision sensor (1-21)
    **/
    Vision(int port);
	/**
    * @brief The vision sensor is a low quality camera that can see "blobs" of colour specified through colour signatures, calibrated with the V5 Vision Utlitiy
    *
	* To calibrate with the V5 Vision Utility, see: https://kb.vex.com/hc/en-us/articles/360035951911-Using-the-Vision-Utility-with-the-V5-Vision-Sensor
	* 
    * @param vision A pros vision sensor
    **/
	Vision(pros::Vision vision);

    /**
     * @brief Sets signatures in vision sensor memory to the signatures in the brain memory.
	 * Make sure to use signature_from_utility before using this method to convert the signature from the V5 Vision Utility to the pros signature
     * 
     * @param key The key that corresponds to the colour signature being set
	 * @param signature The colour signature that is being set
     */
	void setSignature(std::string key, pros::vision_signature_s_t signature);
    
	/**
     * @brief Gets the middle coordinate of a specificed largest object with the specified colour signature key.
     * 
     * @param key The key that corresponds to the colour signature that was assigned with setSignature()
	 * @param numLargest The nth largest object. The default is 1, being the largest object
	 * @return The middle coordinate of the nth largest object of the colour signature
     */
    int getMiddle(std::string key, int numLargest = 1);
    /**
     * @brief Gets the weight of a specificed largest object with the specified colour signature key.
     * 
     * @param key The key that corresponds to the colour signature that was assigned with setSignature()
	 * @param numLargest The nth largest object. The default is 1, being the largest object
	 * @return The width of the nth largest object of the colour signature
     */
    int getWidth(std::string key, int numLargest = 1);
	/**
	 * @brief Gets the height of a specificed largest object with the specified colour signature key.
	 * 
	 * @param key The key that corresponds to the colour signature that was assigned with setSignature()
	 * @param numLargest The nth largest object. The default is 1, being the largest object
	 * @return The height of the nth largest object of the colour signature
	 */
	int getHeight(std::string key, int numLargest = 1);
	/**
	 * @brief Gets the left coordinate value of a specificed largest object with the specified colour signature key.
	 * 
	 * @param key The key that corresponds to the colour signature that was assigned with setSignature()
	 * @param numLargest The nth largest object. The default is 1, being the largest object
	 * @return The left coordinate value of the nth largest object of the colour signature
	 */
	int getX(std::string key, int numLargest = 1);
	/**
	 * @brief Gets the top coordinate value of a specificed largest object with the specified colour signature key.
	 * 
	 * @param key The key that corresponds to the colour signature that was assigned with setSignature()
	 * @param numLargest The nth largest object. The default is 1, being the largest object
	 * @return The top coordinate value of the nth largest object of the colour signature
	 */
	int getY(std::string key, int numLargest = 1);

    /**
     * @brief Get the number of objects in the view of the vision sensor.
     * 
     * @return The number of objects in the camera's FOV
     */
    int getCount();

	/**
	 * @brief Sets the LED colour using a hex code
	 * 
	 * @param hex The hex code of the colour
	 */
	void setLedHex(int hex);
	/**
	 * @brief Sets the LED colour using a set of RGB value
	 * 
	 * @param red The red value in the RGB
	 * @param green The green value in the RGB
	 * @param blue The blue value in the RGB
	 */
	void setLedRGB(int red, int green, int blue);

	/**
	 * @brief Sets the origin of the vision sensor
	 * 
	 * The default is the top left corner
	 * 
	 * @param origin The origin position (only options are center and top left corner)
	 */
	void setOrigin(pros::vision_zero_e_t origin);
};
} // namespace lamaLib