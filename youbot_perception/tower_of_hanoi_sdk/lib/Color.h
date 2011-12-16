/*
 * Color.h
 *
 *  Created on: Dec 5, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#ifndef COLOR_H_
#define COLOR_H_

#include <sstream>

namespace BRICS_3D{
class Color {

	int red;
	int blue;
	int green;

	std::string name;
public:
	Color(){
		this->red = 255;
		this->green = 255;
		this->blue = 255;
		this->name = "WHITE";
	};
	~Color(){};

	void setColor(int red, int green, int blue, std::string label){
		this->red = red%256;
		this->green = green%256;
		this->blue = blue%256;
		this->name = label;
	}

	void setColor(float rgbVal24Bit, std::string label){
		uint32_t rgb = *reinterpret_cast<int*> (&rgbVal24Bit);
		this->red = ((rgb >> 16) & 0xff);
		this->green = ((rgb >> 8) & 0xff);
		this->blue = (rgb & 0xff);
		this->name = label;
	}


	/**
	 * Find the similarity between this color and the quried color value
	 * @return
	 */
	double findSimilarityRGBSpace(int redQ, int greenQ, int blueQ){
		float currentDistance = 	std::sqrt( 	(redQ-this->red)*(redQ-this->red) + (greenQ-this->green)*(greenQ-this->green) +
														(blueQ-this->blue)*(blueQ-this->blue) );
		return currentDistance;
	}

	/**
	 * Find the similarity between this color and the quried color value
	 * @return
	 */
	double findSimilarityRGBSpace(int rgbVal24BitQ){
		uint32_t rgb = *reinterpret_cast<int*> (&rgbVal24BitQ);
		int redQ = ((rgb >> 16) & 0xff);
		int greenQ = ((rgb >> 8) & 0xff);
		int blueQ = (rgb & 0xff);

		float currentDistance = 	std::sqrt( 	(redQ-this->red)*(redQ-this->red) + (greenQ-this->green)*(greenQ-this->green) +
												(blueQ-this->blue)*(blueQ-this->blue) );

		return currentDistance;
	}

    int getBlue() const
    {
        return blue;
    }

    int getGreen() const
    {
        return green;
    }

    std::string getName() const
    {
        return name;
    }

    int getRed() const
    {
        return red;
    }

    void setBlue(int blue)
    {
        this->blue = blue;
    }

    void setGreen(int green)
    {
        this->green = green;
    }

    void setName(std::string name)
    {
        this->name = name;
    }

    void setRed(int red)
    {
        this->red = red;
    }

};
}
#endif /* COLOR_H_ */
