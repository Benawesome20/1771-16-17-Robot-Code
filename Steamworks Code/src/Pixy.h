/*
 * Pixy.h
 *
 *  Created on: February 8, 2017
 *      Author: Benjamin Ventimiglia
 */

#ifndef SRC_PIXY_H_
#define SRC_PIXY_H_

#include <WPILib.h>
#include <CANTalon.h>

/*
//Default address of Pixy Camera. You can change the address of the Pixy in Pixymon under setting-> Interface
#define PIXY_I2C_DEFAULT_ADDR           0x54

// Communication/misc parameters
#define PIXY_INITIAL_ARRAYSIZE      30
#define PIXY_MAXIMUM_ARRAYSIZE      130
#define PIXY_START_WORD             0xaa55 //for regular color recognition
#define PIXY_START_WORD_CC          0xaa56 //for color code - angle rotation recognition
#define PIXY_START_WORDX            0x55aa //regular color another way around
#define PIXY_MAX_SIGNATURE          7
#define PIXY_DEFAULT_ARGVAL         0xffff

// Pixy x-y position values
#define PIXY_MIN_X                  0L   //x: 0~319 pixels, y:0~199 pixels. (0,0) starts at bottom left
#define PIXY_MAX_X                  319L
#define PIXY_MIN_Y                  0L
#define PIXY_MAX_Y                  199L

enum BlockType {
   NORMAL_BLOCK, //normal color recognition
   CC_BLOCK     //color-code(chnage in angle) recognition
};

struct Block { // A "Block" of data
  // print block structure!
  void print()
  {
    int i, j;
    char buf[128], sig[6], d;
   bool flag;
    if (signature>PIXY_MAX_SIGNATURE) // color code! (CC)
   {
      // convert signature number to an octal string
      for (i=12, j=0, flag=false; i>=0; i-=3) //assigns value to signature, x, y, width, height, and anlge
      {
        d = (signature>>i)&0x07;
        if (d>0 && !flag)
          flag = true;
        if (flag)
          sig[j++] = d + '0';
      }
      sig[j] = '\0';
      printf("CC block! sig: %s (%d decimal) x: %d y: %d width: %d height: %d angle %d\n", sig, signature, x, y, width, height, angle);
    }
   else // regular block.  Note, angle is always zero, so no need to print
      printf("sig: %d x: %d y: %d width: %d height: %d\n", signature, x, y, width, height); //prints out data to console instead of smartDashboard -> check on the side of the driver station, check +print and click view console
    //Serial.print(buf);
  }
  uint16_t signature; //Identification number for your object - you could set it in the pixymon
  uint16_t x; //0 - 320
  uint16_t y; //0 - 200
  uint16_t width;
  uint16_t height;
  uint16_t angle;
};
*/

class Pixy {
	AnalogInput offset;

public:
	Pixy(int offsetCh):
		offset(offsetCh)
	{
	}

	/* Purpose: Returns the offset of the tracked target from (-1.0,1.0), with 0 being the center
	 *
	 * Method : Checks if the PixyCam sees a target
	 * 			subtracts half the total voltage to center 0 at the center of the PixyCam's field of view
	 * 			divides by half the total voltage to make the right and left ends 1 and -1 */
	double GetOffset()
	{
		if (offset.GetVoltage())
			return ((double)offset.GetVoltage() - (3.3/2.0)) / (3.3/2.0);
		else
			return 0.0;
	}

	/* Returns the raw offset data in voltage form, (0V, 3.3V) */
	double GetRawOffset()
	{
		return offset.GetVoltage();
	}

};
#endif /* SRC_PIXY_H_ */
