/*
 * PixyI2C.h
 *
 *  Created on: Mar 21, 2017
 *      Author: Justin DeSimpliciis
 */

#ifndef SRC_PIXYI2C_H_
#define SRC_PIXYI2C_H_

#include <WPILib.h>
#include "Definitions.h"

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

class PixyI2C {
	I2C *i2c;
	BlockType blockType;//Enum
	bool  skipStart;   //skips to check 0xaa55, which is byte that tells pixy it is start of new frame
	uint16_t blockCount; //How many signatured objects there are
	Block blocks[100];

public:
	PixyI2C():
		i2c(I2C::Port::kOnboard,PIXY_I2C_DEFAULT_ADDR)
	{
	}

	bool getStart() //checks whether if it is start of the normal frame, CC frame, or the data is out of sync
	      {
	        uint16_t w, lastw;

	        lastw = 0xffff;

	        while(true)
	        {
	          w = getWord();
	          if (w==0 && lastw==0)
	         {
	           return false;
	         }
	          else if (w==PIXY_START_WORD && lastw==PIXY_START_WORD)
	         {
	            blockType = NORMAL_BLOCK;
	            return true;
	         }
	          else if (w==PIXY_START_WORD_CC && lastw==PIXY_START_WORD)
	         {
	            blockType = CC_BLOCK;
	            return true;
	         }
	         else if (w==PIXY_START_WORDX) //when byte recieved was 0x55aa instead of otherway around, the code syncs the byte
	         {
	           printf("Pixy: reorder");
	           getByte(); // resync
	         }
	         lastw = w;
	        }
	      }

	   uint16_t getWord() //Getting two Bytes from Pixy (The full information)
	   {
	      unsigned char buffer[2] = {0, 0};

	      i2c->ReadOnly(2, buffer);
	      return (buffer[1] << 8) | buffer[0]; //shift buffer[1] by 8 bits and add( | is bitwise or) buffer[0] to it
	   }

	   uint8_t getByte()//gets a byte
	   {
	      unsigned char buffer[1] = {0};

	      i2c->ReadOnly(1, buffer);
	      return buffer[0];
	   }

	   uint16_t getBlocks(uint16_t maxBlocks)
	   {
	     blocks[0] = {0}; //resets the array - clears out data from previous reading
	     uint8_t i;
	     uint16_t w, checksum, sum;
	     Block *block;

	     if (!skipStart) //when computer has not seen 0xaa55 (starting frame)
	     {
	       if (getStart()==false)
	         return 0;
	     }
	     else
	      skipStart = false;

	     for(blockCount=0; blockCount<maxBlocks && blockCount<PIXY_MAXIMUM_ARRAYSIZE;)
	     {
	       checksum = getWord();
	       if (checksum==PIXY_START_WORD) // we've reached the beginning of the next frame - checking for 0xaa55
	       {
	         skipStart = true; //starts this function
	        blockType = NORMAL_BLOCK;
	        //Serial.println("skip");
	         return blockCount;
	       }
	      else if (checksum==PIXY_START_WORD_CC) //we've reacehd the beginning of the next frame - checking for 0xaa56
	      {
	        skipStart = true;
	        blockType = CC_BLOCK;
	        return blockCount;
	      }
	       else if (checksum==0)
	         return blockCount;

	      block = blocks + blockCount;

	       for (i=0, sum=0; i<sizeof(Block)/sizeof(uint16_t); i++)
	       {
	        if (blockType==NORMAL_BLOCK && i>=5) // skip --if not an CC block, no need to consider angle
	        {
	         block->angle = 0;
	         break;
	        }
	         w = getWord();
	         sum += w; //sum = w + sum
	         *((uint16_t *)block + i) = w; //converts block to interger value
	       }
	       if (checksum==sum)
	         blockCount++;
	       else
	         printf("Pixy: cs error");

	      w = getWord(); //when this is start of the frame
	      if (w==PIXY_START_WORD)
	        blockType = NORMAL_BLOCK;
	      else if (w==PIXY_START_WORD_CC)
	        blockType = CC_BLOCK;
	      else
	         return blockCount;
	     }
	   }


	   double getXOffset(){
	      if(blocks->signature == BOILER_SIGNATURE){ //If pixy identifies boiler
	         return ((blocks->x) - (319.0/2.0)) / (319.0/2.0);

	      }else{
	    	  return 0.0;
	      }
	   }

	   double getYOffset(){
		   if(blocks->signature == BOILER_SIGNATURE){ //If pixy identifies boiler
		   	   return ((blocks->y) - (199.0/2.0)) / (199.0/2.0);
		   }else{
		   	   return 0.0;
		   }
	   }

};



#endif /* SRC_PIXYI2C_H_ */
