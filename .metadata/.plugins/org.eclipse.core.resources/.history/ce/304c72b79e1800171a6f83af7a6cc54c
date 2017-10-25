/*
 * NavX.h
 *
 *  Created on: Apr 2, 2017
 *      Author: Justin DeSimpliciis
 */

#ifndef SRC_NAVX_H_
#define SRC_NAVX_H_

#include <WPILib.h>
#include <include/AHRS.h>


class NavX {
	AHRS *ahrs;

	public:
		NavX(){
			try{
				ahrs = new AHRS(SPI::Port::kMXP);
			}catch(std::exception& e){
				std::string errbuf = "Error instatiating NavX MXP: ";
				errbuf += e.what();
				//printf("%s", errbuf);
			}
		
		}


		float GetPitch(){
			return ahrs->GetPitch();
		}
                
		float GetRoll(){
			return ahrs->GetRoll();
		}

		float GetYaw(){
			return ahrs->GetYaw();
		}

		double GetAngle(){
			return ahrs->GetAngle();
		}

		void ZeroYaw(){
			ahrs->SetAngleAdjustment(0.0);
			ahrs->Reset();
			ahrs->ZeroYaw();
		}

		void ResetDisplacement(){
			ahrs->ResetDisplacement();
		}

		float GetDisplacementTotal(){
			return GetDisplacementX() + GetDisplacementZ();
		}

		float GetDisplacementX(){
			return ahrs->GetDisplacementX();
		}
		float GetDisplacementZ(){
			return ahrs->GetDisplacementZ();
		}

};

#endif /* SRC_NAVX_H_ */
