#Copyright (c) 2015 Benjamin Pylko

#This program is free software: you can redistribute it and/or modify
#it under the terms of the GNU General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU General Public License for more details.

#You should have received a copy of the GNU General Public License
#along with this program.  If not, see <http://www.gnu.org/licenses/>.

AT = @
INCDIR = -I$(HOME)/wpilib/cpp/current/include -IC:\wpilib\user\cpp\include -IC:\navx-mxp\cpp -I$(HOME)/Users/PROGRAMMING/2016-2017/Robot\ Code/1771-16-17-Robot-Code/Steamworks\ Code/src
CPPFLAGS += -g -Wall -W $(INCDIR) -std=c++14 -fPIC
LFLAGS = -L$(HOME)/wpilib/cpp/current/lib -lwpi -lnavx_frc_cpp -lCTRLib -LC:\navx-mxp\cpp\incldue -LC:\navx-mxp\ -LC:\navx-mxp\cpp -LC:\navx-mxp\cpp\lib -LC:\wpilib\user\cpp\lib -LC:\wpilib\cpp\current\lib
CXX = arm-frc-linux-gnueabi-g++
TEAM = 1771
RMCOMMAND = rm -f
DEPLOYTARGET = roboRIO-$(TEAM)-FRC.local

SOURCES = $(wildcard src/*.cpp)
HEADERS = $(wildcard src/*.h)
OBJECTS = $(patsubst src/%.cpp,src/%.o,$(wildcard src/*.cpp))



all: $(OBJECTS)
	test -d bin/ || mkdir -p bin/
	$(CXX) $(CPPFLAGS) $(OBJECTS) $(LFLAGS) -o bin/FRCUserProgram
clean:
	@for dir in src; do find $$dir -name \*.o -exec $(RMCOMMAND) {} \; ; done
	$(RMCOMMAND) bin/*
	
deploy:
	ssh lvuser$(AT)$(DEPLOYTARGET) "rm -f /home/lvuser/FRCUserProgram || true"
	sftp -oBatchMode=no -b sftpbatchfile lvuser$(AT)$(DEPLOYTARGET)
	ssh lvuser$(AT)$(DEPLOYTARGET) ". /etc/profile.d/natinst-path.sh; chmod a+x /home/lvuser/FRCUserProgram; /usr/local/frc/bin/frcKillRobot.sh -t -r"

kill-robot:
	ssh lvuser$(AT)$(DEPLOYTARGET) "/usr/local/frc/bin/frcKillRobot.sh"
