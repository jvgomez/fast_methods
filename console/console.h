/*
* console.h
* Helper functions to console input-output.
* Copyright (C) 2014 Javier V. Gomez, Isaac Rivero
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONSOLE_H_
#define CONSOLE_H_

#include <string>
#include <vector>

namespace console {
	//Internal function.
	// This function looks for the position index of the parameter 
	// string "-x" and return it (-1 if not found).
	int findArguments  (int argc, char** argv, const char* argument_name);
	
	// All these functions works in the same way: they look for the position of the string "-x" so index is the
	// index of argv in which the value for "-x" is set. For example: ./test -t 1 
	// index will be 2 (referring to the value 1). val or vals contain the parameters theirselves.
			
	// Simple input parameters functions.
	int parseArguments (int argc, char** argv, const char* str, std::string &val);
	int parseArguments (int argc, char** argv, const char* str, bool &val);
	int parseArguments (int argc, char** argv, const char* str, int &val);
	int parseArguments (int argc, char** argv, const char* str, float &val);
	int parseArguments (int argc, char** argv, const char* str, double &val);
	int parseArguments (int argc, char** argv, const char* str, unsigned int &val);
	int parseArguments (int argc, char** argv, const char* str, char &val);
	
	// Multiple input parameters functions.
	int parseArguments (int argc, char** argv, const char* str, std::vector<std::string> & vals);
	int parseArguments (int argc, char** argv, const char* str, std::vector<int> & vals);
	
	// Simple console logging functions.
	void info (const std::string &val);
	void warning (const std::string &val);
	void error (const std::string &val);
	
	// Simple console logging functions designed to use in << overloading.
	std::string str_info (const std::string &val);
	std::string str_warning (const std::string &val);
	std::string str_error (const std::string &val);
}

#endif /* CONSOLE_H_ */ 
