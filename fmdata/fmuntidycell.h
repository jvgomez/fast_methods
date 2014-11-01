/*! \file fmcell.h
    \brief Header of the FMCell class
    
   A stand-alone, standard C++ class which represents each one of the cells
   of a gridmap and its typical members. Inherited from Cell class, in this
   * case the value_ member represents the distance value (or time of arrival).
   
   IMPORTANT NOTE: no checks are done in the set functions.
    Copyright (C) 2014 Javier V. Gomez
    www.javiervgomez.com

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.
	You should have received a copy of the GNU General Public License
	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef FMUntidyCELL_H_
#define FMUntidyCELL_H_

#include <iostream>
#include <string>
#include <limits>

#include "fmcell.h"
#include "../ndgridmap/cell.h"


class FMUntidyCell : public FMCell{
	/**
	   * ostream operator << overloaded for this class.
       */
	friend std::ostream& operator << (std::ostream & os, const FMCell & c);
	
    public: 
     /**
	   * Default constructor which performs and implicit Fast Marching-like initialization of the grid,
       */
       FMUntidyCell() : FMCell()  {};

        virtual ~FMUntidyCell() {};
        
	virtual void setBucket (const int b)    				{bucket_ = b;}
										
	virtual int getbucket () const						{return bucket_;}

    protected:
	//value_ is in this case the time of arrival.
	FMState state_;  /*!< State of the cell */
	int bucket_; /*!< Time travel discretization  */
	float velocity_;  /*!< Wave propagation velocity through this cell */
	
};


#endif /* CELL_H_*/
