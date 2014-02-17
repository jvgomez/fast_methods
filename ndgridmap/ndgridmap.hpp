/* Assuming 4-connectivity */

#ifndef NDGRIDMAP_H_
#define NDGRIDMAP_H_

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>

#include "../console/console.h"

struct neighbours4c
{
  int data[4];
  size_t size = 0;
};


template <class T> class nDGridMap {
	friend std::ostream& operator << 
	(std::ostream & os, const nDGridMap<T> & g) {
		os << console::str_info("Grid cell information");
		os << "\t" << g.getCell(0).type() << std::endl;
		os << "\t" << g.ncells_ << " cells." << std::endl;
		os << "\t" << g.leafsize_ << " leafsize (m)." << std::endl;
		os << "\t" << g.ndims_ << " dimensions:" << std::endl;
		
		for (int i = 0; i < g.ndims_; ++i)
			os << "\t\t" << "d" << i << "\tsize: " << g.dimsize_[i] << std::endl;   
			
		return os;
	}
	
    public: 
    
        nDGridMap<T>() {}; // Default constructor not used.
        nDGridMap<T>
        (const int ndims, const std::vector<int> & dimsize, const float leafsize = 0.05) {
			leafsize_ = leafsize;
			ndims_ = ndims;
			dimsize_ = dimsize;
			ncells_= 1;
			d_.resize(ndims_);
			dd_.resize(ndims_);
			for (int i = 0; i < ndims_; ++i) {
				ncells_ *= dimsize_[i];
				d_[i] = ncells_;
				
			}
			
			dd_[0] = d_[0];
			for (int i = 1; i < ndims_; ++i) 
				dd_[i] *= d_[i] - dd_[i-1];
			
			//Resizing gridmap and initializing with default values.
			cells_.resize(ncells_, T());
			
			for (int i = 0; i < cells_.size(); ++i)
				cells_[i].setIndex(i);
		}
       
        virtual ~nDGridMap<T>() {};  
        
        // grid[i] equivalento to grid.cells_[i];
        T & operator[]
        (const int idx) {
			return cells_[idx];
		} 
        
        float getLeafSize() const {return leafsize_;}
        
        int getNDims() const {return ndims_;}
        
        T getCell 
        (const int idx) const {
			return cells_[idx];
			}
         
        std::vector<int> getDimSizes() const     { return dimsize_;}
               
        inline float getMinValueInDim
        (const int idx, const int dim)   {
			n.size = 0;
			getNeighboursInDim(idx,n,dim);
			
			if (n.size == 1)
				return cells_[n.data[0]].getValue();
			else
				return (cells_[n.data[0]].getValue()<cells_[n.data[1]].getValue()) ? cells_[n.data[0]].getValue() : cells_[n.data[1]].getValue();
			
		}	
		     
        inline void getNeighbours 
        (const int idx, std::array<int> & neighs) {
			neighs.size = 0;
			for (int i = 0; i < ndims_; ++i)
				getNeighboursInDim(idx,neighs,i);
		}
		
		inline void getNeighboursInDim
        (const int idx, std::array<int> & neighs, const int dim) {
			int c1,c2;
			int cc = neighs.size;
			if (dim == 0) {
				c1 = idx-1;
				c2 = idx+1;
				// Checking neighbour 1.
				if ((c1 >= 0) && (c1/d_[0] == idx/d_[0]))
					neighs.data[cc++] = c1;
				// Checking neighbour 2.
				//if ((c2 < ncells_) && (c2/d_[0] == idx/d_[0])) // full check, not necessary.
				if (c2/d_[0] == idx/d_[0])
					neighs.data[cc++] = c2;
			}
			else {
				// Neighbours proposed.
				c1 = idx-d_[dim-1];
				c2 = idx+d_[dim-1];
				// Checking neighbour 1.
				if ((c1 >= 0) && (c1/d_[dim] == idx/d_[dim]))
					neighs.data[cc++] = c1;
				// Checking neighbour 2.
				//if ((c2 < ncells_) && (c2/d_[dimi] == idx/d_[dim])) // full check, not necessary.
				if (c2/d_[dim] == idx/d_[dim])
					neighs.data[cc++] = c2;
			}
			neighs.size = cc;
		}
		
		void getNeighbours8c2D
		(const int idx, std::vector<int> & neighs, bool include_current) { 
			if (include_current) 
				if ((idx >= 0) && (idx < ncells_))
					neighs.push_back(idx);
			
			getNeighboursInDim(idx,neighs,0);
			getNeighboursInDim(idx,neighs,1);
			int c1 = idx - d_[0] - 1;
			int c2 = idx - d_[0] + 1;
			int c3 = idx + d_[0] - 1;
			int c4 = idx + d_[0] + 1;

			// TODO: if we assume we are not in the border of the map we can speed up this by removing checks.
			//if ((c1 >= 0) && (c1/d_[0] == idx/d_[0]-1) && (c1/d_[1] == idx/d_[1])) // Check if it is in a row below and same 2D slice.
				neighs.push_back(c1);
			//if ((c2 >= 0) && (c2/d_[0] == idx/d_[0]-1) && (c2/d_[1] == idx/d_[1])) // Check if it is in a row below.
				neighs.push_back(c2);
			//if ((c3 < ncells_) && (c3/d_[0] == idx/d_[0]+1) && (c3/d_[1] == idx/d_[1])) // Check if it is in a row above.
				neighs.push_back(c3);
			//if ((c4 < ncells_) && (c4/d_[0] == idx/d_[0]+1) && (c4/d_[1] == idx/d_[1])) // Check if it is in a row above.
				neighs.push_back(c4);
		}
		
		void getNeighbours8c3D
		(const int idx, std::vector<int> & neighs) {
			getNeighbours8c2D(idx, neighs, false);
			
			int idx_sup = idx+d_[1];
			int idx_inf = idx-d_[1];
			if (idx_sup < ncells_)
				getNeighbours8c2D(idx+d_[1], neighs, true);
			if (idx_inf >= 0)
				getNeighbours8c2D(idx-d_[1], neighs, true);
		}
		
		
		int idx2coord 
		(const int idx, std::vector<int> & coords) {
			if (coords.size() != ndims_)
				return -1;
			else {
				coords[ndims_-1] = idx/d_[ndims_-2]; // First step done apart.
				int aux = idx - coords[ndims_-1]*d_[ndims_-2];
				for (int i = ndims_ - 2; i > 0; --i) {
					coords[i] = aux/d_[i-1];
					aux -= coords[i]*d_[i-1];
				}
				coords[0] = aux; //Last step done apart.
			}
			return 1;
		}
		
		int coord2idx
		(const std::vector<int> & coords, int & idx) {
			if (coords.size() != ndims_)
				return -1;
			else {
				idx = coords[0];
				for(int i = 1; i < ndims_; ++i)
					idx += coords[i]*d_[i-1];
			}
			return 1;
		}
		
		void showCoords
		(const int idx) {
			std::vector<int> coords(ndims_);
			idx2coord(idx, coords);
			for (int i = 0; i < ndims_; ++i)
				std::cout << coords[i] << "\t";
			std::cout << std::endl;
		}
		
		void showIdx
		(const std::vector<int> & coords) {
			int idx;
			coord2idx(coords, idx);
			std::cout << idx << std::endl;
		}
				
		/* Saved grid format:
		 * CellClass - info of the cell type\n  (string)
		 * leafsize_\n 							(float)
		 * ndims_\n								(int)
		 * dimsize_[0]\n						(int)
		 * dimsize_[1]\n						(int)
		 * ...
		 * dimsize_[ndims_-1]\n					(int)
		 * getCell(0).getValue(whattosave)\n 	(depends on whattosave)
		 * ...
		 * getCell(ncells_-1).getValue(whattosave)\n 	(depends on whattosave)
		 * */
        void saveGrid
        (const std::string filename, const int whattosave = 0) {
			std::ofstream ofs;
			ofs.open (filename,  std::ofstream::out | std::ofstream::trunc);
			
			ofs << getCell(0).type() << std::endl;
			ofs << leafsize_ << std::endl << ndims_ ;
			
			for (int i = 0; i < ndims_; ++i)
				ofs << std::endl << dimsize_[i] << "\t";
				   
			for (int i = 0; i < ncells_; ++i)
			ofs << std::endl << getCell(i).getValue(whattosave);  
		}
		
		int size
		() const {
			return ncells_;
		}
		
		
		std::vector<T> cells_;
        
    protected:
        std::vector<int> dimsize_; // Vector containing the size of each dimension.
        int ndims_;
        float leafsize_; // It is assumed that the cells in the grid are cubic.
        int ncells_;
        
        // Auxiliar vectors to speed things up.
        std::vector<int> d_; // Stores parcial multiplications of dimensions sizes. d_[0] = dimsize_[0];
																			   //   d_[1] = dimsize_[0]*dimsize_[1]; etc.
		std::vector<int> dd_; // dd_[0] = d_[0], dd_[1] = d_[1] - dd_[0], and so on.
		neighbours4c n; // For getMinValueInDim function.
};


#endif /* NDGRIDCELL_H_*/
