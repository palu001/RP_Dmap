#pragma once
#include "grid.h"
#include <deque>

struct DMapCell {
  DMapCell* parent=0;
};

struct DMap: public Grid_<DMapCell> {
  using BaseType = Grid_<DMapCell>;
  using FrontierType = std::deque<DMapCell*>;
  
  DMap(int r=0, int c=0):
    BaseType(r,c){
  }

  void clear() {
    for (auto& c: cells)
      c.parent=0;
  }

  int compute(const std::vector<Vector2i>& obstacles, int dmax2);
  
  template <typename CellType>
  void copyTo(Grid_<CellType>& dest_grid, int dmax2=0) const {
    dest_grid.resize(rows, cols);
    for (size_t i=0; i<cells.size(); ++i) {
      const auto& src=cells[i];
      auto& dest=dest_grid.cells[i];
      int d2=dmax2;
      if (src.parent)
        d2=distance2(&src, src.parent);
      dest=sqrt(d2);
    }
  }

};
