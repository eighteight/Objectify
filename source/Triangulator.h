//
//  Triangulator.h
//  objectify
//
//  Created by Gusev, Vladimir on 10/22/13.
//
//

#ifndef __objectify__Triangulator__
#define __objectify__Triangulator__

#include "Triangle.h"

class Triangulator{
public:
    Triangulator(){};
    ~Triangulator(){};
    std::vector<Triangle> triangulate(std::vector<std::vector<float> >&);

};
#endif /* defined(__objectify__Triangulator__) */
