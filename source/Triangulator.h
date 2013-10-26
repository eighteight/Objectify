//
//  Triangulator.h
//  objectify
//
//  Created by Gusev, Vladimir on 10/22/13.
//
//

#ifndef __objectify__Triangulator__
#define __objectify__Triangulator__


class Triangulator{
public:
    Triangulator(){};
    ~Triangulator(){};

    void triangulate(std::vector<std::vector<float> >& points, std::vector<std::vector<float> >& surfacePoints, std::vector<std::vector<int> >& surfaceIndeces);
        


};
#endif /* defined(__objectify__Triangulator__) */
