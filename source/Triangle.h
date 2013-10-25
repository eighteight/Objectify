//
//  Triangle.h
//  objectify
//
//  Created by eight on 10/24/13.
//
//

#ifndef objectify_Triangle_h
#define objectify_Triangle_h

class Triangle {

    public:
        std::vector<float> a, b, c;
        std::vector<int> index;
    
    Triangle():a(std::vector<float>(3,0.0)), b(std::vector<float>(3,0.0)), c(std::vector<float>(3,0.0)), index(std::vector<int>(3,0)){
    }
    
};

inline std::ostream& operator<<(std::ostream& s, const  Triangle &t)
{
    s << t.index[0]<<" "<< t.a[0] << " "<< t.a[1]<<" "<< t.a[2]<<std::endl;
    s << t.index[1]<<" "<< t.b[0] << " "<< t.b[1]<<" "<< t.b[2]<<std::endl;
    s << t.index[2]<<" "<< t.c[0] << " "<< t.c[1]<<" "<< t.c[2]<<std::endl;
    
    return (s);
}


#endif
