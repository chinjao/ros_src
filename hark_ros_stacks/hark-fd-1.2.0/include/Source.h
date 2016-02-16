/*
 * Copyright 2008 Kyoto University and Honda Motor Co.,Ltd.
 * All rights reserved.
 * HARK was developed by researchers in Okuno Laboratory
 * at the Kyoto University and Honda Research Institute Japan Co.,Ltd.
 */
#include "Object.h"
#include "ObjectParser.h"
#include "misc.h"
#include <cmath>

namespace FD {

class Source : public Object {
public:
    float x[3];
    float power;
    int id;
    float remaining;

    Source() {
    }

    Source(const Source& src) {
      x[0] = src.x[0];
      x[1] = src.x[1];
      x[2] = src.x[2];
      power = src.power;
      id = src.id;
      remaining = src.remaining;
    }

    float angle(const float* y) const
    {
      float prod = (x[0]*y[0]+x[1]*y[1]+x[2]*y[2]) / sqrtf(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]) / sqrtf(y[0]*y[0]+y[1]*y[1]+y[2]*y[2]);
        if (prod < -1) {
            prod = -1;
        }
        if (prod > 1) {
            prod = 1;
        }
        return acosf(prod);
    }

    float angle(const Source& src)
    {
      float prod = (x[0]*src.x[0]+x[1]*src.x[1]+x[2]*src.x[2]) 
	/ sqrtf(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]) 
	/ sqrtf(src.x[0]*src.x[0]+src.x[1]*src.x[1]+src.x[2]*src.x[2]);
      return acosf(prod);
    }

    virtual ObjectRef clone()
    {
        return ObjectRef(new Source(*this));
    }

    virtual void printOn(std::ostream& out) const
    {
        out << "<Source " << std::endl;
        out << "<x " << x[0] << " " << x[1] << x[2] << " >" << std::endl;
        out << "<power " << power << " >" << std::endl;
        out << "<id " << id << " >" << std::endl;
        out << ">" << std::endl;
    }

    void readFrom(std::istream &in)
    {
    }

    friend std::istream& operator>>(std::istream &in, Source& src);
};

inline std::istream& operator>>(std::istream &in, Source& src)
{
    if (!isValidType(in, "Source")) {
        return in;
    }
    src.readFrom(in);
    return in;
}

} // namespace FD
