//
// Created by promise on 11/4/23.
//

#ifndef RM_CPP_AVGFILTER_HPP
#define RM_CPP_AVGFILTER_HPP
#include <vector>

class avgFilter {
private:
    std::vector<double> buffer;
    double avg=0;
public:
    int Size;
    void update(double data);
    void get_avg(double &data);
};


#endif //RM_CPP_AVGFILTER_HPP
