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
    void update(double data) {
        if(!buffer.empty() && std::abs(data)<5){
            if(buffer[buffer.size()-1]>0 && data<0
                or buffer[buffer.size()-1]<0 && data>0){
                data=0;
            }
        }
        if(buffer.size()==Size){
            buffer.erase(buffer.begin());
            buffer.emplace_back(data);
            avg=avg-buffer[0]/Size;
            avg=avg+buffer[Size-1]/Size;
        }
        else{
            buffer.emplace_back(data);
            avg=avg*(int)(buffer.size()-1)/(int)buffer.size()+buffer[buffer.size()-1]/(int)buffer.size();
        }
    }
    void get_avg(double &data) const {
        data = avg;
    }
};


#endif //RM_CPP_AVGFILTER_HPP
