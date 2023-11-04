//
// Created by promise on 11/4/23.
//

#include <valarray>
#include "avgFilter.hpp"

void avgFilter::update(double data) {
    if(!buffer.empty() && std::abs(data)<2){
        if(buffer[buffer.size()-1]>0 && data<0
        or buffer[buffer.size()-1]<0 && data>0){
            data=-data;
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

void avgFilter::get_avg(double &data) {
    data = avg;
}
