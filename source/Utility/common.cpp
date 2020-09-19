#include "Utility/common.hpp"


double map(double value, range_t from, range_t to) {
    if(value < from.first) return to.first;
    if(value > from.second) return to.second;
    double percentage = (value - from.first) / (from.second - from.first); 
    return percentage * (to.second - to.first) + to.first;
}

// element-wise mapping
arma::vec map(arma::vec value, range_t from, range_t to) {
    for(int i = 0; i < size(value).n_rows; i++) {
        value(i) = map(value(i), from, to);
    }
    return value;
}

// pass by reference element-wise mapping, suit for large size vectors
void map2(arma::vec& value, range_t from, range_t to) {
    for(int i = 0; i < size(value).n_rows; i++) {
        value(i) = map(value(i), from, to);
    }
}