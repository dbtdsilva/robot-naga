//
// Created by Diogo Silva on 29/10/16.
//

#ifndef RAZERNAGA_FILTER_H
#define RAZERNAGA_FILTER_H

#include <vector>

template <typename T>
class Filter {
public:
    Filter(int buffer_size) : buffer_size_(buffer_size), calculated_value_(0) {
    }

    T get_value() const {
        return calculated_value_;
    }
    void update(T new_value) {
        if (buffer_.size() >= buffer_size_)
            buffer_.pop_back();
        buffer_.insert(buffer_.begin(), new_value);

        T sum = 0;
        for (int i = 0; i < buffer_.size(); i++)
            sum += buffer_.at(i);

        calculated_value_ = sum / buffer_.size();
    }
private:
    int buffer_size_;
    std::vector<T> buffer_;
    T calculated_value_;
};
#endif //RAZERNAGA_FILTER_H
