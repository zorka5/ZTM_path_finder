#pragma once
#include <stdio.h>

template <typename T>
class Vertex
{
private:
    T data;

public:
    Vertex(const T &data_) : data(data_) {}

public:
    T const& get_data() const { return data; }
};