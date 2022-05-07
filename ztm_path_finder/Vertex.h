#pragma once
#include <stdio.h>

#include "Localization.h"

template <typename T>
class Vertex
{
private:
    T data;
    Localization loc;

public:
    Vertex(const T &data_) : data(data_) {}
    Vertex(const T &data_, const Localization &loc_): data(data_), loc(loc_) {}

public:
    T const& get_data() const { return data; }
    Localization const& get_loc() { return loc; }
};