#pragma once
#include <type_traits>

// CC BY-SA 4.0 - by Francesco Chemolli - https://stackoverflow.com/a/31836401 - modified by Felix Weinmann
template < typename C, C beginVal, C endVal>
class EnumIterator
{
    typedef typename std::underlying_type<C>::type val_t;
    int val;
public:
    explicit EnumIterator(const C & f) : val(static_cast<val_t>(f)) {}
    EnumIterator() : val(static_cast<val_t>(beginVal)) {}

    EnumIterator operator++()
    {
        ++val;
        return *this;
    }
    C operator*() { return static_cast<C>(val); }
    bool operator!=(const EnumIterator& i) { return val != i.val; }

    EnumIterator begin() { return *this; } //default ctor is good
    EnumIterator end()
    {
        static const EnumIterator endIter = ++EnumIterator(endVal); // cache it
        return endIter;
    }

    static unsigned int enumSize() { return static_cast<unsigned int>(endVal) + 1; }
};
