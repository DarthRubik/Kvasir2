#ifndef TEST_REGISTERS_HPP
#define TEST_REGISTERS_HPP
#include "Kvasir2.hpp"

struct cpu
{
    using loc1 = bit_location<100,1<<0,bool,0>;
    using loc2 = bit_location<200,1<<5,bool,5>;
    using loc3 = bit_location<100,1<<16,bool,16>;
    using loc4 = bit_location<100,(uint32_t)~(1<<16 | 1<<0),std::uint32_t,0>;
    using loc5 = bit_location<300,0xff<<8,std::uint8_t,8>;

    template<typename T>
    static constexpr void enumerate(T& t)
    {
        t & loc1{}
          & loc2{}
          & loc3{}
          & loc4{}
          & loc5{};
    }
};

#endif
