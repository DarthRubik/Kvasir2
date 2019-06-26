
#define USE_KVASIR2_DEBUG_WRITES
#include "Kvasir2.hpp"
#include "algorithm.hpp"
#include <cassert>



void test_bubble_sort()
{
    std::array<int,10> unsorted =
    {
        4,
        5,
        6,
        2,
        3,
        10,
        7,
        1,
        8,
        9,
    };
    bubble_sort(view(unsorted),std::less<>{});
    assert(unsorted[9] == 10);
    assert(unsorted[8] == 9);
    assert(unsorted[7] == 8);
    assert(unsorted[6] == 7);
    assert(unsorted[5] == 6);
    assert(unsorted[4] == 5);
    assert(unsorted[3] == 4);
    assert(unsorted[2] == 3);
    assert(unsorted[1] == 2);
    assert(unsorted[0] == 1);
}

using loc1 = bit_location<100,1<<0,bool,0>;
using loc2 = bit_location<200,1<<5,bool,5>;

void apply_no_read()
{
    apply(
        set_value<loc1,true>{},
        set_value<loc2,false>{}
    );
}

void apply_check_bits_set_and_cleared()
{
    debug_memory[200] = 0xffffffff;

    auto x = apply(
        read_value<loc1>{},
        read_value<loc2>{},
        set_value<loc1,true>{},
        set_value<loc2,false>{}
    );
    assert(debug_memory[100] == 1);
    assert(debug_memory[200] == 0xffffffdf);
    assert(std::get<0>(x));
    assert(!std::get<1>(x));
}


int main(void)
{
    test_bubble_sort();
    apply_no_read();
    apply_check_bits_set_and_cleared();
}
