
#define USE_KVASIR2_DEBUG_WRITES
#include "Kvasir2.hpp"
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
    bubble_sort(unsorted.begin(),unsorted.end(),std::less<>{});
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


int main(void)
{
    using loc1 = bit_location<100,1<<0,bool,0>;
    using loc2 = bit_location<200,1<<5,bool,5>;

    debug_memory[200] = 0xffffffff;

    bool x = std::get<0>(
        apply(
            read_value<loc1>{},
            set_value<loc1,true>{},
            set_value<loc2,false>{}
        )
    );
    assert(debug_memory[100] == 1);
    assert(debug_memory[200] == 0xffffffdf);
    assert(x);


    test_bubble_sort();
}
