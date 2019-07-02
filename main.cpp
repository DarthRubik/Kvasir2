
#define USE_KVASIR2_DEBUG_WRITES
#include "map.hpp"
#include "Kvasir2.hpp"
#if 1
#include "algorithm.hpp"
#include <cassert>
#include <iostream>


constexpr std::array<int,10> test_array =
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

void test_quick_sort()
{

    std::array<int,7> u = {1,2,2,2,4,1,2};
    quick_sort(view(u),std::less<>{});
    assert(u[0] == 1);
    assert(u[1] == 1);
    assert(u[2] == 2);
    assert(u[3] == 2);
    assert(u[4] == 2);
    assert(u[5] == 2);
    assert(u[6] == 4);

    auto unsorted = test_array;
    quick_sort(view(unsorted),std::less<>{});
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
void test_partition()
{
    auto u = test_array;
    stable_partition(view(u),[](auto x){ return x % 2 == 0; });
    assert(u[0] == 4);
    assert(u[1] == 6);
    assert(u[2] == 2);
    assert(u[3] == 10);
    assert(u[4] == 8);
    assert(u[5] == 5);
    assert(u[6] == 3);
    assert(u[7] == 7);
    assert(u[8] == 1);
    assert(u[9] == 9);
}
void test_equal_range()
{
    std::array<int,10> a =
    {
        1,2,3,4,4,4,5,6,7,8
    };
    auto r = ::equal_range(cview(a),4,std::less<>{});
    for (auto i : r)
    {
        assert(i == 4);
    }
    assert(std::size(r) == 3);
}
void test_map()
{
    using boost::hana::first;
    using boost::hana::second;
    ::map<int,int,10> my_map;
    my_map.insert(boost::hana::make_pair(5,10));
    my_map.insert(boost::hana::make_pair(2,4));
    my_map.insert(boost::hana::make_pair(10,20));
    my_map.insert(boost::hana::make_pair(3,6));
    my_map.insert(boost::hana::make_pair(42,84));

    assert(my_map[5] == 10);
    assert(my_map[2] == 4);
    assert(my_map[10] == 20);
    assert(my_map[3] == 6);
    assert(my_map[42] == 84);
    assert(my_map.size() == 5);
    assert(!my_map.empty());

    assert(::is_sorted(my_map,[](auto x, auto y){
        return boost::hana::first(x) < boost::hana::first(y);
    }));


    auto [it,suc] = my_map.insert(boost::hana::make_pair(2,8));
    assert(!suc);

    my_map.erase(2);
    assert(my_map.size() == 4);

    std::tie(it,suc) = my_map.insert(boost::hana::make_pair(2,8));
    assert(suc);
    assert(first(*it) == 2);
    assert(second(*it) == 8);
    assert(my_map[2] == 8);
    assert(my_map.size() == 5);

    my_map.clear();
    assert(my_map.empty());
    assert(my_map.size() == 0);
}


using loc1 = bit_location<100,1<<0,bool,0>;
using loc2 = bit_location<200,1<<5,bool,5>;
using loc3 = bit_location<100,1<<16,bool,16>;
using loc4 = bit_location<100,(uint32_t)~(1<<16 | 1<<0),std::uint32_t,0>;
using loc5 = bit_location<300,0xff<<8,std::uint8_t,8>;

void apply_no_read()
{
    apply(
        set_value<loc1,true>{},
        set_value<loc2,false>{}
    );
}

void apply_check_bits_set_and_cleared()
{
    debug_memory[100] = 0xffffff00;
    debug_memory[200] = 0xffffffff;

    auto x = apply(
        read_value<loc1>{},
        read_value<loc2>{},
        read_value<loc3>{},
        set_value<loc1,true>{},
        set_value<loc2,false>{},
        set_value<loc3,false>{}
    );
    assert(debug_memory[100] == 0xfffeff01);
    assert(debug_memory[200] == 0xffffffdf);
    assert(std::get<0>(x));
    assert(!std::get<1>(x));
    assert(!std::get<2>(x));
}
void apply_blind_write()
{
    debug_memory[100] = 0xffffffff;
    apply(
        set_value<loc1,true>{},
        set_value<loc3,false>{},
        set_value<loc4,0>{}
    );
    assert(debug_memory[100] == 1);
}
void apply_combine_reads()
{
    auto x = apply(
        set_value<loc1,true>{},
        set_value<loc3,true>{},
        set_value<loc4,(10<<17)>{},
        set_value<loc5,(0xaa)>{},
        read_value<loc4>{},
        read_value<loc5>{}
    );
    assert(std::get<0>(x) == (10<<17));
    assert(std::get<1>(x) == 0xaa);
}


int main(void)
{
    test_quick_sort();
    test_partition();
    test_equal_range();
    test_map();
    apply_no_read();
    apply_check_bits_set_and_cleared();
    apply_blind_write();
    apply_combine_reads();
}
#endif
