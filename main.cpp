
#define USE_KVASIR2_DEBUG_WRITES
#include "Kvasir2.hpp"
#if 1
#include "test_registers.hpp"
#include "map.hpp"
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


using ra_t = reg_access_t<::cpu>;

void apply_no_read()
{
    ra_t::apply(
        ra_t::set_value<cpu::loc1,true>{},
        ra_t::set_value<cpu::loc2,false>{}
    );
}

void apply_check_bits_set_and_cleared()
{
    ra_t::debug_memory[100] = 0xffffff00;
    ra_t::debug_memory[200] = 0xffffffff;

    auto x = ra_t::apply(
        ra_t::read_value<cpu::loc1>{},
        ra_t::read_value<cpu::loc2>{},
        ra_t::read_value<cpu::loc3>{},
        ra_t::set_value<cpu::loc1,true>{},
        ra_t::set_value<cpu::loc2,false>{},
        ra_t::set_value<cpu::loc3,false>{}
    );
    assert(ra_t::debug_memory[100] == 0xfffeff01);
    assert(ra_t::debug_memory[200] == 0xffffffdf);
    assert(std::get<0>(x));
    assert(!std::get<1>(x));
    assert(!std::get<2>(x));
}
void apply_blind_write()
{
    ra_t::debug_memory[100] = 0xffffffff;
    ra_t::apply(
        ra_t::set_value<cpu::loc1,true>{},
        ra_t::set_value<cpu::loc3,false>{},
        ra_t::set_value<cpu::loc4,0>{}
    );
    assert(ra_t::debug_memory[100] == 1);
}
void apply_combine_reads()
{
    auto x = ra_t::apply(
        ra_t::set_value<cpu::loc1,true>{},
        ra_t::set_value<cpu::loc3,true>{},
        ra_t::set_value<cpu::loc4,(10<<17)>{},
        ra_t::set_value<cpu::loc5,(0xaa)>{},
        ra_t::read_value<cpu::loc4>{},
        ra_t::read_value<cpu::loc5>{}
    );
    assert(std::get<0>(x) == (10<<17));
    assert(std::get<1>(x) == 0xaa);
}
void apply_cache_value()
{
    ra_t::debug_memory[100] = 0;
    auto x = ra_t::apply(
        ra_t::set_value<cpu::loc1,true>{},
        ra_t::read_value<cpu::loc3>{}
    );
    assert(!std::get<0>(x));

    auto y = ra_t::apply(
        ra_t::read_value<cpu::loc1>{},
        ra_t::read_value<cpu::loc3>{}
    );
    assert(std::get<0>(y));
    assert(!std::get<1>(y));
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
    apply_cache_value();
}
#endif
