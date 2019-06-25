
#define USE_KVASIR2_DEBUG_WRITES
#include "Kvasir2.hpp"


int main(void)
{
    using loc1 = bit_location<100,1<<0,bool,0>;
    using loc2 = bit_location<200,1<<5,bool,5>;
	
	debug_memory[200] = 0xffffffff;
	
    bool x = std::get<0>(
        apply(
            set_value<loc1,true>{},
            set_value<loc2,false>{},
            read_value<loc1>{}
        )
    );
	assert(debug_memory[100] == 1);
	assert(debug_memory[200] == 0xffffffdf);
	assert(x);
}