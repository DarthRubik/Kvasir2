#ifndef KVASIR2_HPP
#define KVASIR2_HPP

#include <array>
#include <utility>
#include <boost/hana.hpp>
#include <algorithm>
#include <type_traits>
#include <tuple>
#include <cinttypes>
#include <variant>
#include <variant>
#include "algorithm.hpp"


using reg_type = std::uint32_t;
using op_type = std::conditional_t<
    (sizeof(std::uint32_t) > sizeof(std::uintptr_t)),
    std::uint32_t,std::uintptr_t>;


enum access_kind
{
    nop,            // Pretty self explanatory
    write_lit,      // *addr  <-- literal
    read,           // *addr  --> reg[r1]
    write_rel,      // *addr  <-- reg[r1]
    or_lit,         // reg[0] <-- lit | reg[r1]
    and_lit,        // reg[0] <-- lit & reg[r1]
    not_op,         // reg[0] <-- ~reg[r1]
    or_op,          // reg[0] <-- reg[r1] | reg[r2]
    and_op,         // reg[0] <-- reg[r1] & reg[r2]
    output_op,      // output <-- reg[r1]
    shift_right,    // reg[0] <-- reg[r1] >> lit
};
struct access
{
    access_kind kind;
    op_type op1;
    op_type op2;
};
struct null_t{};
template<access_kind, op_type, op_type>
struct action{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T) const{}
};

#ifdef USE_KVASIR2_DEBUG_WRITES
reg_type debug_memory[1000];
#endif

template<op_type addr, op_type val>
struct action<write_lit,addr,val>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T) const
    {
#ifdef USE_KVASIR2_DEBUG_WRITES
        debug_memory[addr] = val;
#else
        *reinterpret_cast<reg_type volatile*>(addr) = val;
#endif
    }
};


template<op_type addr, op_type reg>
struct action<write_rel,addr,reg>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T) const
    {
#ifdef USE_KVASIR2_DEBUG_WRITES
        debug_memory[addr] = cont[reg];
#else
        *reinterpret_cast<reg_type volatile*>(addr) = cont[reg];
#endif
    }
};
template<op_type addr, op_type reg>
struct action<read,addr,reg>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T) const
    {
#ifdef USE_KVASIR2_DEBUG_WRITES
        cont[reg] = debug_memory[addr];
#else
        cont[reg] = *reinterpret_cast<reg_type volatile*>(addr);
#endif
    }
};
template<op_type literal, op_type reg>
struct action<or_lit,literal,reg>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont, T) const
    {
        cont[0] = literal | cont[reg];
    }
};
template<op_type literal, op_type reg>
struct action<and_lit,literal,reg>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T) const
    {
        cont[0] = literal & cont[reg];
    }
};
template<op_type reg, op_type op2>
struct action<not_op,reg,op2>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T) const
    {
        cont[0] = ~cont[reg];
    }
};
template<op_type r1, op_type r2>
struct action<or_op,r1,r2>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T) const
    {
        cont[0] = cont[r1] | cont[r2];
    }
};
template<op_type r1, op_type r2>
struct action<and_op,r1,r2>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T) const
    {
        cont[0] = cont[r1] & cont[r2];
    }
};
template<op_type r1, op_type lit>
struct action<shift_right,r1,lit>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T) const
    {
        cont[0] = cont[r1] >> lit;
    }
};
template<op_type r1, op_type tuple_index>
struct action<output_op,r1,tuple_index>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T& t) const
    {
        using type = std::decay_t<decltype(std::get<tuple_index>(t))>;
        std::get<tuple_index>(t) = (type)cont[r1];
    }
};



struct bit_location_rt
{
    op_type Addr;
    reg_type Mask;
    unsigned int Shift;
    constexpr bit_location_rt() : bit_location_rt(0,0,0){}
    constexpr bit_location_rt(op_type Addr,reg_type Mask,unsigned int Shift)
        : Addr(Addr), Mask(Mask), Shift(Shift) {}
};

template<op_type addr, op_type mask, typename type_t, unsigned int shift = 0>
struct bit_location
{
    static constexpr op_type Addr = addr;
    static constexpr reg_type Mask = mask;
    using type = type_t;
    static constexpr unsigned int Shift = shift;

    constexpr auto to_rt() const
        { return bit_location_rt{ Addr, Mask, Shift }; }
};

static constexpr int max_inst_size = 5;


struct blind_write_rt
{
    op_type addr;
    reg_type value;
    constexpr blind_write_rt(op_type addr, reg_type value) : addr(addr), value(value) {}
    constexpr auto get_inst() const
    {
        return std::array<access,max_inst_size>{
            access{write_lit,addr,value},
        };
    }
};
struct set_value_rt
{
    bit_location_rt bit_loc;
    op_type value;
    constexpr set_value_rt() : set_value_rt(bit_location_rt{},0){}
    constexpr set_value_rt(bit_location_rt bit_loc, op_type value)
        : bit_loc(bit_loc), value(value)
    {
        this->value <<= this->bit_loc.Shift;
        this->bit_loc.Shift = 0;
    }
    constexpr auto get_inst() const
    {
        return std::array<access,max_inst_size>{
            access{read,bit_loc.Addr,0},
            access{and_lit,~bit_loc.Mask,0},
            access{or_lit,(value)<<bit_loc.Shift,0},
            access{write_rel,bit_loc.Addr,0},
        };
    }
};

constexpr set_value_rt merge_writes(set_value_rt a, set_value_rt b)
{
    return set_value_rt{
        bit_location_rt{b.bit_loc.Addr,a.bit_loc.Mask | b.bit_loc.Mask,0},
        a.value | b.value
    };
}


struct read_value_rt
{
    bit_location_rt bit_loc;
    op_type tuple_index;
    constexpr auto get_inst() const
    {
        return std::array<access,max_inst_size>{
            access{read,bit_loc.Addr,0},
            access{and_lit,bit_loc.Mask,0},
            access{shift_right,0,bit_loc.Shift},
            access{output_op,0,tuple_index},
        };
    }
};
struct nop_rt
{
    constexpr auto get_inst() const
    {
        return std::array<access,max_inst_size>{};
    }
};


using ast_node = std::variant<set_value_rt,blind_write_rt,read_value_rt,nop_rt>;



// These are the things that you pass to "apply"
// They create the "instructions" to be executed
template<typename bit_loc, typename bit_loc::type value>
struct set_value
{
    using type = null_t;
    constexpr auto get_rt() const
    {
        return set_value_rt{ bit_loc{}.to_rt(), value };
    }
};
template<typename bit_loc>
struct read_value
{
    using type = typename bit_loc::type;
    constexpr auto get_rt() const
    {
        return read_value_rt{ bit_loc{}.to_rt() };
    }

};


struct register_state
{
    reg_type bits = 0;
    reg_type known_mask = 0;
    bool cached = false;
    constexpr register_state() = default;
    constexpr register_state(const register_state&) = default;
    constexpr register_state(register_state&&) = default;
    constexpr register_state& operator=(const register_state&) = default;
    constexpr register_state& operator=(register_state&&) = default;

    constexpr void write(reg_type value, reg_type mask)
    {
        bits = (bits & ~mask) | value;
        known_mask |= mask;
        cached = true;
    }
    constexpr void read()
    {
        cached = true;
    }
};


template<std::size_t size>
constexpr std::array<ast_node,size> optimize_ast(std::array<ast_node,size> ast)
{
    using boost::hana::make_pair;
    ::map<op_type,register_state,size> mem_state;

    int index = 0;
    for (auto i = ast.begin(); i != ast.end(); ++i)
    {
        if (std::holds_alternative<read_value_rt>(*i))
        {
            std::get<read_value_rt>(*i).tuple_index = index++;
        }
    };

    auto part = [](auto x) { return std::holds_alternative<set_value_rt>(x); };
    auto addr_v = [](auto x){ return std::get<set_value_rt>(x).bit_loc.Addr; };
    auto addr_lt =  [&](auto x, auto y){ return addr_v(x) < addr_v(y); };

    // Partition the instructions into a range of writes and other stuff
    auto [ set_value_view, other_view ] = ::stable_partition(ast,part);

    // Sorting the writes by address lets us merge them together later
    ::quick_sort(set_value_view,addr_lt);

    std::array<decltype(view(set_value_view)),size> same_addr_range{};
    auto sadr_last = equal_ranges(set_value_view,same_addr_range.begin(),addr_lt);
    auto same_addr_ranges_v = split(same_addr_range,sadr_last).first;


    for (auto range : same_addr_ranges_v)
    {
        auto addr = addr_v(*std::begin(range));
        if (!mem_state.contains(addr))
        {
            mem_state.insert(make_pair(addr,register_state{}));
        }
        // Merge the writes together
        auto value = accumulate(range,ast_node{set_value_rt{}},[](auto x, auto y){
            return merge_writes(std::get<set_value_rt>(x),std::get<set_value_rt>(y));
        });

        set_value_rt val = std::get<set_value_rt>(value);
        mem_state[addr].write(val.value,val.bit_loc.Mask);

        // If the state of every bit is known after we write, make it into a
        // blind write
        if (mem_state[addr].known_mask == ~reg_type{0})
        {
            // FIXME: Move assign operator isn't constexpr for whatever reason
            auto value_ = ast_node{blind_write_rt{addr,mem_state[addr].bits}};
            value = value_;
        }

        // Clear out all the operations to and make room for the one that we
        // just made (These are arrays and arn't resizeable, so we just replace
        // the the ops with nops)
        transform(range,range.begin(),
            [](auto x){return ast_node(nop_rt{});});

        *std::begin(range) = value;
    }

    return ast;
}

template<typename... T, std::size_t... t_index, std::size_t... inst_index>
auto apply_impl(std::index_sequence<t_index...>,std::index_sequence<inst_index...>,T... t)
{
    using boost::hana::_;
    using boost::hana::type_c;
    using boost::hana::tuple_t;
    using boost::hana::unpack;
    using boost::hana::template_;
    using boost::hana::filter;

    // Make a tuple with all the outputs:
    // eg:
    // T = [write, read<bool> , read<char>]
    // Then we do this: [null_t, bool, char] -> [bool, char]
    // Then we add a final "null_t" so that we never try to access off
    // the end of the tuple down below
    auto return_filtered = filter(tuple_t<typename T::type...>,_!=type_c<null_t>);
    
    // Now convert the type list into a std::tuple
    using ret_t = typename decltype(unpack(return_filtered, template_<std::tuple>))::type;

    ret_t ret{};

    constexpr std::array<ast_node,sizeof...(T)> nodes =
    {
        t.get_rt()...
    };

    constexpr std::array<ast_node,sizeof...(T)> optimized = optimize_ast(nodes);

    constexpr std::array<std::array<access,max_inst_size>,sizeof...(T)> sub_prog =
    {
        std::visit([](auto x) { return x.get_inst(); }, optimized[t_index])...
    };
    
    constexpr std::array<access,sizeof...(inst_index)> inst =
    {
        sub_prog[inst_index/max_inst_size][inst_index%max_inst_size]...
    };

    // A set of "registers"
    std::array<op_type, 16> virtual_registers{};
    (action<
        inst[inst_index].kind,
        inst[inst_index].op1,
        inst[inst_index].op2>{}(
            virtual_registers,ret),...);
    
    return ret;
}
template<typename... T>
auto apply(T... t)
{
    return apply_impl(
            std::make_index_sequence<sizeof...(T)>{},
            std::make_index_sequence<sizeof...(T)*max_inst_size>{},t...);
}


#endif
