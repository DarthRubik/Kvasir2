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

template<op_type addr, op_type val>
struct action<write_lit,addr,val>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T) const
    {
        *reinterpret_cast<reg_type volatile*>(addr) = val;
    }
};

#ifdef USE_KVASIR2_DEBUG_WRITES
reg_type debug_memory[1000];
#endif


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
template<op_type r1, op_type ptr>
struct action<output_op,r1,ptr>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T& t) const
    {
        t = (std::decay_t<T>)cont[r1];
    }
};



struct bit_location_rt
{
    op_type Addr;
    reg_type Mask;
    unsigned int Shift;
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

struct set_value_rt
{
    bit_location_rt bit_loc;
    op_type value;
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
struct read_value_rt
{
    bit_location_rt bit_loc;
    constexpr auto get_inst() const
    {
        return std::array<access,max_inst_size>{
            access{read,bit_loc.Addr,0},
            access{and_lit,bit_loc.Mask,0},
            access{shift_right,0,bit_loc.Shift},
            access{output_op,0},
        };
    }
};


using ast_node = std::variant<set_value_rt,read_value_rt>;



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

template<typename it,typename pred_t>
constexpr auto count_if(it begin, it end, pred_t pred)
{  // constexpr version of count_if
    std::size_t count = 0;
    for (it i = begin; i != end; ++i)
    {
        if (pred(*i))
            count++;
    }
    return count;
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
    using boost::hana::append;

    // Make a tuple with all the outputs:
    // eg:
    // T = [write, read<bool> , read<char>]
    // Then we do this: [null_t, bool, char] -> [bool, char]
    // Then we add a final "null_t" so that we never try to access off
    // the end of the tuple down below
    auto return_filtered = append(
        filter(tuple_t<typename T::type...>,_!=type_c<null_t>),type_c<null_t>);
    
    // Now convert the type list into a std::tuple
    using ret_t = typename decltype(unpack(return_filtered, template_<std::tuple>))::type;

    ret_t ret{};

    constexpr std::array<ast_node,sizeof...(T)> nodes =
    {
        t.get_rt()...
    };

    // Preform optimization on ast_nodes (TODO)

    constexpr std::array<std::array<access,max_inst_size>,sizeof...(T)> sub_prog =
    {
        std::visit([](auto x) { return x.get_inst(); }, nodes[t_index])...
    };
    
    constexpr std::array<access,sizeof...(inst_index)> inst =
    {
        sub_prog[inst_index/max_inst_size][inst_index%max_inst_size]...
    };

    // This creates an index array to pass to std::get later
    // We essentially increment the index if we see an output_op
    // because then we move to the next tuple type
    constexpr std::array<int,std::end(inst)-std::begin(inst)> tuple_index{
        count_if(std::begin(inst),std::begin(inst)+inst_index,
            [](auto x) { return x.kind == output_op; }
        )...
    };

    // A set of "registers"
    std::array<op_type, 16> virtual_registers{};
    (action<
        inst[inst_index].kind,
        inst[inst_index].op1,
        inst[inst_index].op2>{}(
            virtual_registers,std::get<tuple_index[inst_index]>(ret)),...);
    
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
