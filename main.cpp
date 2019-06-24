#include <array>
#include <utility>
#include <boost/hana.hpp>
#include <algorithm>
#include <type_traits>
#include <tuple>


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
    unsigned int op1;
    unsigned int op2;
};
struct null_t{};
template<access_kind, unsigned int, unsigned int>
struct action{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T) const{}
};

template<unsigned int addr, unsigned int val>
struct action<write_lit,addr,val>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T) const
    {
        *reinterpret_cast<unsigned int volatile*>(addr) = val;
    }
};
template<unsigned int addr, unsigned int reg>
struct action<write_rel,addr,reg>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T) const
    {
        *reinterpret_cast<unsigned int volatile*>(addr) = cont[reg];
    }
};
template<unsigned int addr, unsigned int reg>
struct action<read,addr,reg>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T) const
    {
        cont[reg] = *reinterpret_cast<unsigned int volatile*>(addr);
    }
};
template<unsigned int literal, unsigned int reg>
struct action<or_lit,literal,reg>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont, T) const
    {
        cont[0] = literal | cont[reg];
    }
};
template<unsigned int literal, unsigned int reg>
struct action<and_lit,literal,reg>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T) const
    {
        cont[0] = literal & cont[reg];
    }
};
template<unsigned int reg, unsigned int op2>
struct action<not_op,reg,op2>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T) const
    {
        cont[0] = ~cont[reg];
    }
};
template<unsigned int r1, unsigned int r2>
struct action<or_op,r1,r2>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T) const
    {
        cont[0] = cont[r1] | cont[r2];
    }
};
template<unsigned int r1, unsigned int r2>
struct action<and_op,r1,r2>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T) const
    {
        cont[0] = cont[r1] & cont[r2];
    }
};
template<unsigned int r1, unsigned int lit>
struct action<shift_right,r1,lit>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T) const
    {
        cont[0] = cont[r1] >> lit;
    }
};
template<unsigned int r1, unsigned int ptr>
struct action<output_op,r1,ptr>
{
    template<typename cont_t,typename T>
    void operator()(cont_t& cont,T& t) const
    {
        t = (std::decay_t<T>)cont[r1];
    }
};

template<unsigned int addr, unsigned int mask, typename type_t, unsigned int shift = 0>
struct bit_location
{
    static constexpr unsigned int Addr = addr;
    static constexpr unsigned int Mask = mask;
    using type = type_t;
    static constexpr unsigned int Shift = shift;
};

static constexpr int max_inst_size = 5;



// These are the things that you pass to "apply"
// They create the "instructions" to be executed
template<typename bit_loc, typename bit_loc::type value>
struct set_value
{
    using type = null_t;
    constexpr auto get_inst() const
    {
        return std::array<access,max_inst_size>{
            access{read,bit_loc::Addr,0},
            access{and_lit,~bit_loc::Mask,0},
            access{or_lit,((unsigned int)value)<<bit_loc::Shift,0},
            access{write_rel,bit_loc::Addr,0},
        };
    }
};
template<typename bit_loc>
struct read_value
{
    using type = typename bit_loc::type;
    constexpr auto get_inst() const
    {
        return std::array<access,max_inst_size>{
            access{read,bit_loc::Addr,0},
            access{and_lit,bit_loc::Mask,0},
            access{shift_right,0,bit_loc::Shift},
            access{output_op,0},
        };
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

template<typename... T, std::size_t... inst_index>
auto apply_impl(std::index_sequence<inst_index...>,T... t)
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

    constexpr std::array<std::array<access,max_inst_size>,sizeof...(T)> sub_prog =
    {
        t.get_inst()...
    };

    // Preform optimization on instruction array (TODO)

    
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
    std::array<unsigned int, 16> virtual_registers{};
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
            std::make_index_sequence<sizeof...(T)*max_inst_size>{},t...);
}

bool foo(void)
{
    using loc1 = bit_location<1234,1<<0,bool,0>;
    using loc2 = bit_location<4321,1<<5,bool,5>;
    return std::get<0>(
        apply(
            set_value<loc1,true>{},
            set_value<loc2,false>{},
            read_value<loc1>{}
        )
    );
}