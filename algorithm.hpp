#ifndef ALGORITHM_HPP
#define ALGORITHM_HPP


template<typename it,typename pred_t>
constexpr auto count_if(it begin, it end, pred_t pred)
{
    std::size_t count = 0;
    for (it i = begin; i != end; ++i)
    {
        if (pred(*i))
            count++;
    }
    return count;
}

template<typename It,typename Op>
constexpr void bubble_sort(It begin, It end, Op op)
{
    bool did_swap = true;
    while (did_swap)
    {
        did_swap = false;
        for (It i = begin; i != std::prev(end); ++i)
        {
            if (op(*(std::next(i)),*i))
            {
                auto temp = *i;
                *i = *std::next(i);
                *std::next(i) = temp;
                did_swap = true;
            }
        }
    }
}


#endif
