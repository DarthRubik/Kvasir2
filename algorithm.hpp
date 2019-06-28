#ifndef ALGORITHM_HPP
#define ALGORITHM_HPP
#include <iterator>
#include <type_traits>
#include <utility>

template <typename value_type_t,
          typename reference_t,
          typename const_reference_t,
          typename difference_type_t,
          typename size_type_t,
          typename iterator_t,
          typename const_iterator_t>
class view_t
{
public:
    using value_type = value_type_t;
    using reference = reference_t;
    using const_reference = const_reference_t;
    using difference_type = difference_type_t;
    using size_type = size_type_t;

    using iterator = iterator_t;
    using const_iterator = const_iterator_t;


    typedef std::reverse_iterator<iterator> reverse_iterator;
    typedef std::reverse_iterator<const_iterator> const_reverse_iterator;


    constexpr view_t() = default;
    constexpr view_t(iterator begin_, iterator end_) : begin_(begin_), end_(end_) {}


    constexpr bool operator==(view_t other) const
    {
        return begin_ == other.begin_ && end_ == other.end_;
    }
    constexpr bool operator!=(view_t other) const
    {
        return !(*this == other);
    }

    constexpr iterator begin() const { return begin_; }
    constexpr const_iterator cbegin() const { return begin_; }
    constexpr iterator end() const { return end_; }
    constexpr const_iterator cend() const { return end_; }

    constexpr size_type size() const { return end_ - begin_; }
    constexpr bool empty() const { return end_ == begin_; }

private:
    iterator begin_;
    iterator end_;
};


template<typename cont_t>
constexpr auto slice(cont_t& cont,
        typename cont_t::iterator begin,
        typename cont_t::iterator end)
{
    return view_t<
        typename cont_t::value_type,
        typename cont_t::reference,
        typename cont_t::const_reference,
        typename cont_t::difference_type,
        typename cont_t::size_type,
        typename cont_t::iterator,
        typename cont_t::const_iterator>{ begin, end };
}

template<typename cont_t>
constexpr auto slice(cont_t const& cont,
        typename cont_t::const_iterator begin,
        typename cont_t::const_iterator end)
{
    return view_t<
        typename cont_t::value_type,
        typename cont_t::reference,
        typename cont_t::const_reference,
        typename cont_t::difference_type,
        typename cont_t::size_type,
        typename cont_t::const_iterator,
        typename cont_t::const_iterator>{ begin, end };
}

template<typename cont_t>
constexpr auto cslice(cont_t const& cont,
        typename cont_t::const_iterator begin,
        typename cont_t::const_iterator end)
{
    return slice(cont,begin,end);
}

template<typename cont_t>
constexpr auto view(cont_t& cont)
{
    return slice(cont,std::begin(cont),std::end(cont));
}
template<typename cont_t>
constexpr auto view(cont_t const& cont)
{
    return slice(cont,std::begin(cont),std::end(cont));
}
template<typename cont_t>
constexpr auto cview(cont_t const& cont)
{
    return slice(cont,std::begin(cont),std::end(cont));
}

template<typename cont_t,typename it_t>
constexpr auto split(cont_t&& cont,it_t it)
{
    return std::make_pair(
            slice(cont,std::begin(cont),it),
            slice(cont,it,std::end(cont)));
}



template<typename T>
constexpr auto swap(T& a, T& b)
{
    T temp = std::move(a);
    a = std::move(b);
    b = std::move(temp);
}

template<typename cont_t>
constexpr void reverse(cont_t&& cont)
{
    auto front = std::begin(cont);
    auto back = std::end(cont) - 1;
    while (front < back)
    {
        ::swap(*front++,*back--);
    }
}

template<typename cont_t>
constexpr auto rotate(
    cont_t&& cont,typename std::decay_t<cont_t>::iterator new_begin)
{
    auto parts = split(cont,new_begin);
    ::reverse(parts.first);
    ::reverse(parts.second);
    ::reverse(cont);
    return std::begin(cont) + (std::end(cont) - new_begin);
}

template<typename cont_t>
constexpr auto divide_cont(cont_t&& cont)
{
    auto elems = std::size(cont);
    auto mid = std::begin(cont) + elems/2;
    return split(cont,mid);
}

template<typename cont_t, typename pred_t>
constexpr auto stable_partition(cont_t&& cont, pred_t pred)
{
    if (std::size(cont) == 0)
        return std::make_pair(view(cont),view(cont));

    if (std::size(cont) == 1)
    {
        if (pred(*std::begin(cont)))
        {
            return split(cont,std::end(cont));
        }
        else
        {
            return split(cont,std::begin(cont));
        }
    }
    auto ranges = ::divide_cont(cont);

    auto top_parts = ::stable_partition(ranges.first,pred);
    auto bot_parts = ::stable_partition(ranges.second,pred);

    /*
     * [+++++++------++--]
     *  ^      ^     ^ ^ ^
     *  | top  | top | | |
     *  | first| sec | | |
     *               | | |
     *               bot
     *              first
     *                 | |
     *                 | |
     *                 bot
     *                second
     */
    auto partition_point =
        ::rotate(slice(cont,std::begin(top_parts.second),std::end(bot_parts.first)),
            std::begin(bot_parts.first));
    return split(cont,partition_point);
}


template<typename cont_t, typename pred_t>
constexpr auto partition_point(cont_t&& cont, pred_t pred)
{
    for (auto i = std::begin(cont); i != std::end(cont); ++i)
    {
        if (!pred(*i))
            return split(cont,i);
    }
    return split(cont,std::end(cont));
}

template<typename cont_t,typename pred_t>
constexpr auto count_if(const cont_t& cont, pred_t pred)
{
    std::size_t count = 0;
    for (auto const& i : cont)
    {
        if (pred(i))
            count++;
    }
    return count;
}

template<typename cont_t,typename It,typename Op>
constexpr auto transform(cont_t const& cont, It it, Op op)
{
    for (auto i = std::begin(cont); i != std::end(cont); ++i)
    {
        *it++ = op(*i);
    }
    return it;
}


// Taken from en.cppreference.com
template<class cont_t, class BinaryPredicate>
constexpr auto unique(cont_t&& cont, BinaryPredicate p)
{
    auto first = std::begin(cont);
    auto last = std::end(cont);
    if (first == last)
        return split(cont,last);

    auto result = first;
    while (++first != last) {
        if (!p(*result, *first) && ++result != first) {
            *result = std::move(*first);
        }
    }
    return split(cont,++result);
}

template<typename cont_t, typename T, typename Op>
constexpr auto lower_bound(cont_t const& cont, T const& value, Op op)
{
    auto search_range = view(cont);
    while (std::size(search_range) > 0)
    {
        auto [top, bottom] = divide_cont(search_range);

        // [ 1 2 3 4 5 5 5 8 9 10 ]
        //         ^
        if (op(*std::begin(bottom),value))
        {
            // [ 4 ]
            if (search_range == bottom)
            {
                return std::end(bottom);
            }
            search_range = bottom;
        }
        // [ 1 2 3 4 5 5 5 8 9 10 ]
        //               ^
        else
        {
            // [ 5 ]
            if (search_range == top)
            {
                return std::begin(top);
            }
            search_range = top;
        }
    }
    return std::begin(search_range);
}

template<typename cont_t, typename T, typename Op>
constexpr auto upper_bound(cont_t const& cont, T const& value, Op op)
{
    return lower_bound(cont,value,[&](auto x, auto y){ return !op(y,x); });
}
template<typename cont_t,typename T, typename Op>
constexpr auto equal_range(cont_t const& cont, T const& value, Op op)
{
    return slice(cont,lower_bound(cont,value,op),upper_bound(cont,value,op));
}
template<typename cont_t,typename T, typename Op>
constexpr auto accumulate(cont_t const& cont, T init,Op op)
{
    for (auto item : cont)
    {
        T value = op(init,item);
        init = value;
    }
    return init;
}



template<typename cont_t,typename Op>
constexpr bool is_sorted(const cont_t& cont, Op op)
{
    if (std::size(cont) == 0) return true;
    if (std::size(cont) == 1) return true;

    for (auto i = std::begin(cont); i != std::end(cont)-1; ++i)
    {
        if (op(*(i+1),*i))
            return false;
    }
    return true;
}

template<typename cont_t,typename Op>
constexpr void quick_sort(cont_t&& cont, Op op)
{
    if (::is_sorted(cont,op)) return;

    auto last = *(std::end(cont) - 1);

    auto sub = ::stable_partition(cont,[&](auto x) { return !op(last,x); });

    auto fix = [](auto& x, auto& y) {
        if (std::size(x) == 0)
        {
            ::swap(*std::begin(y),*(std::end(y)-1));
        }
    };
    fix(sub.first,sub.second);
    fix(sub.second,sub.first);


    quick_sort(sub.first,op);
    quick_sort(sub.second,op);
}

#endif
