#ifndef ALGORITHM_HPP
#define ALGORITHM_HPP
#include <iterator>

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
constexpr auto view(cont_t& cont)
{
    return view_t<
        typename cont_t::value_type,
        typename cont_t::reference,
        typename cont_t::const_reference,
        typename cont_t::difference_type,
        typename cont_t::size_type,
        typename cont_t::iterator,
        typename cont_t::const_iterator>{ std::begin(cont), std::end(cont) };
}

template<typename cont_t>
constexpr auto view(cont_t const& cont)
{
    return view_t<
        typename cont_t::value_type,
        typename cont_t::reference,
        typename cont_t::const_reference,
        typename cont_t::difference_type,
        typename cont_t::size_type,
        typename cont_t::const_iterator,
        typename cont_t::const_iterator>{ std::cbegin(cont), std::cend(cont) };
}

template<typename cont_t>
constexpr auto cview(cont_t const& cont)
{
    return view(cont);
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

template<typename cont_t,typename Op>
constexpr void bubble_sort(cont_t&& cont, Op op)
{
    bool did_swap = true;
    while (did_swap)
    {
        did_swap = false;
        for (auto i = std::begin(cont); i != std::prev(std::end(cont)); ++i)
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
