#ifndef MAP_HPP
#define MAP_HPP

#include <array>
#include <utility>
#include <cstdint>
#include "algorithm.hpp"

template <typename Key,typename T,std::size_t max_size_>
class map {
private:
    using array_t = std::array<std::pair<Key,T>,max_size_>;
public:
    typedef typename array_t::value_type value_type;
    typedef typename array_t::reference reference;
    typedef typename array_t::const_reference const_reference;
    typedef typename array_t::difference_type difference_type;
    typedef typename array_t::size_type size_type;
    using iterator = typename array_t::iterator;
    using const_iterator = typename array_t::const_iterator;
private:
    array_t inner;

    // Using an index instead of a pointer makes the defaults work more often
    size_type end_index;
public:

    constexpr map() : end_index(0) {}
    constexpr map(const map& other) = default;
    constexpr map(map&& other) = default;

    constexpr map& operator=(const map& other) = default;
    constexpr bool operator==(const map& other) const
    {
        return (inner == other.inner) && (end_index == other.end_index);
    }
    bool operator!=(const map& other) const
    {
        return !(*this == other);
    }

    constexpr iterator begin() { return inner.begin(); }
    constexpr const_iterator begin() const { return inner.begin(); }
    constexpr const_iterator cbegin() const { return inner.cbegin(); }
    constexpr iterator end() { return inner.begin() + end_index; }
    constexpr const_iterator end() const { return inner.begin() + end_index; }
    constexpr const_iterator cend() const { return inner.begin() + end_index; }

private:
    constexpr iterator it_get(Key key)
    {
        return ::lower_bound(view(*this),key,[](value_type v, Key k) {
            return v.first < k;
        });
    }
    constexpr const_iterator it_get(Key key) const
    {
        return ::lower_bound(view(*this),key,[](value_type v, Key k) {
            return v.first < k;
        });
    }

public:
    constexpr T& operator[](Key key)
    {
        return it_get(key)->second;
    }
    constexpr T const& operator[](Key key) const
    {
        return it_get(key)->second;
    }

    constexpr bool contains(Key key) const
    {
        return it_get(key) != end();
    }

    constexpr std::pair<iterator,bool> insert(const value_type& value)
    {
        // [124563xxxxx]
        //       ^ new element
        iterator it = it_get(value.first);

        if (it == end() || it->first != value.first)
        {
            *this->end() = value;
            end_index++;
            // [123456xxxxx]
            //    ^ new element
            ::rotate(split(*this,it).second,this->end()-1);
            return std::make_pair(it,true);
        }
        return std::make_pair(it,false);
    }
    constexpr iterator erase(Key key)
    {
        // [123456xxxxx]
        //    ^
        iterator it = it_get(key);
        if (it == end() || it->first != key)
        {
            return it;
        }

        // [124563xxxx]
        //       ^
        ::rotate(split(*this,it).second,this->begin()+1);
        end_index--;
        return it;
    }
    constexpr void clear() { end_index = 0; }

    constexpr size_type size() const
    {
        return end_index;
    }
    constexpr size_type max_size() const
    {
        return max_size_;
    }
    constexpr bool empty() const
    {
        return size() == 0;
    }
};
#endif
