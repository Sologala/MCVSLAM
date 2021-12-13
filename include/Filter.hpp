#ifndef FILTER_HPP
#define FILTER_HPP

#include <unordered_set>

#include "Map.hpp"
#include "unordered_map"
#include "vector"

namespace MCVSLAM {

template <typename _T, typename T>
static std::vector<_T> &operator>>(std::vector<_T> &container, T &callBack) {
    int l = 0, r = container.size();
    while (l < r) {
        if (callBack(container[l])) {
            container[l] = container[r--];
        } else {
            l++;
        }
    }
    container.resize(r);
    return container;
}

template <typename _T, typename T>
static std::unordered_set<_T> &operator>>(std::unordered_set<_T> &container, T &callBack) {
    for (auto iter = container.begin(), it_end = container.end(); iter != it_end;)
        if (callBack(*iter)) {
            container.erase(iter++);
        } else {
            iter++;
        }
    return container;
}

template <typename _T1, typename _T2, typename T>
static std::unordered_map<_T1, _T2> &operator>>(std::unordered_map<_T1, _T2> &container, T &callBack) {
    for (auto iter = container.begin(), it_end = container.end(); iter != it_end;)
        if (callBack(*iter)) {
            container.erase(iter++);
        } else {
            iter++;
        }
    return container;
}
}  // namespace MCVSLAM

#endif  // FILTER_HPP
