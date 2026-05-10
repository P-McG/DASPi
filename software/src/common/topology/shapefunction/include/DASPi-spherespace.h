// DASPi-spherespaceimpl.h
#pragma once

#include <cstddef>
#include <tuple>
#include <utility>
#include <type_traits>

namespace DASPi {

template<template<std::size_t> class FacetSpace, std::size_t n, class Sequence>
struct SphereSpaceImpl;

template<template<std::size_t> class FacetSpace, std::size_t n, std::size_t... i>
struct SphereSpaceImpl<FacetSpace, n, std::index_sequence<i...>>
{
    static constexpr std::size_t n_ = n;

    std::tuple<FacetSpace<i>...> facets_{};

    template<std::size_t index>
    requires (index < n)
    static consteval bool IsValidFacet()
    {
        return index < n;
    }

    template<std::size_t index>
    requires (index < n)
    constexpr auto& Facet() noexcept
    {
        return std::get<index>(facets_);
    }

    template<std::size_t index>
    requires (index < n)
    constexpr const auto& Facet() const noexcept
    {
        return std::get<index>(facets_);
    }

    template<class Function>
    constexpr void ForEachFacet(Function&& function)
    {
        (
            function.template operator()<i>(std::get<i>(facets_)),
            ...
        );
    }
};

template<template<std::size_t> class FacetSpace, std::size_t n>
using SphereSpace =
    SphereSpaceImpl<FacetSpace, n, std::make_index_sequence<n>>;

} // namespace DASPi
