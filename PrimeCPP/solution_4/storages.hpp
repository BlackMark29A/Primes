#pragma once

#include <array>
#include <string>
#include <vector>

#include <climits>
#include <cstdint>
#include <cstring>

#include "utils.hpp"

namespace detail {

template<typename T>
static inline std::string formatType()
{
    if constexpr(std::is_same_v<std::remove_cvref_t<T>, bool>) {
        return "<bool>";
    }
    else if constexpr(std::is_same_v<std::remove_cvref_t<T>, std::uint8_t>) {
        return "<u8>";
    }
    else if constexpr(std::is_same_v<std::remove_cvref_t<T>, std::uint16_t>) {
        return "<u16>";
    }
    else if constexpr(std::is_same_v<std::remove_cvref_t<T>, std::uint32_t>) {
        return "<u32>";
    }
    else if constexpr(std::is_same_v<std::remove_cvref_t<T>, std::uint64_t>) {
        return "<u64>";
    }
    else {
        static_assert(utils::always_false_v<T>, "Unknown type");
    }
}

} // namespace detail

template<typename T, bool Invert = true>
class VectorStorage {

    class ElementReference {
      public:
        explicit ElementReference(VectorStorage& parent, const std::size_t idx) : m_parent(parent), m_idx(idx) {}

        inline ElementReference& operator=(const T& value)
        {
            m_parent.m_storage[m_idx] = Invert ? !value : value;
            return *this;
        }

        inline operator T() const { return Invert ? !m_parent.m_storage[m_idx] : m_parent.m_storage[m_idx]; }

      private:
        VectorStorage& m_parent;
        const std::size_t m_idx;
    };

  public:
    explicit VectorStorage(const std::size_t size) : m_storage(size, !Invert) {}

    inline ElementReference operator[](const std::size_t idx) { return ElementReference{*this, idx}; }

    inline operator std::string() const
    {
        auto desc = Invert ? std::string{"inv_"} : std::string{""};
        desc += "vec";
        desc += detail::formatType<T>();
        return desc;
    }

    std::size_t getBitCount() const
    {
        if constexpr(std::is_same_v<std::remove_cv_t<T>, bool>) {
            return 1;
        }
        else {
            return sizeof(T) * CHAR_BIT;
        }
    }

  private:
    std::vector<T> m_storage;
};

template<typename T, bool Invert = true>
class BitStorage {
    static constexpr auto STORAGE_WIDTH = sizeof(T) * CHAR_BIT;

    class BitReference {
      public:
        explicit BitReference(BitStorage& parent, const std::size_t idx) : m_parent(parent), m_idx(idx) {}

        inline BitReference& operator=(const bool value)
        {
            const auto byteIdx = m_idx / STORAGE_WIDTH;
            const auto bitIdx = m_idx % STORAGE_WIDTH;

            if(value ^ Invert) {
                m_parent.m_storage[byteIdx] |= (T{1} << bitIdx);
            }
            else {
                m_parent.m_storage[byteIdx] &= ~(T{1} << bitIdx);
            }

            return *this;
        }

        inline operator bool() const
        {
            const auto byteIdx = m_idx / STORAGE_WIDTH;
            const auto bitIdx = m_idx % STORAGE_WIDTH;

            return ((m_parent.m_storage[byteIdx] >> bitIdx) & 1) ^ Invert;
        }

      private:
        BitStorage& m_parent;
        const std::size_t m_idx;
    };

  public:
    BitStorage() : m_size(0), m_storage(nullptr) {}

    template<std::size_t SieveSize>
    explicit BitStorage(const std::array<T, SieveSize>& bitSieve) : m_size(SieveSize)
                                                                  , m_storage(new T[m_size])
    {
        std::memcpy(m_storage, bitSieve.data(), m_size * sizeof(T));
    }

    explicit BitStorage(const std::size_t size) : m_size(utils::ceildiv(size, STORAGE_WIDTH)), m_storage(new T[m_size])
    {
        for(auto i = std::size_t{0}; i < m_size; ++i) {
            m_storage[i] = Invert ? T{} : ~T{};
        }
    }

    BitStorage& operator=(BitStorage&& other)
    {
        m_size = other.m_size;
        m_storage = other.m_storage;

        other.m_size = 0;
        other.m_storage = nullptr;
        return *this;
    }

    ~BitStorage() { delete[] m_storage; }

    inline BitReference operator[](const std::size_t idx) { return BitReference(*this, idx); }

    inline operator std::string() const
    {
        auto desc = Invert ? std::string{"inv_"} : std::string{""};
        desc += "bits";
        desc += detail::formatType<T>();
        return desc;
    }

    std::size_t getBitCount() const { return 1; }

  private:
    std::size_t m_size;
    T* m_storage = nullptr;
};

template<typename T, bool Invert = true>
class MaskedBitStorage {
    static constexpr auto countSetBits(const auto& num)
    {
        auto cnt = std::size_t{0};
        for(auto i = std::size_t{0}; i < sizeof(num); ++i) {
            if(num >> i & 1) {
                ++cnt;
            }
        }
        return cnt;
    }

    static constexpr auto genMaskLUT(bool invert)
    {
        auto maskLUT = std::array<std::size_t, STORAGE_WIDTH>{};
        for(auto i = std::size_t{0}; i < maskLUT.size(); ++i) {
            maskLUT[i] = std::size_t{1} << i;
            if(invert) {
                maskLUT[i] = ~maskLUT[i];
            }
        }
        return maskLUT;
    }

    static constexpr auto STORAGE_WIDTH = sizeof(T) * CHAR_BIT;
    static constexpr auto BIT_MASK = STORAGE_WIDTH - 1;
    static constexpr auto BIT_SHIFT = countSetBits(BIT_MASK);
    static constexpr auto BYTE_MASK = ~std::size_t{} & ~BIT_MASK;
    static constexpr auto MASK_LUT = genMaskLUT(false);
    static constexpr auto MASK_LUT_INV = genMaskLUT(true);

    class BitReference {
      public:
        explicit BitReference(MaskedBitStorage& parent, const std::size_t idx) : m_parent(parent), m_idx(idx) {}

        inline BitReference& operator=(const bool value)
        {
            const auto byteIdx = m_idx >> BIT_SHIFT;
            const auto bitIdx = m_idx & BIT_MASK;

            if(value ^ Invert) {
                m_parent.m_storage[byteIdx] |= MASK_LUT[bitIdx];
            }
            else {
                m_parent.m_storage[byteIdx] &= MASK_LUT_INV[bitIdx];
            }

            return *this;
        }

        inline operator bool() const
        {
            const auto byteIdx = m_idx >> BIT_SHIFT;
            const auto bitIdx = m_idx & BIT_MASK;

            if constexpr(Invert) {
                return !(m_parent.m_storage[byteIdx] & MASK_LUT[bitIdx]);
            }
            else {
                return (m_parent.m_storage[byteIdx] & MASK_LUT[bitIdx]);
            }
        }

      private:
        MaskedBitStorage& m_parent;
        const std::size_t m_idx;
    };

  public:
    MaskedBitStorage() : m_size(0), m_storage(nullptr) {}

    explicit MaskedBitStorage(const std::size_t size) : m_size(utils::ceildiv(size, STORAGE_WIDTH)), m_storage(new T[m_size])
    {
        for(auto i = std::size_t{0}; i < m_size; ++i) {
            m_storage[i] = Invert ? T{} : ~T{};
        }
    }

    ~MaskedBitStorage() { delete[] m_storage; }

    inline BitReference operator[](const std::size_t idx) { return BitReference(*this, idx); }

    inline operator std::string() const
    {
        auto desc = Invert ? std::string{"inv_"} : std::string{""};
        desc += "maskedbits";
        desc += detail::formatType<T>();
        return desc;
    }

    std::size_t getBitCount() const { return 1; }

  private:
    std::size_t m_size;
    T* m_storage = nullptr;
};
