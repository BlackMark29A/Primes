#include <chrono>
#include <map>
#include <type_traits>
#include <vector>

#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

template<typename T, typename U>
constexpr auto ceildiv(const T& dividend, const U& divisor)
{
    return static_cast<std::common_type_t<T, U>>(std::ceil(static_cast<double>(dividend) / divisor));
}

class BitStorage {
    using storage_t = std::uint16_t;
    static constexpr auto STORAGE_WIDTH = sizeof(storage_t) * CHAR_BIT;
    static constexpr auto INVERT = true;

    class BitReference {
      public:
        explicit BitReference(BitStorage& bitStorage, const std::size_t idx) : m_parent(bitStorage), m_idx(idx) {}

        inline BitReference& operator=(const bool value)
        {
            const auto byteIdx = m_idx / STORAGE_WIDTH;
            const auto bitIdx = m_idx % STORAGE_WIDTH;

            if(value ^ INVERT)
                m_parent.m_storage[byteIdx] |= (storage_t{1} << bitIdx);
            else
                m_parent.m_storage[byteIdx] &= ~(storage_t{1} << bitIdx);

            return *this;
        }

        inline operator bool() const
        {
            const auto byteIdx = m_idx / STORAGE_WIDTH;
            const auto bitIdx = m_idx % STORAGE_WIDTH;

            return ((m_parent.m_storage[byteIdx] >> bitIdx) & 1) ^ INVERT;
        }

      private:
        BitStorage& m_parent;
        const std::size_t m_idx;
    };

  public:
    explicit BitStorage(const std::size_t size, const bool defaultValue = false) : m_size(ceildiv(size, STORAGE_WIDTH)), m_storage(new storage_t[m_size])
    {
        for(auto i = std::size_t{0}; i < m_size; ++i) {
            m_storage[i] = (defaultValue ^ INVERT) ? ~storage_t{} : storage_t{};
        }
    }

    ~BitStorage() { delete[] m_storage; }

    inline BitReference operator[](const std::size_t idx) { return BitReference(*this, idx); }

  private:
    const std::size_t m_size;
    storage_t* m_storage = nullptr;
};

class PrimeSieve {
    using sieve_size_t = std::size_t;
    using bit_storage_t = BitStorage;

  public:
    PrimeSieve(const sieve_size_t sieveSize) : m_sieveSize(sieveSize), m_bits(sieveSize / 2 + 1, true) {}

    ~PrimeSieve() {}

    inline void runSieve()
    {
        for(auto i = sieve_size_t{1}; i * i <= m_sieveSize; ++i) {
            for(auto num = i; num < m_sieveSize / 2; ++num) {
                if(m_bits[num]) {
                    i = num;
                    break;
                }
            }
            const auto factor = i * 2 + 1;
            for(auto num = factor * factor / 2; num < m_sieveSize / 2; num += factor)
                m_bits[num] = false;
        }
    }

    inline void printResults(double duration, int passes)
    {
        if(!validateResults())
            std::printf("Error: Results not valid!\n");

        std::printf("BlackMark;%d;%f;1;algorithm=base,faithful=yes,bits=1\n", passes, duration);
    }

    inline std::size_t countPrimes()
    {
        auto count = std::size_t{1};
        for(auto i = sieve_size_t{1}; i < m_sieveSize / 2; ++i)
            if(m_bits[i])
                ++count;
        return count;
    }

  private:
    const sieve_size_t m_sieveSize = 0;
    bit_storage_t m_bits;

    // Historical data for validating our results - the number of primes to be found under some limit, such as 168 primes under 1000
    static const std::map<const sieve_size_t, const std::size_t> m_resultsDictionary;

    inline bool validateResults()
    {
        auto result = m_resultsDictionary.find(m_sieveSize);
        if(m_resultsDictionary.end() == result)
            return false;
        return result->second == countPrimes();
    }
};

// clang-format off
const std::map<const PrimeSieve::sieve_size_t, const std::size_t> PrimeSieve::m_resultsDictionary = {
    {            10,           4},
    {           100,          25},
    {         1'000,         168},
    {        10'000,       1'229},
    {       100'000,       9'592},
    {     1'000'000,      78'498},
    {    10'000'000,     664'579},
    {   100'000'000,   5'761'455},
    { 1'000'000'000,  50'847'534},
    {10'000'000'000, 455'052'511},
};
// clang-format on

int main()
{
    auto passes = 0;
    const auto start = std::chrono::high_resolution_clock::now();

    while(true) {
        PrimeSieve sieve(1'000'000);
        sieve.runSieve();
        ++passes;
        if(const auto end = std::chrono::high_resolution_clock::now(); std::chrono::duration_cast<std::chrono::seconds>(end - start).count() >= 5) {
            sieve.printResults(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1'000'000.0, passes);
            break;
        }
    }
}
