#ifndef SIMDJSON_PORTABILITY_H
#define SIMDJSON_PORTABILITY_H

#ifdef _MSC_VER
/* Microsoft C/C++-compatible compiler */
#include <intrin.h>
#include <iso646.h>
#include <cstdint>

static inline bool add_overflow(uint64_t value1, uint64_t value2, uint64_t *result) {
	return _addcarry_u64(0, value1, value2, reinterpret_cast<unsigned __int64 *>(result));
}

#  pragma intrinsic(_umul128)
static inline bool mul_overflow(uint64_t value1, uint64_t value2, uint64_t *result) {
	uint64_t high;
	*result = _umul128(value1, value2, &high);
	return high;
}

static inline int trailingzeroes(uint64_t input_num) {
    return _tzcnt_u64(input_num);
}

static inline int leadingzeroes(uint64_t  input_num) {
    return _lzcnt_u64(input_num);
}

static inline int hamming(uint64_t input_num) {
#ifdef _WIN64  // highly recommended!!!
	return (int)__popcnt64(input_num);
#else  // if we must support 32-bit Windows
	return (int)(__popcnt((uint32_t)input_num) +
		__popcnt((uint32_t)(input_num >> 32)));
#endif
}

#else
#include <cstdint>
#include <cstdlib>

#if defined(__BMI2__) || defined(__POPCOUNT__) || defined(__SSE4_2__)
#include <emmintrin.h>
#include <immintrin.h>
#include <smmintrin.h>
#include <tmmintrin.h>
#include <wmmintrin.h>
#endif

static inline bool add_overflow(uint64_t  value1, uint64_t  value2, uint64_t *result) {
	return __builtin_uaddll_overflow(value1, value2, (unsigned long long*)result);
}
static inline bool mul_overflow(uint64_t  value1, uint64_t  value2, uint64_t *result) {
	return __builtin_umulll_overflow(value1, value2, (unsigned long long *)result);
}

/* result might be undefined when input_num is zero */
static inline int trailingzeroes(uint64_t input_num) {
#ifdef __BMI2__
	return _tzcnt_u64(input_num);
#else
	return __builtin_ctzll(input_num);
#endif
}

/* result might be undefined when input_num is zero */
static inline int leadingzeroes(uint64_t  input_num) {
#ifdef __BMI2__
	return _lzcnt_u64(input_num);
#else
	return __builtin_clzll(input_num);
#endif
}

/* result might be undefined when input_num is zero */
static inline int hamming(uint64_t input_num) {
#ifdef __POPCOUNT__
	return _popcnt64(input_num);
#else
	return __builtin_popcountll(input_num);
#endif
}

#endif // _MSC_VER


// portable version of  posix_memalign
static inline void *aligned_malloc(size_t alignment, size_t size) {
	void *p;
#ifdef _MSC_VER
	p = _aligned_malloc(size, alignment);
#elif defined(__MINGW32__) || defined(__MINGW64__)
	p = __mingw_aligned_malloc(size, alignment);
#else
	// somehow, if this is used before including "x86intrin.h", it creates an
	// implicit defined warning.
	if (posix_memalign(&p, alignment, size) != 0) { return nullptr; }
#endif
	return p;
}

#ifdef __SSE4_2__

static __m128i inline _mm_loadu2_m128(__m64 const *__addr_hi,
                                          __m64 const *__addr_lo) {
    __m128i __v128 = _mm_set_epi64(*__addr_hi, *__addr_lo);

    return __v128;
}

static inline void _mm_storeu2_m64(__m64 *__addr_hi, __m64 *__addr_lo,
                                        __m128i __a) {
    *__addr_hi = (__m64) _mm_extract_epi64(__a, 1);
    *__addr_lo = (__m64) _mm_extract_epi64(__a, 0);
}

#endif // SSE4_2

static inline void aligned_free(void *memblock) {
    if(memblock == nullptr) { return; }
#ifdef _MSC_VER
    _aligned_free(memblock);
#elif defined(__MINGW32__) || defined(__MINGW64__)
    __mingw_aligned_free(memblock);
#else
    free(memblock);
#endif
}

#endif // SIMDJSON_PORTABILITY_H
