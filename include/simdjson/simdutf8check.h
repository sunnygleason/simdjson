
#ifndef SIMDJSON_SIMDUTF8CHECK_H
#define SIMDJSON_SIMDUTF8CHECK_H


#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "simdjson/portability.h"
/*
 * legal utf-8 byte sequence
 * http://www.unicode.org/versions/Unicode6.0.0/ch03.pdf - page 94
 *
 *  Code Points        1st       2s       3s       4s
 * U+0000..U+007F     00..7F
 * U+0080..U+07FF     C2..DF   80..BF
 * U+0800..U+0FFF     E0       A0..BF   80..BF
 * U+1000..U+CFFF     E1..EC   80..BF   80..BF
 * U+D000..U+D7FF     ED       80..9F   80..BF
 * U+E000..U+FFFF     EE..EF   80..BF   80..BF
 * U+10000..U+3FFFF   F0       90..BF   80..BF   80..BF
 * U+40000..U+FFFFF   F1..F3   80..BF   80..BF   80..BF
 * U+100000..U+10FFFF F4       80..8F   80..BF   80..BF
 *
 */

// all byte values must be no larger than 0xF4


#ifdef __SSE4_2__

/*****************************/
static inline __m128i push_last_byte_of_a_to_b(__m128i a, __m128i b) {
    //
    // FIXME: need help figuring out a more SSE way to do this, was:
    //  return _mm_alignr_epi8(b, _mm_permute2x64_si128(a, b, 0x21), 15);
    //
    __m128i a_mask = _mm_setr_epi8(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFF);
    __m128i b_mask = _mm_setr_epi8(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0);

    return _mm_or_si128(_mm_and_si128(a, a_mask), _mm_and_si128(b, b_mask));
}

static inline __m128i push_last_2bytes_of_a_to_b(__m128i a, __m128i b) {
    //
    // FIXME: need help figuring out a more SSE way to do this, was:
    //  return _mm_alignr_epi8(b, _mm_permute2x64_si128(a, b, 0x21), 15);
    //
    __m128i a_mask = _mm_setr_epi8(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFF, 0xFF);
    __m128i b_mask = _mm_setr_epi8(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0);

    return _mm_or_si128(_mm_and_si128(a, a_mask), _mm_and_si128(b, b_mask));
}

// all byte values must be no larger than 0xF4
static inline void avxcheckSmallerThan0xF4(__m128i current_bytes,
                                           __m128i *has_error) {
  // unsigned, saturates to 0 below max
  *has_error = _mm_or_si128(
      *has_error, _mm_subs_epu8(current_bytes, _mm_set1_epi8(0xF4)));
}

static inline __m128i avxcontinuationLengths(__m128i high_nibbles) {
  return _mm_shuffle_epi8(
      _mm_setr_epi8(1, 1, 1, 1, 1, 1, 1, 1, // 0xxx (ASCII)
                       0, 0, 0, 0,             // 10xx (continuation)
                       2, 2,                   // 110x
                       3,                      // 1110
                       4                       // 1111, next should be 0 (not checked here)
                       ),
      high_nibbles);
}

static inline __m128i avxcarryContinuations(__m128i initial_lengths,
                                            __m128i previous_carries) {

  __m128i right1 = _mm_subs_epu8(
      push_last_byte_of_a_to_b(previous_carries, initial_lengths),
      _mm_set1_epi8(1));
  __m128i sum = _mm_add_epi8(initial_lengths, right1);

  __m128i right2 = _mm_subs_epu8(
      push_last_2bytes_of_a_to_b(previous_carries, sum), _mm_set1_epi8(2));
  return _mm_add_epi8(sum, right2);
}

static inline void avxcheckContinuations(__m128i initial_lengths,
                                         __m128i carries, __m128i *has_error) {

  // overlap || underlap
  // carry > length && length > 0 || !(carry > length) && !(length > 0)
  // (carries > length) == (lengths > 0)
  __m128i overunder = _mm_cmpeq_epi8(
      _mm_cmpgt_epi8(carries, initial_lengths),
      _mm_cmpgt_epi8(initial_lengths, _mm_setzero_si128()));

  *has_error = _mm_or_si128(*has_error, overunder);
}

// when 0xED is found, next byte must be no larger than 0x9F
// when 0xF4 is found, next byte must be no larger than 0x8F
// next byte must be continuation, ie sign bit is set, so signed < is ok
static inline void avxcheckFirstContinuationMax(__m128i current_bytes,
                                                __m128i off1_current_bytes,
                                                __m128i *has_error) {
  __m128i maskED =
      _mm_cmpeq_epi8(off1_current_bytes, _mm_set1_epi8(0xED));
  __m128i maskF4 =
      _mm_cmpeq_epi8(off1_current_bytes, _mm_set1_epi8(0xF4));

  __m128i badfollowED = _mm_and_si128(
      _mm_cmpgt_epi8(current_bytes, _mm_set1_epi8(0x9F)), maskED);
  __m128i badfollowF4 = _mm_and_si128(
      _mm_cmpgt_epi8(current_bytes, _mm_set1_epi8(0x8F)), maskF4);

  *has_error =
      _mm_or_si128(*has_error, _mm_or_si128(badfollowED, badfollowF4));
}

// map off1_hibits => error condition
// hibits     off1    cur
// C       => < C2 && true
// E       => < E1 && < A0
// F       => < F1 && < 90
// else      false && false
static inline void avxcheckOverlong(__m128i current_bytes,
                                    __m128i off1_current_bytes, __m128i hibits,
                                    __m128i previous_hibits,
                                    __m128i *has_error) {
  __m128i off1_hibits = push_last_byte_of_a_to_b(previous_hibits, hibits);
  __m128i initial_mins = _mm_shuffle_epi8(
      _mm_setr_epi8(-128, -128, -128, -128, -128, -128, -128, -128,
                       -128, -128, -128, -128, // 10xx => false
                       0xC2, -128,             // 110x
                       0xE1,                   // 1110
                       0xF1                    // 1111
                       ),
      off1_hibits);

  __m128i initial_under = _mm_cmpgt_epi8(initial_mins, off1_current_bytes);

  __m128i second_mins = _mm_shuffle_epi8(
      _mm_setr_epi8(-128, -128, -128, -128, -128, -128, -128, -128,
                       -128, -128, -128, -128, // 10xx => false
                       127, 127,               // 110x => true
                       0xA0,                   // 1110
                       0x90                    // 1111
      ),
      off1_hibits);
  __m128i second_under = _mm_cmpgt_epi8(second_mins, current_bytes);
  *has_error = _mm_or_si128(*has_error,
                               _mm_and_si128(initial_under, second_under));
}

struct avx_processed_utf_bytes {
  __m128i rawbytes;
  __m128i high_nibbles;
  __m128i carried_continuations;
};

static inline void avx_count_nibbles(__m128i bytes,
                                     struct avx_processed_utf_bytes *answer) {
  answer->rawbytes = bytes;
  answer->high_nibbles =
      _mm_and_si128(_mm_srli_epi16(bytes, 4), _mm_set1_epi8(0x0F));
}

// check whether the current bytes are valid UTF-8
// at the end of the function, previous gets updated
static inline struct avx_processed_utf_bytes
avxcheckUTF8Bytes(__m128i current_bytes,
                  struct avx_processed_utf_bytes *previous,
                  __m128i *has_error) {
  struct avx_processed_utf_bytes pb{};
  avx_count_nibbles(current_bytes, &pb);

  avxcheckSmallerThan0xF4(current_bytes, has_error);

  __m128i initial_lengths = avxcontinuationLengths(pb.high_nibbles);

  pb.carried_continuations =
      avxcarryContinuations(initial_lengths, previous->carried_continuations);

  avxcheckContinuations(initial_lengths, pb.carried_continuations, has_error);

  __m128i off1_current_bytes =
      push_last_byte_of_a_to_b(previous->rawbytes, pb.rawbytes);
  avxcheckFirstContinuationMax(current_bytes, off1_current_bytes, has_error);

  avxcheckOverlong(current_bytes, off1_current_bytes, pb.high_nibbles,
                   previous->high_nibbles, has_error);
  return pb;
}

#else // __AVX2__
#warning "We require AVX2 support!"
#endif // __AVX2__
#endif
