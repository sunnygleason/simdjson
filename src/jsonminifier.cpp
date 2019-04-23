#include "simdjson/portability.h"
#include <cstdint>
#ifndef __SSE4_2__


static uint8_t jump_table[256 * 3] = {
    0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0,
    1, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1,
    1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1,
    0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0,
    1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1,
    1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1,
    0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0,
    1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1,
    1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1,
    0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0,
    1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1,
    1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1,
    0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0,
    1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1,
    1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1,
    0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0,
    1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1,
    1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1,
    0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0,
    1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1,
    1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1,
    0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0,
    1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1,
    1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1,
    0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0,
    1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1,
    1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1,
    0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0,
    1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1,
    1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1,
    0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1,
};

size_t jsonminify(const unsigned char *bytes, size_t howmany,
                  unsigned char *out) {
  size_t i = 0, pos = 0;
  uint8_t quote = 0;
  uint8_t nonescape = 1;

  while (i < howmany) {
    unsigned char c = bytes[i];
    uint8_t *meta = jump_table + 3 * c;

    quote = quote ^ (meta[0] & nonescape);
    out[pos] = c;
    pos += meta[2] | quote;

    i += 1;
    nonescape = (~nonescape) | (meta[1]);
  }
  return pos;
}

#else

#include "simdjson/simdprune_tables.h"
#include <cstring>

// a straightforward comparison of a mask against input.
static uint64_t cmp_mask_against_input_mini(__m128i input_lo, __m128i input_hi,
                                            __m128i mask) {
  __m128i cmp_res_0 = _mm_cmpeq_epi8(input_lo, mask);
  uint64_t res_0 = static_cast<uint32_t>(_mm_movemask_epi8(cmp_res_0));
  __m128i cmp_res_1 = _mm_cmpeq_epi8(input_hi, mask);
  uint64_t res_1 = _mm_movemask_epi8(cmp_res_1);
  return res_0 | (res_1 << 16);
}

// take input from buf and remove useless whitespace, input and output can be
// the same, result is null terminated, return the string length (minus the null termination)
size_t jsonminify(const uint8_t *buf, size_t len, uint8_t *out) {
  // Useful constant masks
  const uint32_t even_bits = 0x55555555ULL;
  const uint32_t odd_bits = ~even_bits;
  uint8_t *initout(out);
  uint32_t prev_iter_ends_odd_backslash =
      0ULL;                               // either 0 or 1, but a 64-bit value
  uint32_t prev_iter_inside_quote = 0ULL; // either all zeros or all ones
  size_t idx = 0;
  if (len >= 32) {
    size_t avxlen = len - 31;

    for (; idx < avxlen; idx += 32) {
      __m128i input_lo = _mm_loadu_si128(reinterpret_cast<const __m128i *>(buf + idx + 0));
      __m128i input_hi = _mm_loadu_si128(reinterpret_cast<const __m128i *>(buf + idx + 16));
      uint32_t bs_bits = cmp_mask_against_input_mini(input_lo, input_hi,
                                                     _mm_set1_epi8('\\'));
      uint64_t start_edges = bs_bits & ~(bs_bits << 1);
      uint64_t even_start_mask = even_bits ^ prev_iter_ends_odd_backslash;
      uint64_t even_starts = start_edges & even_start_mask;
      uint64_t odd_starts = start_edges & ~even_start_mask;
      uint64_t even_carries = bs_bits + even_starts;
      uint64_t odd_carries;
      bool iter_ends_odd_backslash = add_overflow(
          bs_bits, odd_starts, &odd_carries);
      odd_carries |= prev_iter_ends_odd_backslash;
      prev_iter_ends_odd_backslash = iter_ends_odd_backslash ? 0x1ULL : 0x0ULL;
      uint32_t even_carry_ends = even_carries & ~bs_bits;
      uint32_t odd_carry_ends = odd_carries & ~bs_bits;
      uint32_t even_start_odd_end = even_carry_ends & odd_bits;
      uint32_t odd_start_even_end = odd_carry_ends & even_bits;
      uint32_t odd_ends = even_start_odd_end | odd_start_even_end;
      uint32_t quote_bits = cmp_mask_against_input_mini(input_lo, input_hi,
                                                        _mm_set1_epi8('"'));
      quote_bits = quote_bits & ~odd_ends;
      uint32_t quote_mask = _mm_cvtsi128_si64(_mm_clmulepi64_si128(
          _mm_set_epi64x(0ULL, quote_bits), _mm_set1_epi8(0xFF), 0));
      quote_mask ^= prev_iter_inside_quote;
      prev_iter_inside_quote = static_cast<uint64_t>(static_cast<int32_t>(quote_mask) >> 31);// might be undefined behavior, should be fully defined in C++20, ok according to John Regher from Utah University
      const __m128i low_nibble_mask = _mm_setr_epi8(
          //  0                           9  a   b  c  d
          16, 0, 0, 0, 0, 0, 0, 0, 0, 8, 12, 1, 2, 9, 0, 0);
      const __m128i high_nibble_mask = _mm_setr_epi8(
          //  0     2   3     5     7
          8, 0, 18, 4, 0, 1, 0, 1, 0, 0, 0, 3, 2, 1, 0, 0);
      __m128i whitespace_shufti_mask = _mm_set1_epi8(0x18);
      __m128i v_lo = _mm_and_si128(
          _mm_shuffle_epi8(low_nibble_mask, input_lo),
          _mm_shuffle_epi8(high_nibble_mask,
                              _mm_and_si128(_mm_srli_epi32(input_lo, 4),
                                               _mm_set1_epi8(0x7f))));

      __m128i v_hi = _mm_and_si128(
          _mm_shuffle_epi8(low_nibble_mask, input_hi),
          _mm_shuffle_epi8(high_nibble_mask,
                              _mm_and_si128(_mm_srli_epi32(input_hi, 4),
                                               _mm_set1_epi8(0x7f))));
      __m128i tmp_ws_lo = _mm_cmpeq_epi8(
          _mm_and_si128(v_lo, whitespace_shufti_mask), _mm_set1_epi8(0));
      __m128i tmp_ws_hi = _mm_cmpeq_epi8(
          _mm_and_si128(v_hi, whitespace_shufti_mask), _mm_set1_epi8(0));

      uint64_t ws_res_0 = static_cast<uint32_t>(_mm_movemask_epi8(tmp_ws_lo));
      uint64_t ws_res_1 = _mm_movemask_epi8(tmp_ws_hi);
      uint64_t whitespace = ~(ws_res_0 | (ws_res_1 << 32));
      whitespace &= ~quote_mask;

      int mask1 = whitespace & 0xFFFF;
      int mask2 = (whitespace >> 16) & 0xFFFF;
      int mask3 = (whitespace >> 32) & 0xFFFF;
      int mask4 = (whitespace >> 48) & 0xFFFF;
      int pop1 = hamming((~whitespace) & 0xFFFF);
      int pop2 = hamming((~whitespace) & UINT64_C(0xFFFFFFFF));
      int pop3 = hamming((~whitespace) & UINT64_C(0xFFFFFFFFFFFF));
      int pop4 = hamming((~whitespace));

//
//      FIXME: need help with this...
//
//      __m128i vmask1 =
//          _mm_set_epi64(
//                  reinterpret_cast<const uint64_t *>(mask64_epi8) + (mask2 & 0x7FFF),
//                  reinterpret_cast<const uint64_t *>(mask64_epi8) + (mask1 & 0x7FFF));
//      __m128i vmask2 =
//          _mm_set_epi64(
//                  reinterpret_cast<const uint64_t *>(mask64_epi8) + (mask4 & 0x7FFF),
//                  reinterpret_cast<const uint64_t *>(mask64_epi8) + (mask3 & 0x7FFF));
//
//      __m128i result1 = _mm_shuffle_epi8(input_lo, vmask1);
//      __m128i result2 = _mm_shuffle_epi8(input_hi, vmask2);
//
//      _mm_storeu_m64(reinterpret_cast<uint64_t *>(out + pop1), reinterpret_cast<uint64_t *>(out), result1);
//      _mm_store_m128(reinterpret_cast<uint64_t *>(out + pop3), reinterpret_cast<uint64_t *>(out + pop2), result2);

      out += pop4;
    }
  }
  // we finish off the job... copying and pasting the code is not ideal here,
  // but it gets the job done.
  if (idx < len) {
    uint8_t buffer[32];
    memset(buffer, 0, 32);
    memcpy(buffer, buf + idx, len - idx);
    __m128i input_lo = _mm_loadu_si128(reinterpret_cast<const __m128i *>(buffer));
    __m128i input_hi = _mm_loadu_si128(reinterpret_cast<const __m128i *>(buffer + 16));
    uint64_t bs_bits =
        cmp_mask_against_input_mini(input_lo, input_hi, _mm_set1_epi8('\\'));
    uint64_t start_edges = bs_bits & ~(bs_bits << 1);
    uint64_t even_start_mask = even_bits ^ prev_iter_ends_odd_backslash;
    uint64_t even_starts = start_edges & even_start_mask;
    uint64_t odd_starts = start_edges & ~even_start_mask;
    uint64_t even_carries = bs_bits + even_starts;
    uint64_t odd_carries;
    //bool iter_ends_odd_backslash =
	add_overflow( bs_bits, odd_starts, &odd_carries);
    odd_carries |= prev_iter_ends_odd_backslash;
    //prev_iter_ends_odd_backslash = iter_ends_odd_backslash ? 0x1ULL : 0x0ULL; // we never use it
    uint64_t even_carry_ends = even_carries & ~bs_bits;
    uint64_t odd_carry_ends = odd_carries & ~bs_bits;
    uint64_t even_start_odd_end = even_carry_ends & odd_bits;
    uint64_t odd_start_even_end = odd_carry_ends & even_bits;
    uint64_t odd_ends = even_start_odd_end | odd_start_even_end;
    uint64_t quote_bits =
        cmp_mask_against_input_mini(input_lo, input_hi, _mm_set1_epi8('"'));
    quote_bits = quote_bits & ~odd_ends;
    uint64_t quote_mask = _mm_cvtsi128_si64(_mm_clmulepi64_si128(
        _mm_set_epi64x(0ULL, quote_bits), _mm_set1_epi8(0xFF), 0));
    quote_mask ^= prev_iter_inside_quote;
    // prev_iter_inside_quote = (uint64_t)((int64_t)quote_mask >> 63);// we don't need this anymore

    __m128i mask_20 = _mm_set1_epi8(0x20); // c==32
    __m128i mask_70 =
        _mm_set1_epi8(0x70); // adding 0x70 does not check low 4-bits
    // but moves any value >= 16 above 128

    __m128i lut_cntrl = _mm_setr_epi8(
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00,
        0x00, 0xFF, 0x00, 0x00);

    __m128i tmp_ws_lo = _mm_or_si128(
        _mm_cmpeq_epi8(mask_20, input_lo),
        _mm_shuffle_epi8(lut_cntrl, _mm_adds_epu8(mask_70, input_lo)));
    __m128i tmp_ws_hi = _mm_or_si128(
        _mm_cmpeq_epi8(mask_20, input_hi),
        _mm_shuffle_epi8(lut_cntrl, _mm_adds_epu8(mask_70, input_hi)));
    uint64_t ws_res_0 = static_cast<uint32_t>(_mm_movemask_epi8(tmp_ws_lo));
    uint64_t ws_res_1 = _mm_movemask_epi8(tmp_ws_hi);
    uint64_t whitespace = (ws_res_0 | (ws_res_1 << 32));
    whitespace &= ~quote_mask;

    if (len - idx < 32) {
      whitespace |= UINT64_C(0xFFFFFFFFFFFFFFFF) << (len - idx);
    }
    int mask1 = whitespace & 0xFFFF;
    int mask2 = (whitespace >> 16) & 0xFFFF;
    int mask3 = (whitespace >> 32) & 0xFFFF;
    int mask4 = (whitespace >> 48) & 0xFFFF;
    int pop1 = hamming((~whitespace) & 0xFFFF);
    int pop2 = hamming((~whitespace) & UINT64_C(0xFFFFFFFF));
    int pop3 = hamming((~whitespace) & UINT64_C(0xFFFFFFFFFFFF));
    int pop4 = hamming((~whitespace));

//    __m128i vmask1 =
//          _mm_set_epi64(
//                  reinterpret_cast<const __m64 *>(mask64_epi8) + (mask2 & 0x7FFF),
//                  reinterpret_cast<const __m64 *>(mask64_epi8) + (mask1 & 0x7FFF));
//    __m128i vmask2 =
//          _mm_set_epi64(
//                  reinterpret_cast<const __m64 *>(mask64_epi8) + (mask4 & 0x7FFF),
//                  reinterpret_cast<const __m64 *>(mask64_epi8) + (mask3 & 0x7FFF));
//
//    __m128i result1 = _mm_shuffle_epi8(input_lo, vmask1);
//    __m128i result2 = _mm_shuffle_epi8(input_hi, vmask2);
//    _mm_storeu2_m64i(reinterpret_cast<__m128i *>(buffer + pop1), reinterpret_cast<__m128i *>(buffer), result1);
//    _mm_storeu2_m64i(reinterpret_cast<__m128i *>(buffer + pop3), reinterpret_cast<__m128i *>(buffer + pop2), result2);

    memcpy(out, buffer, pop4);
    out += pop4;
  }
  *out = '\0';// NULL termination
  return out - initout;
}

#endif
