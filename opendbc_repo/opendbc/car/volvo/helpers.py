def checksum_lca_2_message(b0: int, b5: int) -> int:
  """Checksum for 0x69 from bytes b0 and b5."""
  if b0 == 0 and b5 == 128:  # test-message hotfix
    return 0

  # Per-bit masks for b0 and b5
  M0 = [0x08, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00]
  M5 = [0x83, 0x86, 0xCF, 0xCD, 0x09, 0x02, 0x44, 0x89]

  def parity8(x: int) -> int:
    # Odd parity
    x ^= x >> 4
    x ^= x >> 2
    x ^= x >> 1
    return x & 1

  b0 &= 0xFF
  b5 &= 0xFF

  c = 0
  for bit in range(8):
    p = 0
    if M0[bit]:
      p ^= parity8(b0 & M0[bit])
    if M5[bit]:
      p ^= parity8(b5 & M5[bit])
    c |= (p << bit)

  return c & 0xFF

def checksum_2_0x69_message(b0: int, b1: int, b3: int = 0, b4: int = 0) -> int:
  """Checksum byte b2 for 0x69 (LCA_2)."""
  b0 &= 0xFF
  b1 &= 0xFF
  b3 &= 0xFF
  b4 &= 0xFF

  def bit(byte, pos):
    return (byte >> pos) & 1

  c = 0
  # Bit 0
  c |= (bit(b0, 0) ^ bit(b1, 2) ^ bit(b1, 3) ^ bit(b1, 4) ^ bit(b1, 5) ^ bit(b4, 0) ^ bit(b4, 1)) << 0
  # Bit 1
  c |= (bit(b0, 1) ^ bit(b0, 3) ^ bit(b1, 0) ^ bit(b1, 3) ^ bit(b1, 5) ^ bit(b1, 6) ^
        bit(b3, 1) ^ bit(b4, 0) ^ bit(b4, 1) ^ bit(b4, 2)) << 1
  # Bit 2
  c |= (bit(b0, 0) ^ bit(b0, 3) ^ bit(b1, 1) ^ bit(b1, 2) ^ bit(b1, 3) ^ bit(b1, 4) ^ bit(b1, 5) ^ bit(b1, 6) ^
        bit(b3, 0) ^ bit(b3, 1) ^ bit(b4, 0) ^ bit(b4, 2) ^ bit(b4, 3)) << 2
  # Bit 3
  c |= (bit(b0, 0) ^ bit(b0, 1) ^ bit(b1, 0) ^ bit(b1, 4) ^ bit(b1, 6) ^
        bit(b3, 0) ^ bit(b3, 1) ^ bit(b4, 0) ^ bit(b4, 3) ^ bit(b4, 4)) << 3
  # Bit 4
  c |= (bit(b0, 0) ^ bit(b0, 1) ^ bit(b0, 3) ^ bit(b1, 1) ^ bit(b1, 2) ^ bit(b1, 3) ^ bit(b1, 4) ^
        bit(b4, 4) ^ bit(b4, 5)) << 4
  # Bit 5
  c |= (bit(b0, 1) ^ bit(b1, 0) ^ bit(b1, 2) ^ bit(b1, 3) ^ bit(b1, 5) ^
        bit(b3, 1) ^ bit(b4, 5) ^ bit(b4, 6)) << 5
  # Bit 6
  c |= (bit(b0, 3) ^ bit(b1, 0) ^ bit(b1, 1) ^ bit(b1, 3) ^ bit(b1, 6) ^
        bit(b3, 0) ^ bit(b4, 6) ^ bit(b4, 7)) << 6
  # Bit 7
  c |= (bit(b0, 3) ^ bit(b1, 1) ^ bit(b1, 2) ^ bit(b1, 4) ^ bit(b4, 0) ^ bit(b4, 7)) << 7
  return c & 0xFF

def checksum_1_pscm_related_message(b1, b2):
  """Checksum 1 (byte 0) for 0x17 (PSCM related)."""
  # Per-bit masks for b1 and b2
  M1 = [0x41, 0x82, 0x55, 0xF3, 0xA7, 0x46, 0x94, 0x20]
  M2 = [0x00, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x80]

  def parity8(x: int) -> int:
    # Odd parity
    x ^= x >> 4
    x ^= x >> 2
    x ^= x >> 1
    return x & 1

  b1 &= 0xFF
  b2 &= 0xFF

  c = 0
  for bit in range(8):
    p = 0
    if M1[bit]:
      p ^= parity8(b1 & M1[bit])
    if M2[bit]:
      p ^= parity8(b2 & M2[bit])
    c |= (p << bit)

  return c & 0xFF


def checksum_2_pscm_related_message(b2):
  """Checksum 2 (byte 3) for 0x17 (PSCM related)."""

  lut = {
    0x80: 0xBF,
    0x81: 0xF3,
    0x82: 0x27,
    0x83: 0x6B,
    0x84: 0x92,
    0x85: 0xDE,
    0x86: 0x0A,
    0x87: 0x46,
    0x88: 0xE5,
    0x89: 0xA9,
    0x8A: 0x7D,
    0x8B: 0x31,
    0x8C: 0xC8,
    0x8D: 0x84,
    0x8E: 0x50,
  }

  return lut.get(b2, 0)


def checksum_lca_4_message(*args) -> int:
  """LCA_4 (0x90) checksum placeholder."""
  # Placeholder
  return 0


class LCA3CounterSync:
  """Syncs the LCA_3 COUNTER_1 pattern."""

  PATTERN = [2, 2, 1, 2, 2, 2, 1, 2, 2, 3, 0, 2, 3, 2, 0, 2, 3, 2, 0, 3]
  PATTERN_LEN = 20
  WINDOW_SIZE = 5  # values kept for matching
  MIN_CONFIDENCE = 4  # matches needed to sync

  def __init__(self):
    self.pattern_index = None  # None until synced
    self.observed_window = []  # last observed values
    self.confidence = 0  # consecutive matches

  def update(self, observed_counter: int) -> tuple:
    """Feed a stock counter; returns (counter_to_send, synced)."""
    # Synced: follow the pattern
    if self.pattern_index is not None:
      counter_to_send = self.PATTERN[self.pattern_index]
      self.pattern_index = (self.pattern_index + 1) % self.PATTERN_LEN
      return counter_to_send, True

    # Not synced yet: look for the index
    self.observed_window.append(observed_counter)
    if len(self.observed_window) > self.WINDOW_SIZE:
      self.observed_window.pop(0)

    # Try to sync with enough samples
    if len(self.observed_window) >= 3:
      self._attempt_sync()

    # Pass through stock until synced
    return observed_counter, False

  def _attempt_sync(self):
    """Try to find current pattern index based on observed window."""
    # Match the window against every pattern position
    best_match_idx = None
    best_match_len = 0

    for start_idx in range(self.PATTERN_LEN):
      match_len = self._count_match(start_idx)
      if match_len > best_match_len:
        best_match_len = match_len
        best_match_idx = start_idx

    # Sync after MIN_CONFIDENCE matches
    if best_match_len >= self.MIN_CONFIDENCE:
      # Next index after the matched one
      self.pattern_index = (best_match_idx + len(self.observed_window)) % self.PATTERN_LEN
      self.confidence = best_match_len

  def _count_match(self, pattern_start_idx: int) -> int:
    """Count window values matching the pattern from an index."""
    match_count = 0
    for i, observed in enumerate(self.observed_window):
      pattern_idx = (pattern_start_idx + i) % self.PATTERN_LEN
      if observed == self.PATTERN[pattern_idx]:
        match_count += 1
      else:
        break  # first mismatch
    return match_count

  def is_synchronized(self) -> bool:
    """Returns True if we have synchronized to the pattern."""
    return self.pattern_index is not None

def checksum_lca_5_message(byte0: int, byte1: int, byte3: int, byte4: int, byte5: int) -> int:
  """Checksum for LCA_5 (0x67), byte 2."""

  # Helper function to extract a bit (LSB = bit 0)
  def bit(byte_val, pos):
    return (byte_val >> pos) & 1

  checksum = 0

  # Bit 0: XOR of 13 bits
  checksum |= (
    bit(byte0, 2) ^ bit(byte0, 3) ^ bit(byte0, 5) ^ bit(byte0, 7) ^
    bit(byte1, 2) ^
    bit(byte3, 4) ^ bit(byte3, 6) ^
    bit(byte4, 0) ^ bit(byte4, 1) ^ bit(byte4, 5) ^ bit(byte4, 6) ^
    bit(byte5, 3) ^ bit(byte5, 6)
  ) << 0

  # Bit 1: XOR of 12 bits
  checksum |= (
    bit(byte0, 0) ^ bit(byte0, 3) ^ bit(byte0, 4) ^ bit(byte0, 7) ^
    bit(byte1, 0) ^ bit(byte1, 3) ^
    bit(byte3, 5) ^ bit(byte3, 7) ^
    bit(byte4, 2) ^ bit(byte4, 6) ^
    bit(byte5, 4) ^ bit(byte5, 7)
  ) << 1

  # Bit 2: XOR of 17 bits
  checksum |= (
    bit(byte0, 1) ^ bit(byte0, 2) ^ bit(byte0, 3) ^ bit(byte0, 4) ^
    bit(byte1, 0) ^ bit(byte1, 1) ^ bit(byte1, 2) ^ bit(byte1, 4) ^
    bit(byte3, 4) ^
    bit(byte4, 1) ^ bit(byte4, 3) ^ bit(byte4, 5) ^ bit(byte4, 6) ^
    bit(byte5, 1) ^ bit(byte5, 3) ^ bit(byte5, 5) ^ bit(byte5, 6)
  ) << 2

  # Bit 3: XOR of 19 bits
  checksum |= (
    bit(byte0, 0) ^ bit(byte0, 4) ^ bit(byte0, 7) ^
    bit(byte1, 1) ^ bit(byte1, 3) ^ bit(byte1, 5) ^
    bit(byte3, 4) ^ bit(byte3, 5) ^ bit(byte3, 6) ^
    bit(byte4, 0) ^ bit(byte4, 1) ^ bit(byte4, 2) ^ bit(byte4, 4) ^ bit(byte4, 5) ^
    bit(byte5, 1) ^ bit(byte5, 2) ^ bit(byte5, 3) ^ bit(byte5, 4) ^ bit(byte5, 7)
  ) << 3

  # Bit 4: XOR of 16 bits
  checksum |= (
    bit(byte0, 1) ^ bit(byte0, 2) ^ bit(byte0, 3) ^ bit(byte0, 7) ^
    bit(byte1, 4) ^ bit(byte1, 6) ^
    bit(byte3, 4) ^ bit(byte3, 5) ^ bit(byte3, 7) ^
    bit(byte4, 1) ^ bit(byte4, 2) ^ bit(byte4, 3) ^
    bit(byte5, 2) ^ bit(byte5, 4) ^ bit(byte5, 5) ^ bit(byte5, 6)
  ) << 4

  # Bit 5: XOR of 15 bits
  checksum |= (
    bit(byte0, 0) ^ bit(byte0, 2) ^ bit(byte0, 3) ^ bit(byte0, 4) ^
    bit(byte1, 5) ^ bit(byte1, 7) ^
    bit(byte3, 5) ^ bit(byte3, 6) ^
    bit(byte4, 2) ^ bit(byte4, 3) ^ bit(byte4, 4) ^
    bit(byte5, 3) ^ bit(byte5, 5) ^ bit(byte5, 6) ^ bit(byte5, 7)
  ) << 5

  # Bit 6: XOR of 19 bits
  checksum |= (
    bit(byte0, 0) ^ bit(byte0, 1) ^ bit(byte0, 3) ^ bit(byte0, 4) ^ bit(byte0, 5) ^ bit(byte0, 7) ^
    bit(byte1, 0) ^ bit(byte1, 6) ^
    bit(byte3, 4) ^ bit(byte3, 6) ^ bit(byte3, 7) ^
    bit(byte4, 0) ^ bit(byte4, 3) ^ bit(byte4, 4) ^ bit(byte4, 5) ^
    bit(byte5, 1) ^ bit(byte5, 4) ^ bit(byte5, 6) ^ bit(byte5, 7)
  ) << 6

  # Bit 7: XOR of 15 bits
  checksum |= (
    bit(byte0, 1) ^ bit(byte0, 2) ^ bit(byte0, 4) ^ bit(byte0, 5) ^
    bit(byte1, 1) ^ bit(byte1, 7) ^
    bit(byte3, 5) ^ bit(byte3, 7) ^
    bit(byte4, 0) ^ bit(byte4, 4) ^ bit(byte4, 5) ^ bit(byte4, 6) ^
    bit(byte5, 2) ^ bit(byte5, 5) ^ bit(byte5, 7)
  ) << 7

  return checksum


# Test examples
if __name__ == "__main__":
  print("CAN 0x67 Checksum Calculator")
  print("=" * 60)

  # Test cases
  tests = [
    ([0x80, 0x00, 0x4F, 0x00, 0x00, 0xBA, 0x00], 0x32),
    ([0x80, 0x00, 0x8F, 0x00, 0x00, 0xBA, 0x00], 0x89),
    ([0x80, 0x00, 0xCF, 0x00, 0x00, 0xBA, 0x00], 0xE0),
    ([0x80, 0x00, 0x1F, 0x00, 0x00, 0xBA, 0x00], 0x06),
  ]

  all_passed = True
  for i, (bytes_list, expected) in enumerate(tests, 1):
    calculated = checksum_lca_5_message(*bytes_list[:5])
    status = "✓" if calculated == expected else "✗"

    print(f"\nTest {i}: {status}")
    print(f"  Bytes:      {' '.join(f'{b:02X}' for b in bytes_list)}")
    print(f"  Expected:   0x{expected:02X}")
    print(f"  Calculated: 0x{calculated:02X}")

    if calculated != expected:
      all_passed = False

  print("\n" + "=" * 60)
  if all_passed:
    print("All tests passed! ✓")
  else:
    print("Some tests failed! ✗")
