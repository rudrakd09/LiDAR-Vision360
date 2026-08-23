"""Unit tests for `datasources.stm32.sequence.SequenceValidator` against synthetic sequence-number
streams. PARSER/UNIT TESTS ONLY -- not hardware validation.
"""

from datasources.stm32.sequence import SequenceValidator


class TestNoWraparound:
    def test_first_sequence_number(self):
        v = SequenceValidator()
        result = v.validate(0)
        assert result.is_first is True
        assert result.dropped_count == 0
        assert result.is_duplicate_or_out_of_order is False

    def test_consecutive_increments_report_no_drops(self):
        v = SequenceValidator()
        v.validate(0)
        for seq in range(1, 6):
            result = v.validate(seq)
            assert result.dropped_count == 0
            assert result.is_duplicate_or_out_of_order is False
        assert v.dropped_count == 0
        assert v.received_count == 6

    def test_gap_reports_dropped_count(self):
        v = SequenceValidator()
        v.validate(10)
        result = v.validate(15)  # 11,12,13,14 missing
        assert result.dropped_count == 4
        assert v.dropped_count == 4

    def test_duplicate_sequence_number(self):
        v = SequenceValidator()
        v.validate(5)
        result = v.validate(5)
        assert result.is_duplicate_or_out_of_order is True
        assert v.duplicate_or_out_of_order_count == 1

    def test_out_of_order_older_sequence_number(self):
        v = SequenceValidator()
        v.validate(10)
        result = v.validate(7)
        assert result.is_duplicate_or_out_of_order is True

    def test_out_of_order_does_not_move_last_seen_forward(self):
        v = SequenceValidator()
        v.validate(10)
        v.validate(3)  # out of order, ignored for "last" tracking
        result = v.validate(11)  # should still be treated as the very next after 10
        assert result.dropped_count == 0
        assert result.is_duplicate_or_out_of_order is False


class TestWraparound:
    def test_wrap_from_max_to_zero_is_not_a_drop(self):
        v = SequenceValidator(modulus=256)
        v.validate(255)
        result = v.validate(0)
        assert result.dropped_count == 0
        assert result.is_duplicate_or_out_of_order is False

    def test_wrap_with_a_genuine_gap(self):
        v = SequenceValidator(modulus=256)
        v.validate(254)
        result = v.validate(1)  # 255, 0 missing
        assert result.dropped_count == 2

    def test_small_backward_step_after_wrap_is_out_of_order_not_a_huge_drop(self):
        v = SequenceValidator(modulus=256)
        v.validate(10)
        result = v.validate(9)  # one step backward, not ~255 frames dropped
        assert result.is_duplicate_or_out_of_order is True
        assert result.dropped_count == 0
