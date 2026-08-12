"""Tests for streaming.framing: encode_message, MessageFramer -- partial messages, multiple
messages per chunk, and malformed-JSON resilience."""

import json

from streaming.framing import MessageFramer, encode_message


class TestEncodeMessage:
    def test_produces_newline_terminated_json(self):
        encoded = encode_message({"a": 1})
        assert encoded.endswith(b"\n")
        assert json.loads(encoded.decode("utf-8").strip()) == {"a": 1}

    def test_no_embedded_newline_in_the_json_itself(self):
        encoded = encode_message({"a": 1, "b": [1, 2, 3]})
        assert encoded.count(b"\n") == 1


class TestMessageFramerCompleteMessage:
    def test_single_complete_message_in_one_feed(self):
        framer = MessageFramer()
        messages = framer.feed(encode_message({"a": 1}))
        assert messages == [{"a": 1}]

    def test_no_leftover_buffer_after_a_complete_message(self):
        framer = MessageFramer()
        framer.feed(encode_message({"a": 1}))
        assert framer.pending_bytes() == 0


class TestMessageFramerPartialMessages:
    def test_message_split_across_two_feeds(self):
        framer = MessageFramer()
        full = encode_message({"a": 1, "b": "hello"})
        split_point = len(full) // 2

        first = framer.feed(full[:split_point])
        assert first == []  # nothing complete yet
        assert framer.pending_bytes() > 0

        second = framer.feed(full[split_point:])
        assert second == [{"a": 1, "b": "hello"}]
        assert framer.pending_bytes() == 0

    def test_message_split_byte_by_byte(self):
        framer = MessageFramer()
        full = encode_message({"x": 42})
        collected = []
        for i in range(len(full)):
            collected.extend(framer.feed(full[i : i + 1]))
        assert collected == [{"x": 42}]


class TestMessageFramerMultipleMessages:
    def test_two_complete_messages_in_one_feed(self):
        framer = MessageFramer()
        chunk = encode_message({"a": 1}) + encode_message({"b": 2})
        messages = framer.feed(chunk)
        assert messages == [{"a": 1}, {"b": 2}]

    def test_many_messages_in_one_feed(self):
        framer = MessageFramer()
        chunk = b"".join(encode_message({"i": i}) for i in range(50))
        messages = framer.feed(chunk)
        assert messages == [{"i": i} for i in range(50)]

    def test_complete_plus_partial_in_one_feed(self):
        framer = MessageFramer()
        full = encode_message({"a": 1}) + encode_message({"b": 2})
        chunk = full[: -3]  # cut off the tail of the second message
        messages = framer.feed(chunk)
        assert messages == [{"a": 1}]
        assert framer.pending_bytes() > 0


class TestMessageFramerMalformedInput:
    def test_malformed_json_line_is_skipped_not_raised(self):
        framer = MessageFramer()
        chunk = b"not valid json\n" + encode_message({"ok": True})
        messages = framer.feed(chunk)
        assert messages == [{"ok": True}]  # the bad line is skipped, the good one after it still parses

    def test_empty_lines_are_skipped(self):
        framer = MessageFramer()
        chunk = b"\n\n" + encode_message({"a": 1}) + b"\n"
        messages = framer.feed(chunk)
        assert messages == [{"a": 1}]

    def test_invalid_utf8_does_not_raise(self):
        framer = MessageFramer()
        chunk = b"\xff\xfe\xfd\n" + encode_message({"ok": True})
        messages = framer.feed(chunk)
        assert messages == [{"ok": True}]


class TestMessageFramerReset:
    def test_reset_clears_pending_buffer(self):
        framer = MessageFramer()
        framer.feed(b'{"partial":')
        assert framer.pending_bytes() > 0
        framer.reset()
        assert framer.pending_bytes() == 0
