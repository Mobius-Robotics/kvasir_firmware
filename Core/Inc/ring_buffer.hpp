#pragma once

#include <algorithm>

class RingBuffer {
public:
	static constexpr size_t SIZE = 256;
	static constexpr size_t MASK = SIZE - 1;  // For fast mod operations.

	RingBuffer() :
			head_(0), tail_(0) {
	}

	// Push is called from the ISR.
	void push(uint8_t byte) {
		buffer_[head_] = byte;
		head_ = (head_ + 1) & MASK;
		// Handle overflow: here we overwrite the oldest data.
		if (head_ == tail_) {
			tail_ = (tail_ + 1) & MASK;
		}
	}

	// Pop removes the next byte and returns it.
	// Returns true if a byte was available.
	bool pop(uint8_t &byte) {
		if (empty())
			return false;
		byte = buffer_[tail_];
		tail_ = (tail_ + 1) & MASK;
		return true;
	}

	// Returns the number of bytes currently stored.
	size_t available() const {
		if (head_ >= tail_)
			return head_ - tail_;
		else
			return SIZE - tail_ + head_;
	}

	bool empty() const {
		return head_ == tail_;
	}

	// Peek at both the next byte and the second-next byte
	bool peek_two(uint8_t &firstByte, uint8_t &secondByte) const {
		if (available() < 2)  // Ensure at least two bytes are available
			return false;
		firstByte = buffer_[tail_];
		secondByte = buffer_[(tail_ + 1) & MASK];
		return true;
	}

	void discard(const size_t n = 1) {
		tail_ = (tail_ + std::min(available(), n)) & MASK;
	}

private:
	volatile uint8_t buffer_[SIZE];
	volatile size_t head_;
	volatile size_t tail_;
};
