// CircularBuffer.hpp
#ifndef CIRCULAR_BUFFER_HPP
#define CIRCULAR_BUFFER_HPP

#include <cstddef> // for size_t
#include <algorithm> // for std::fill

template<typename T, size_t Size>
class CircularBuffer {
private:
    T buffer[Size];
    size_t index = 0;

public:
    CircularBuffer() {
        // Initialize buffer to zero
        std::fill(std::begin(buffer), std::end(buffer), T{});
    }

    void push(T value) {
        buffer[index] = value;
        index = (index + 1) % Size;
    }

    T& operator[](size_t i) {
        return buffer[(index + i) % Size];
    }

    const T& operator[](size_t i) const {
        return buffer[(index + i) % Size];
    }

    static constexpr size_t size() {
        return Size;
    }
};

#endif // CIRCULAR_BUFFER_HPP
