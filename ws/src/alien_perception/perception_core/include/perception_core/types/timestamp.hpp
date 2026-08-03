#ifndef PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_TYPES_TIMESTAMP_HPP
#define PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_TYPES_TIMESTAMP_HPP

#include <cstdint>

namespace Perception {

    /// Timestamp representation (nanoseconds since epoch)
    struct Timestamp {
        int64_t nanoseconds;

        static Timestamp from_seconds(double seconds)
        {
            return Timestamp {static_cast<int64_t>(seconds * 1e9)};
        }

        static Timestamp from_nanoseconds(int64_t ns)
        {
            return Timestamp {ns};
        }

        double to_seconds() const
        {
            return static_cast<double>(nanoseconds) / 1e9;
        }

        bool operator==(const Timestamp & other) const { return nanoseconds == other.nanoseconds; }
        bool operator!=(const Timestamp & other) const { return nanoseconds != other.nanoseconds; }
        bool operator<(const Timestamp & other) const { return nanoseconds < other.nanoseconds; }
        bool operator<=(const Timestamp & other) const { return nanoseconds <= other.nanoseconds; }
        bool operator>(const Timestamp & other) const { return nanoseconds > other.nanoseconds; }
        bool operator>=(const Timestamp & other) const { return nanoseconds >= other.nanoseconds; }

        Timestamp operator-(const Timestamp & other) const
        {
            return Timestamp {nanoseconds - other.nanoseconds};
        }

        Timestamp operator+(const Timestamp & other) const
        {
            return Timestamp {nanoseconds + other.nanoseconds};
        }
    };

    /// Duration representation
    struct Duration {
        int64_t nanoseconds;

        static Duration from_seconds(double seconds)
        {
            return Duration {static_cast<int64_t>(seconds * 1e9)};
        }

        static Duration from_nanoseconds(int64_t ns)
        {
            return Duration {ns};
        }

        double to_seconds() const
        {
            return static_cast<double>(nanoseconds) / 1e9;
        }

        bool operator==(const Duration & other) const { return nanoseconds == other.nanoseconds; }
        bool operator!=(const Duration & other) const { return nanoseconds != other.nanoseconds; }
        bool operator<(const Duration & other) const { return nanoseconds < other.nanoseconds; }
        bool operator<=(const Duration & other) const { return nanoseconds <= other.nanoseconds; }
        bool operator>(const Duration & other) const { return nanoseconds > other.nanoseconds; }
        bool operator>=(const Duration & other) const { return nanoseconds >= other.nanoseconds; }

        Duration operator+(const Duration & other) const
        {
            return Duration {nanoseconds + other.nanoseconds};
        }

        Duration operator-(const Duration & other) const
        {
            return Duration {nanoseconds - other.nanoseconds};
        }
    };

}// namespace Perception

#endif// PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_TYPES_TIMESTAMP_HPP
