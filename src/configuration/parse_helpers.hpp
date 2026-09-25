#pragma once

// Internal parser helpers shared by every configuration entry point. They live
// in one definition so the runtime and application loaders cannot drift into
// two subtly different sets of semantics: the same exact-integer, finite-value,
// joint-name and joint-map checks reject the same input with the same
// field-path messages.

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

#include "irmv/configuration/config.hpp"
#include "irmv/protocol/types.hpp"

namespace irmv::configuration::detail {

using Json = nlohmann::json;

inline std::optional<std::uint64_t> ExactNonnegativeInteger(
    std::string_view token) {
  std::size_t position = 0;
  bool negative = false;
  if (position < token.size() &&
      (token[position] == '-' || token[position] == '+')) {
    negative = token[position] == '-';
    ++position;
  }

  const std::size_t exponent_position = token.find_first_of("eE", position);
  const std::size_t significand_end =
      exponent_position == std::string_view::npos ? token.size()
                                                  : exponent_position;
  const std::size_t decimal_position = token.find('.', position);
  const std::size_t fractional_digits =
      decimal_position == std::string_view::npos
          ? 0U
          : significand_end - decimal_position - 1U;

  std::string digits;
  digits.reserve(significand_end - position);
  for (std::size_t index = position; index < significand_end; ++index) {
    if (token[index] != '.') {
      digits.push_back(token[index]);
    }
  }
  const std::size_t first_nonzero = digits.find_first_not_of('0');
  if (first_nonzero == std::string::npos) {
    return 0U;
  }
  if (negative) {
    return std::nullopt;
  }
  digits.erase(0, first_nonzero);

  bool exponent_negative = false;
  std::size_t exponent_magnitude = 0;
  if (exponent_position != std::string_view::npos) {
    position = exponent_position + 1U;
    if (position < token.size() &&
        (token[position] == '-' || token[position] == '+')) {
      exponent_negative = token[position] == '-';
      ++position;
    }
    const std::size_t exponent_cap =
        token.size() > std::numeric_limits<std::size_t>::max() - 64U
            ? std::numeric_limits<std::size_t>::max()
            : token.size() + 64U;
    for (; position < token.size(); ++position) {
      const std::size_t digit = static_cast<std::size_t>(token[position] - '0');
      if (exponent_magnitude > (exponent_cap - digit) / 10U) {
        exponent_magnitude = exponent_cap;
        break;
      }
      exponent_magnitude = exponent_magnitude * 10U + digit;
    }
  }

  bool negative_shift = false;
  std::size_t shift = 0;
  if (exponent_negative) {
    negative_shift = true;
    shift = exponent_magnitude >
                    std::numeric_limits<std::size_t>::max() - fractional_digits
                ? std::numeric_limits<std::size_t>::max()
                : exponent_magnitude + fractional_digits;
  } else if (exponent_magnitude >= fractional_digits) {
    shift = exponent_magnitude - fractional_digits;
  } else {
    negative_shift = true;
    shift = fractional_digits - exponent_magnitude;
  }

  if (negative_shift) {
    if (shift > digits.size()) {
      return std::nullopt;
    }
    for (std::size_t index = digits.size() - shift; index < digits.size();
         ++index) {
      if (digits[index] != '0') {
        return std::nullopt;
      }
    }
    digits.resize(digits.size() - shift);
    if (digits.empty()) {
      return 0U;
    }
  } else {
    if (shift > 20U || digits.size() > 20U - shift) {
      return std::nullopt;
    }
    digits.append(shift, '0');
  }

  std::uint64_t result = 0;
  for (const char digit_character : digits) {
    const std::uint64_t digit =
        static_cast<std::uint64_t>(digit_character - '0');
    if (result > (std::numeric_limits<std::uint64_t>::max() - digit) / 10U) {
      return std::nullopt;
    }
    result = result * 10U + digit;
  }
  return result;
}

class ExactIntegerSax {
 public:
  using number_integer_t = Json::number_integer_t;
  using number_unsigned_t = Json::number_unsigned_t;
  using number_float_t = Json::number_float_t;
  using string_t = Json::string_t;
  using binary_t = Json::binary_t;

  explicit ExactIntegerSax(Json &result) : dom_(result) {}

  bool null() { return dom_.null(); }
  bool boolean(bool value) { return dom_.boolean(value); }
  bool number_integer(number_integer_t value) {
    return dom_.number_integer(value);
  }
  bool number_unsigned(number_unsigned_t value) {
    return dom_.number_unsigned(value);
  }
  bool number_float(number_float_t value, const string_t &token) {
    const auto integer = ExactNonnegativeInteger(token);
    return integer.has_value() ? dom_.number_unsigned(*integer)
                               : dom_.number_float(value, token);
  }
  bool string(string_t &value) { return dom_.string(value); }
  bool binary(binary_t &value) { return dom_.binary(value); }
  bool start_object(std::size_t elements) {
    return dom_.start_object(elements);
  }
  bool key(string_t &value) { return dom_.key(value); }
  bool end_object() { return dom_.end_object(); }
  bool start_array(std::size_t elements) { return dom_.start_array(elements); }
  bool end_array() { return dom_.end_array(); }
  bool parse_error(std::size_t position, const std::string &last_token,
                   const nlohmann::detail::exception &exception) {
    return dom_.parse_error(position, last_token, exception);
  }

 private:
  nlohmann::detail::json_sax_dom_parser<Json> dom_;
};

[[noreturn]] inline void Fail(std::string path, std::string message) {
  throw ConfigError(std::move(path), std::move(message));
}

inline const Json &Member(const Json &object, std::string_view key,
                          const std::string &path) {
  if (!object.is_object()) {
    Fail(path, "must be an object");
  }
  const auto member = object.find(key);
  if (member == object.end()) {
    Fail(path.empty() ? std::string(key) : path + "." + std::string(key),
         "is required");
  }
  return *member;
}

inline const Json &ObjectMember(const Json &object, std::string_view key,
                                const std::string &path) {
  const std::string member_path =
      path.empty() ? std::string(key) : path + "." + std::string(key);
  const Json &value = Member(object, key, path);
  if (!value.is_object()) {
    Fail(member_path, "must be an object");
  }
  return value;
}

inline std::string StringMember(const Json &object, std::string_view key,
                                const std::string &path) {
  const std::string member_path =
      path.empty() ? std::string(key) : path + "." + std::string(key);
  const Json &value = Member(object, key, path);
  if (!value.is_string()) {
    Fail(member_path, "must be a string");
  }
  return value.get<std::string>();
}

inline double FiniteNumber(const Json &value, const std::string &path) {
  if (!value.is_number()) {
    Fail(path, "must be a number");
  }
  const double result = value.get<double>();
  if (!std::isfinite(result)) {
    Fail(path, "must be finite");
  }
  return result == 0.0 ? 0.0 : result;
}

inline double FiniteMember(const Json &object, std::string_view key,
                           const std::string &path) {
  const std::string member_path =
      path.empty() ? std::string(key) : path + "." + std::string(key);
  return FiniteNumber(Member(object, key, path), member_path);
}

inline std::uint64_t IntegerInRange(const Json &value, const std::string &path,
                                    std::uint64_t minimum,
                                    std::uint64_t maximum) {
  std::uint64_t result = 0;
  if (value.is_number_unsigned()) {
    result = value.get<std::uint64_t>();
  } else if (value.is_number_integer()) {
    const auto signed_result = value.get<std::int64_t>();
    if (signed_result < 0) {
      Fail(path, "is outside the supported range");
    }
    result = static_cast<std::uint64_t>(signed_result);
  } else {
    Fail(path, "must be a mathematical integer");
  }
  if (result < minimum || result > maximum) {
    Fail(path, "is outside the supported range");
  }
  return result;
}

inline std::uint64_t PositiveInteger(
    const Json &value, const std::string &path,
    std::uint64_t maximum = std::numeric_limits<std::uint64_t>::max()) {
  return IntegerInRange(value, path, 1U, maximum);
}

inline std::uint64_t PositiveIntegerMember(
    const Json &object, std::string_view key, const std::string &path,
    std::uint64_t maximum = std::numeric_limits<std::uint64_t>::max()) {
  const std::string member_path =
      path.empty() ? std::string(key) : path + "." + std::string(key);
  return PositiveInteger(Member(object, key, path), member_path, maximum);
}

inline bool BoolMember(const Json &object, std::string_view key,
                       const std::string &path) {
  const std::string member_path =
      path.empty() ? std::string(key) : path + "." + std::string(key);
  const Json &value = Member(object, key, path);
  if (!value.is_boolean()) {
    Fail(member_path, "must be a boolean");
  }
  return value.get<bool>();
}

// The canonical joint-name check the runtime parser applies to robot.joints:
// nonempty and at most kMaxNameBytes UTF-8 bytes. It lives beside the other
// helpers so a second parser cannot drift from it.
inline void ValidateJointName(std::string_view name, const std::string &path) {
  if (name.empty() || name.size() > protocol::kMaxNameBytes) {
    Fail(path, "must contain 1 to 64 UTF-8 bytes");
  }
}

// Every key of a joint map must name a configured joint. The completeness
// checks (a configured joint with no entry) stay in the reader that knows which
// value rule applies; this one owns the "unknown joint" half for both readers.
inline void RejectUnknownJointKeys(
    const Json &mapping, const std::string &path,
    const std::vector<std::string> &joint_names) {
  std::unordered_set<std::string> expected(joint_names.begin(),
                                           joint_names.end());
  for (auto item = mapping.begin(); item != mapping.end(); ++item) {
    if (expected.count(item.key()) == 0U) {
      Fail(path + "." + item.key(), "does not name a configured joint");
    }
  }
}

// Which range a joint-map value must fall in. The runtime parser needs positive
// gains and non-negative continuity deltas; the application parser also reads
// signed target positions, so all three rules live here instead of a second
// completeness check elsewhere.
enum class JointValueRule {
  Finite,
  NonNegative,
  Positive,
};

inline void ReadJointMap(const Json &mapping, const std::string &path,
                         const std::vector<std::string> &joint_names,
                         JointValueRule rule, JointArray &destination) {
  if (!mapping.is_object()) {
    Fail(path, "must be an object keyed by joint name");
  }
  for (std::size_t index = 0; index < joint_names.size(); ++index) {
    const std::string &name = joint_names[index];
    const auto value = mapping.find(name);
    if (value == mapping.end()) {
      Fail(path + "." + name, "is required");
    }
    destination[index] = FiniteNumber(*value, path + "." + name);
    if (rule == JointValueRule::Positive && destination[index] <= 0.0) {
      Fail(path + "." + name, "must be positive");
    }
    if (rule == JointValueRule::NonNegative && destination[index] < 0.0) {
      Fail(path + "." + name, "must be non-negative");
    }
  }
  RejectUnknownJointKeys(mapping, path, joint_names);
}

inline Json LoadDocument(const std::filesystem::path &path) {
  std::ifstream input(path);
  if (!input) {
    Fail("$", "cannot open configuration file " + path.string());
  }

  Json document;
  try {
    ExactIntegerSax sax(document);
    if (!Json::sax_parse(input, &sax)) {
      Fail("$", "invalid JSON");
    }
  } catch (const Json::exception &error) {
    Fail("$", std::string("invalid JSON: ") + error.what());
  }
  if (!document.is_object()) {
    Fail("$", "must be an object");
  }
  return document;
}

}  // namespace irmv::configuration::detail
