#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

// =========================
// FileReader & Evt3 Declarations
// =========================

namespace FileReader {

typedef struct {
  uint64_t timestamp;
  uint32_t x;
  uint32_t y;
  uint16_t pol;
} event_t;

typedef struct {
  uint32_t width;
  uint32_t height;
  uint64_t max_time;
  uint64_t min_time;
} metadata_t;

typedef struct {
  std::vector<event_t> events;
  metadata_t metadata;
} filedata_t;

filedata_t read_file(std::string filename);

std::vector<event_t> filter_event_time(std::vector<event_t>, uint64_t, uint64_t);

void print_events(std::vector<event_t>);

} // namespace FileReader

namespace Evt3 {

enum class EventTypes : uint8_t {
  EVT_ADDR_Y = 0x0,
  EVT_ADDR_X = 0x2,
  VECT_BASE_X = 0x3,
  VECT_12 = 0x4,
  VECT_8 = 0x5,
  EVT_TIME_LOW = 0x6,
  EVT_TIME_HIGH = 0x8,
  EXT_TRIGGER = 0xA
};

struct RawEvent {
  uint16_t pad : 12;  // Padding
  uint16_t type : 4;  // Event type
};

struct RawEventTime {
  uint16_t time : 12;
  uint16_t type : 4;  // Event type: EVT_TIME_LOW or EVT_TIME_HIGH
};

struct RawEventXAddr {
  uint16_t x : 11;   // Pixel X coordinate
  uint16_t pol : 1;  // Polarity: 0 for decrease, 1 for increase
  uint16_t type : 4; // Event type: EVT_ADDR_X
};

struct RawEventVect12 {
  uint16_t valid : 12; // Encodes validity of events for a vector of 12 pixels
  uint16_t type : 4;   // Event type: VECT_12
};

struct RawEventVect8 {
  uint16_t valid : 8;  // Encodes validity of events for a vector of 8 pixels
  uint16_t unused : 4;
  uint16_t type : 4;   // Event type: VECT_8
};

struct RawEventY {
  uint16_t y : 11;   // Pixel Y coordinate
  uint16_t orig : 1; // Camera identifier (e.g., master/slave)
  uint16_t type : 4; // Event type: EVT_ADDR_Y
};

struct RawEventXBase {
  uint16_t x : 11;   // Base pixel X coordinate
  uint16_t pol : 1;  // Polarity
  uint16_t type : 4; // Event type: VECT_BASE_X
};

struct RawEventExtTrigger {
  uint16_t value : 1; // Trigger value (edge polarity)
  uint16_t unused : 7;
  uint16_t id : 4;    // Trigger channel ID
  uint16_t type : 4;  // Event type: EXT_TRIGGER
};

using timestamp_t = uint64_t; // Timestamp in microseconds

} // namespace Evt3

// =========================
// FileReader Implementations
// =========================

namespace FileReader {

std::vector<event_t> filter_event_time(std::vector<event_t> events,
                                       uint64_t t_0, uint64_t t_end) {
  if (t_end < t_0) {
    throw std::invalid_argument("t_0 must be greater than 0 and less than t_end");
  }
  std::vector<event_t> filtered_events;
  std::copy_if(events.begin(), events.end(),
               std::back_inserter(filtered_events),
               [t_0, t_end](event_t event) {
                 return event.timestamp < t_end && event.timestamp > t_0;
               });
  return filtered_events;
}

filedata_t read_file(std::string filename) {
  filedata_t filedata;
  std::vector<event_t> events = {};
  metadata_t metadata;

  std::ifstream input_file(filename, std::ios::in | std::ios::binary);
  const auto tp_start = std::chrono::system_clock::now();

  if (!input_file.is_open()) {
    std::cerr << "Could not open file: " << filename << "\n";
    throw std::invalid_argument("Could not open input file");
  }

  // Read header lines starting with '%'
  while (input_file.peek() == '%') {
    std::string header_line;
    std::getline(input_file, header_line);
    if (header_line == "\% end") {
      break;
    } else if (header_line.substr(0, 9) == "\% format ") {
      std::istringstream sf(header_line.substr(9));
      std::string format_name;
      std::getline(sf, format_name, ';');
      if (format_name != "EVT3") {
        throw std::invalid_argument("Input file must be in the EVT3 format");
      }
    } else if (header_line.substr(0, 11) == "\% geometry ") {
      std::istringstream sg(header_line.substr(11));
      std::string sw, sh;
      std::getline(sg, sw, 'x');
      std::getline(sg, sh);
      metadata.height = std::stoi(sh);
      metadata.width = std::stoi(sw);
    }
  }

  if (metadata.width == 0 || metadata.height == 0) {
    metadata.width = 1280;
    metadata.height = 720;
  }

  metadata.min_time = UINT64_MAX;
  metadata.max_time = 0;

  // Define the number of words to read at a time.
  static constexpr uint32_t WORDS_TO_READ = 1000000;
  std::vector<Evt3::RawEvent> buffer_read(WORDS_TO_READ);

  // State variables for decoding.
  bool first_time_base_set = false;
  Evt3::timestamp_t current_time_base = 0;
  Evt3::timestamp_t current_time_low = 0;
  Evt3::timestamp_t current_time = 0;
  uint16_t current_ev_addr_y = 0;
  uint16_t current_base_x = 0;
  uint16_t current_polarity = 0;
  unsigned int n_time_high_loop = 0;

  while (input_file) {
    input_file.read(reinterpret_cast<char *>(buffer_read.data()),
                    WORDS_TO_READ * sizeof(Evt3::RawEvent));
    Evt3::RawEvent *current_word = buffer_read.data();
    Evt3::RawEvent *last_word = current_word + input_file.gcount() / sizeof(Evt3::RawEvent);

    // Set the time base.
    for (; !first_time_base_set && current_word != last_word; ++current_word) {
      Evt3::EventTypes type = static_cast<Evt3::EventTypes>(current_word->type);
      if (type == Evt3::EventTypes::EVT_TIME_HIGH) {
        Evt3::RawEventTime *ev_timehigh =
            reinterpret_cast<Evt3::RawEventTime *>(current_word);
        current_time_base = (Evt3::timestamp_t(ev_timehigh->time) << 12);
        first_time_base_set = true;
        break;
      }
    }

    // Decode the remaining words.
    for (; current_word != last_word; ++current_word) {
      Evt3::EventTypes type = static_cast<Evt3::EventTypes>(current_word->type);
      switch (type) {
      case Evt3::EventTypes::EVT_ADDR_X: {
        Evt3::RawEventXAddr *ev_addr_x =
            reinterpret_cast<Evt3::RawEventXAddr *>(current_word);
        event_t event;
        event.x = ev_addr_x->x;
        event.y = current_ev_addr_y;
        event.timestamp = current_time;
        event.pol = ev_addr_x->pol;
        events.push_back(event);
        if (event.timestamp < metadata.min_time) {
          metadata.min_time = current_time;
        }
        if (event.timestamp > metadata.max_time) {
          metadata.max_time = current_time;
        }
        break;
      }
      case Evt3::EventTypes::VECT_12: {
        uint16_t end = current_base_x + 12;
        Evt3::RawEventVect12 *ev_vec_12 =
            reinterpret_cast<Evt3::RawEventVect12 *>(current_word);
        uint32_t valid = ev_vec_12->valid;
        for (uint16_t i = current_base_x; i != end; ++i) {
          if (valid & 0x1) {
            event_t event;
            event.x = i;
            event.y = current_ev_addr_y;
            event.timestamp = current_time;
            event.pol = current_polarity;
            if (current_time < metadata.min_time) {
              metadata.min_time = current_time;
            }
            if (current_time > metadata.max_time) {
              metadata.max_time = current_time;
            }
            events.push_back(event);
          }
          valid >>= 1;
        }
        current_base_x = end;
        break;
      }
      case Evt3::EventTypes::VECT_8: {
        uint16_t end = current_base_x + 8;
        Evt3::RawEventVect8 *ev_vec_8 =
            reinterpret_cast<Evt3::RawEventVect8 *>(current_word);
        uint32_t valid = ev_vec_8->valid;
        for (uint16_t i = current_base_x; i != end; ++i) {
          if (valid & 0x1) {
            event_t event;
            event.x = i;
            event.y = current_ev_addr_y;
            event.timestamp = current_time;
            event.pol = current_polarity;
            if (current_time < metadata.min_time) {
              metadata.min_time = current_time;
            }
            if (current_time > metadata.max_time) {
              metadata.max_time = current_time;
            }
            events.push_back(event);
          }
          valid >>= 1;
        }
        current_base_x = end;
        break;
      }
      case Evt3::EventTypes::EVT_ADDR_Y: {
        Evt3::RawEventY *ev_addr_y =
            reinterpret_cast<Evt3::RawEventY *>(current_word);
        current_ev_addr_y = ev_addr_y->y;
        break;
      }
      case Evt3::EventTypes::VECT_BASE_X: {
        Evt3::RawEventXBase *ev_xbase =
            reinterpret_cast<Evt3::RawEventXBase *>(current_word);
        current_polarity = ev_xbase->pol;
        current_base_x = ev_xbase->x;
        break;
      }
      case Evt3::EventTypes::EVT_TIME_HIGH: {
        static constexpr Evt3::timestamp_t MaxTimestampBase =
            ((Evt3::timestamp_t(1) << 12) - 1) << 12;
        static constexpr Evt3::timestamp_t TimeLoop =
            MaxTimestampBase + (1 << 12);
        static constexpr Evt3::timestamp_t LoopThreshold = (10 << 12);
        Evt3::RawEventTime *ev_timehigh =
            reinterpret_cast<Evt3::RawEventTime *>(current_word);
        Evt3::timestamp_t new_time_base =
            (Evt3::timestamp_t(ev_timehigh->time) << 12);
        new_time_base += n_time_high_loop * TimeLoop;
        if ((current_time_base > new_time_base) &&
            (current_time_base - new_time_base >= MaxTimestampBase - LoopThreshold)) {
          new_time_base += TimeLoop;
          ++n_time_high_loop;
        }
        current_time_base = new_time_base;
        current_time = current_time_base;
        break;
      }
      case Evt3::EventTypes::EVT_TIME_LOW: {
        Evt3::RawEventTime *ev_timelow =
            reinterpret_cast<Evt3::RawEventTime *>(current_word);
        current_time_low = ev_timelow->time;
        current_time = current_time_base + current_time_low;
        break;
      }
      case Evt3::EventTypes::EXT_TRIGGER: {
        // Currently, EXT_TRIGGER events are not processed.
        break;
      }
      default:
        break;
      }
    }
  }
  const auto tp_end = std::chrono::system_clock::now();
  const double duration_s =
      std::chrono::duration_cast<std::chrono::microseconds>(tp_end - tp_start)
          .count() / 1e6;
  std::cout << "Decoded " << duration_s << " s" << std::endl;

  filedata.metadata = metadata;
  filedata.events = events;
  return filedata;
}

void print_events(std::vector<event_t> events) {
  // A simple implementation to print event details.
  for (const auto &event : events) {
    std::cout << "Timestamp: " << event.timestamp
              << ", x: " << event.x
              << ", y: " << event.y
              << ", polarity: " << event.pol << std::endl;
  }
}

} // namespace FileReader
