#include "contrastmax.hpp"
#include "filereader.hpp"
#include <Eigen/Dense>

#include <iostream>
#include <vector>

int main(int argc, char *argv[]) {
  FileReader::filedata_t fileData =
      FileReader::read_file("../data/recording2.raw");

  int width = fileData.metadata.width;
  int height = fileData.metadata.height;

  std::cout << fileData.events[0].timestamp << std::endl;

  fileData.events =
      FileReader::filter_event_time(fileData.events, 10000000, 10200000);

  ContrastMax::image_t prev_image =
      ContrastMax::create_image(fileData.events, width, height);

  ContrastMax::write_image(prev_image, "prev.pgm");

  auto start = std::chrono::high_resolution_clock::now();

  Eigen::Vector3d val;
  if (argc > 1 && std::string(argv[1]) == std::string("--blur")) {
    std::cout << "Running blur optimization" << std::endl;
    val = ContrastMax::maximize(fileData);
  } else {
    std::cout << "Running standard optimization" << std::endl;
    val = ContrastMax::maximize(fileData);
  }

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;

  std::vector<ContrastMax::event_t> warped_events =
      ContrastMax::warp_events(fileData.events, val);

  ContrastMax::image_t image =
      ContrastMax::create_image(warped_events, width, height);
  ContrastMax::write_image(image, "warped.pgm");

  std::cout << "Single pass: " << elapsed.count() << std::endl;
  std::cout << val << std::endl;
  std::cout << fileData.events.size() << std::endl;

  return 0;
}
