#ifndef IMAGE_HPP
#define IMAGE_HPP

// opencv stuff to get images
#include "opencv2/opencv.hpp"
#include "common.hpp"
#include <ctime>

namespace mp4_img_extractor
{
  template<class N>
  class image
  {
  public:

    image(const bool verbose)
      : _verbose(verbose)
    {
    }

    void save (const std::string& name)
    {
      DEBUG("Saving image in %s\n", name.c_str());
      std::clock_t begin_time = std::clock();
      cv::imwrite(name,_frame);
      DEBUG("Time cv::imwrite: %f\n",double(clock()-begin_time)/CLOCKS_PER_SEC);
    }

    inline N& timestamp()
    {
      return _timestamp;
    }

    inline uint32_t& index()
    {
      return _idx;
    }

    inline cv::Mat& get()
    {
      return _frame;
    }

  private:
    bool     _verbose;
    cv::Mat  _frame;
    N        _timestamp;
    uint32_t _idx;
  };
}

#endif // IMAGE_HPP
