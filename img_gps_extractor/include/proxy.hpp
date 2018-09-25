#ifndef PROXY_HPP
#define PROXY_HPP

#include <image.hpp>
#include <chrono>

namespace gpmf_proxy
{
  template<class T, class N>
  struct time_proxy
  {
    N get(const T& timestamp)
    {
      return timestamp;
    }
  };

  template<class T>
  struct time_proxy<T, uint64_t>
  {
    time_proxy()
      : _offset(static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::system_clock::now().time_since_epoch()).count()))
    {
    }

    uint64_t get(const T& timestamp)
    {
      return _offset + std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<T>(timestamp)).count();
    }

    uint64_t _offset;
  };

  template<class N>
  struct name_proxy
  {
    std::string get(const mp4_img_extractor::image<N>& image)
    {
      // create filename as real ts with 6 digits(assume less than 1 million imgs)
      // pad with 0's
      std::string name = std::to_string(const_cast<mp4_img_extractor::image<N>&>(image).index());
      size_t zero_pad = 6 - name.length();
      size_t i = 0;
      while(i < zero_pad)
      {
        name = '0' + name;
        i++;
      }

      return name;
    }
  };

  template<>
  struct name_proxy<uint64_t>
  {
    std::string get(const mp4_img_extractor::image<uint64_t>& image)
    {
      return std::to_string(const_cast<mp4_img_extractor::image<uint64_t>&>(image).timestamp());
    }
  };
}

#endif // PROXY_HPP
