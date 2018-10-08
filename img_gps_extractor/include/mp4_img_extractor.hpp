/* 
 * MP4 to frame extractor
 * 
 * Takes in mp4 video from gopro containing metadata processes frame
 * by frame operations, like getting a frame given the timestamp.
 *
 * May 2017 - Andres Milioto
 * 
 */

#ifndef _MP4_IMG_EXTRACTOR_H_
#define _MP4_IMG_EXTRACTOR_H_

// opencv stuff to get images
#include "opencv2/opencv.hpp"

// basic stuff
#include <string>
#include <iostream>
#include "common.hpp"
#include "image.hpp"

namespace mp4_img_extractor
{

  typedef enum
  {
    EXTR_OK=0,
    EXTR_SKIPPING_FRAME,
    EXTR_ERROR,
    EXTR_CANT_LOAD_VIDEO,
    EXTR_CANT_FRAME_OUT_OF_BOUNDS,
  }EXTR_RET;

  template<class T>
  class img_extractor
  {
    public:
      img_extractor(bool verbose=false);
      img_extractor(const std::string& in, const std::string& out_dir,bool verbose);
      ~img_extractor();
      int32_t init(const T fr);
      int32_t init(const std::string& in, const std::string& out_dir, const T fr);

      template<class N>
      int32_t get_frame(T ts, T & real_ts, image<N>& frame);

      inline const T& fr() const
      {
        return _fr;
      }

    private:
      std::string _input;
      std::string _output_dir;
      cv::VideoCapture _cap;
      T _fr;
      T _duration;
      bool _verbose;
  };

}

#include "mp4_img_extractor_impl.hpp"

#endif // _MP4_IMG_EXTRACTOR_H_
