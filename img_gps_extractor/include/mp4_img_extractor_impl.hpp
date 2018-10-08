/* 
 * MP4 to frame extractor
 * 
 * Takes in mp4 video from gopro containing metadata processes frame
 * by frame operations, like getting a frame given the timestamp.
 *
 * May 2017 - Andres Milioto
 * 
 */

#include <ctime>

namespace mp4_img_extractor
{
  template<class T>
  img_extractor<T>::img_extractor(bool verbose):_verbose(verbose)
  {
  }

  template<class T>
  img_extractor<T>::img_extractor(const std::string& in,
                               const std::string& out_dir,
                               bool verbose):
                               _input(in),_output_dir(out_dir),
                               _verbose(verbose),_cap(_input),
                               _fr(1)
  {
  }

  template<class T>
  img_extractor<T>::~img_extractor()
  {
  }
  
  template<class T>
  int32_t img_extractor<T>::init(const T fr)
  {
    if(!_cap.isOpened())
    {
      // error!
      DEBUG("Can not load video. Exiting...\n");
      return EXTR_CANT_LOAD_VIDEO;
    }
    else
    {
      // Success!
      DEBUG("Opened cv capture successfully...\n");
    }

    T frames = _cap.get(CV_CAP_PROP_FRAME_COUNT);
    DEBUG("There are %f frames in the video.\n",frames);
    T fps = _cap.get(CV_CAP_PROP_FPS);
    DEBUG("Fps is %f for video.\n",fps);
    _duration = frames / fps;
    DEBUG("Duration of video is %f.\n",_duration);

    _fr = fr;
    if (!(_fr > 0.))
    {
      _fr = fps;
      DEBUG("Frame rate is negative, using fps instead %f.\n",_fr);
    }

    return EXTR_OK;
  }

  template<class T>
  int32_t img_extractor<T>::init(const std::string& in, const std::string& out_dir, const T fr)
  {
    _input = in;
    _output_dir = out_dir;
    _cap.release();
    _cap = cv::VideoCapture(_input);

    // init
    int ret = init(fr);

    return ret;
  }
  

  // gets the frame closest to ts and returns the real ts extracted and 
  // the filename
  template<class T>
  template<class N>
  int32_t img_extractor<T>::get_frame(T ts, T & real_ts, image<N>& frame)
  {
    int ret = EXTR_OK;

    // timestamp comes in seconds, and opencv works in ms, so convert:
    real_ts = ts * 1000;
    const T fps  = _cap.get(CV_CAP_PROP_FPS);
    const T diff = 1. / fps;

    if (_fr < fps)
    {
      // set the timestamp in video to obtain frame
      _cap.set(CV_CAP_PROP_POS_MSEC,real_ts);
      DEBUG("Setting timestamp to %.5f\n",real_ts);
    }

    // if frame was not captured, try again n times
    int tries=1;
    std::clock_t begin_time = std::clock();

    // get frame
    cv::Mat& cvFrame = frame.get();
    while(cvFrame.empty() && tries <= 50)
    {
      // Check if timestamp is within boundaries
      if(std::abs(real_ts - _duration * 1000) < diff)
      {
        DEBUG("Frame %f requested is out of bounds. Exiting\n",real_ts);
        return EXTR_CANT_FRAME_OUT_OF_BOUNDS;
      }

      _cap >> cvFrame;

      // get real timestamp to return
      real_ts = _cap.get(CV_CAP_PROP_POS_MSEC);
      if(cvFrame.empty() && tries > 1)
      {
        DEBUG("Frame skipped trying again for %d time: Real timestamp set to %.5f\n",tries, real_ts);
      }
      else
      {
        DEBUG("Real timestamp set to %.5f\n",real_ts);
      }

      tries++;
    }

    // 50 times is an exaggeration, if it still didn't work, something is wrong,
    // report back to user.
    if(cvFrame.empty())
    {
      std::cerr << "Skipping frame at " << real_ts << "ms. Exiting" <<std::endl;
      return EXTR_SKIPPING_FRAME;
    }

    DEBUG("Time cv cap>>frame: %f\n",T(clock()-begin_time)/CLOCKS_PER_SEC);

    // return real_ts in seconds again
    real_ts /= 1000;

    return ret;

  }

}
