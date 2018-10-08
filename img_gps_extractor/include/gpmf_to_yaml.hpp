/* 
 * GPMF to YAML Converter
 * 
 * Takes in mp4 video from gopro containing metadata and saves metadata to 
 * yaml file, for easy query from matlab, python or c++.
 *
 * May 2017 - Andres Milioto
 * 
 */

#ifndef _GPMF_TO_YAML_
#define _GPMF_TO_YAML_

// basic stuff
#include <string>
#include <stdint.h>
#include <map>
#include <vector>
#include "common.hpp"
#include "printer.hpp"

// includes for metadata parsing
#include "GPMF_parser.h"
#include "GPMF_mp4reader.h"
extern "C" void PrintGPMF(GPMF_stream *ms);

// opencv stuff to get images
#include "mp4_img_extractor.hpp"
namespace img_extr = mp4_img_extractor;

namespace gpmf_to_yaml
{
  
  typedef enum
  {
    CONV_OK = 0,
    CONV_ERROR,
    CONV_INIT_ERROR,
    CONV_INPUT_NON_EXISTENT,
    CONV_OUTPUT_NON_EXISTENT,
    CONV_NO_PAYLOAD,
    CONV_CANT_CREATE_OUTPUT,
    CONV_INVALID_STRUCT,
  }CONV_RET;

  typedef enum
  {
    TAG_GPS  = 0x1,
    TAG_GYRO = 0x2,
    TAG_ACCL = 0x4,
    TAG_ALL  = TAG_GPS | TAG_GYRO | TAG_ACCL
  }TAG;

  template<class T, class N>
  struct sensorframe_t
  {
    N ts; // timestamp in seconds (from video start)
    T gps[5]; // GPS data (lat deg, long deg, altitude m , 2D ground speed m/s, 3D speed m/s)
    //gpst // GPS time (UTC)
    //gpsf // GPS fix? 0-no lock. 2 or 3, 2D or 3D lock
    //gpsp // GPS precision: Under 300 is good (tipically around 5m to 10m)
    T gyro[3]; // IMU Gyroscope data rad/s
    T accl[3]; // IMU Accelerometer data m/s²
    //isog // ISO gain (dimensionless)
    //ss // shutter speed in seconds
  };


  template<class T, class N, template <class T, class N> class time_proxy, template <class N> class name_proxy>
  class converter
  {
    public:
      converter(bool verbose=false);
      converter(const std::string& in, const std::string& out_dir,bool verbose);
      ~converter();
      int32_t init(const T fr); //re-init parsing with same parameters
      int32_t init(const std::string& in, const std::string& out_dir,T fr,const uint32_t idx_offset=0); //init parsing changing parameters
      int32_t cleanup(); //cleanup and exit
      int32_t run(gpmf_io::printer* out, const uint16_t flags = TAG_ALL); //run conversion
      int32_t get_offset(); //offset for next run

    private:
      std::string _input;
      std::string _output_dir;  
      img_extr::img_extractor<T> _extractor;
      bool _verbose;
      uint32_t _idx_offset;
      
      // intermediate functions
      int32_t gpmf_to_maps(); // take in stream and build maps
      int32_t populate_images(gpmf_io::printer* out, const uint16_t flags = TAG_ALL); // get still images at desired framerate
      int32_t sensorframes_to_yaml(gpmf_io::printer*          out,
                                   const std::string&         name,
                                   const sensorframe_t<T, N>& sf,
                                   const bool                 first,
                                   const uint16_t             flags = TAG_ALL); // output desired yaml

      void process_tag (const uint32_t index, std::map<N,std::vector<T> >& out);
      void interpolate_data(N ts, std::map<N,std::vector<T> >& in, T* out);

      // maps for parsed values
      std::map<N,std::vector<T> > _gps;  //ts is key, gps data is value
      std::map<N,std::vector<T> > _accl; //ts is key, accl data is value
      std::map<N,std::vector<T> > _gyro; //ts is key, gyro data is value

      //map for interpolated values
      uint32_t _n_images; // final number of images in database

      // gpmf data
      GPMF_stream _metadata_stream, *_ms;
      size_t _mp4;
      double _metadatalength;
      uint32_t *_payload; //buffer to store GPMF samples from the MP4.

      time_proxy<T, N> _timeProxy;
      name_proxy<N>    _nameProxy;
  };

}

#include "gpmf_to_yaml_impl.hpp"

#endif // _GPMF_TO_YAML_
