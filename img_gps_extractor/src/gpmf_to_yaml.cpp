/* 
 * GPMF to YAML Converter
 * 
 * Takes in mp4 video from gopro containing metadata and saves metadata to 
 * yaml file, for easy query from matlab, python or c++.
 *
 * May 2017 - Andres Milioto
 * 
 */

// class definitions
#include "gpmf_to_yaml.hpp"

// basic stuff
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <iostream> 
#include <string> 
#include <fstream>

namespace gpmf_to_yaml
{

  converter::converter(bool verbose):_extractor(verbose)
  {
    // init some members
    _ms = &_metadata_stream;
    _payload = NULL;
    _verbose = verbose; //verbose is false by default
  }

  converter::converter(const std::string& in,
                       const std::string& out_dir, 
                       const float fr,
                       bool verbose):_input(in),_output_dir(out_dir),
                                     _fr(fr),_verbose(verbose),
                                     _extractor(_input,_output_dir,verbose)

  {
    // init some members
    _ms = &_metadata_stream;
    _payload = NULL;
  }

  int32_t converter::init()
  {
    // init some members
    _ms = &_metadata_stream;
    _payload = NULL;
    _mp4 = OpenMP4Source(const_cast<char*>(_input.c_str()), MOV_GPMF_TRAK_TYPE, MOV_GPMF_TRAK_SUBTYPE);
    _metadatalength = GetDuration (_mp4);
    if(_metadatalength > 0.0)
    {
      uint32_t index, payloads = GetNumberPayloads(_mp4);
      std::cout << "Found " << _metadatalength << "s of metadata, from " 
                << payloads << " payloads, within " << _input << std::endl;
    }
    else
    {
      std::cerr << "Found no payload. Exiting..." << std::endl;
      return CONV_NO_PAYLOAD;
    }

    // init the frame extractor
    int ret = _extractor.init();
    if(ret)
    {
      std::cerr << "Couldn't open mp4 video. Exiting..." << std::endl;
      return CONV_ERROR;
    }
    else
    {
      std::cout << "Done opening video with opencv! Ready to extract frames" << std::endl;
    }

    return CONV_OK;
  }

  int32_t converter::init(const std::string& in,
                          const std::string& out_dir, 
                          const float fr,
                          const uint32_t idx_offset)
  {
    // reload args into members
    _input = in;
    _output_dir = out_dir;
    _fr = fr;
    _extractor.init(_input,_output_dir);
    _idx_offset = idx_offset;
    
    // init
    int ret = init();

    return ret;
  }

  int32_t converter::run(YAML::Emitter & out)
  {
    int32_t ret = CONV_OK;
    
    /*
     * Our intention is to sub-sample at a specific frame rate, so 
     * we will:
     *  - Loop our log and extract all meta-data (different frequencies).
     *  - Loop with our frame-rate extract still images. This involves creating
     *    a folder and putting every image there with a certain name.
     *  - Interpolate sensor info for each still image time-stamp
     *  - Save the interpolated data with the image to a structure
     *  - Save every structure in the yaml file to create database. It should
     *    also contain the original file name and the frame rate at the
     *    beginning of the yaml file.
     */

    // extract all metadata to maps
    std::cout << "Parsing GPMF data..." << std::endl;
    ret = gpmf_to_maps();
    if(ret)
    {
      std::cout << "Error parsing GPMF data. Exiting..." << std::endl;
      return ret;
    }
    else
    {
      std::cout << "Done parsing GPMF data." << std::endl << std::endl ;
    }

    // sample images at desired framerate
    std::cout << "Sampling images at desired framerate..." << std::endl;
    ret = populate_images();
    if(ret)
    {
      std::cout << "Error populating images. Exiting..." << std::endl;
      return ret;
    }
    else
    {
      std::cout << "Done sampling images at desired framerate." << std::endl << std::endl ;
    }

    // interpolate sensor metadata to adjust to image sampling rate
    std::cout << "Interpolating sensors and populating sensor frames..." << std::endl;
    ret = sensors_to_sensorframes();
    if(ret)
    {
      std::cout << "Error interpolating sensors. Exiting..." << std::endl;
      return ret;
    }
    else
    {
      std::cout << "Done interpolating sensors." << std::endl << std::endl ;
    }
    
    // create yaml database in the output folder with the metadata for each img
    std::cout << "Creating metadata yaml dict..." << std::endl;
    ret = sensorframes_to_yaml(out);
    if(ret)
    {
      std::cout << "Error creating metadata yaml dict. Exiting..." << std::endl;
      return ret;
    }
    else
    {
      std::cout << "Done creating metadata yaml dict." << std::endl << std::endl;
    }


    return ret;
  }

  int32_t converter::get_offset()
  {
    return _idx_offset;
  }

  int32_t converter::cleanup()
  {
    if (_payload) FreePayload(_payload);
    _payload = NULL;
    CloseSource(_mp4);

    // empty the maps
    _gps.clear();
    _accel.clear ();
    _sensor_frames.clear();

    return CONV_OK;
  }

  // intermediate functions
  int32_t converter::gpmf_to_maps()
  {
    int32_t ret = CONV_OK;

    if (_metadatalength > 0.0)
    {
      uint32_t index, payloads = GetNumberPayloads(_mp4);
      for (index = 0; index < payloads; index++)
      {
        uint32_t payloadsize = GetPayloadSize(_mp4, index);
        float inPl = 0.0, outPl = 0.0; //times
        _payload = GetPayload(_mp4, _payload, index);
        if (_payload == NULL)
        {
          cleanup();
          return CONV_NO_PAYLOAD;
        }

        ret = GetPayloadTime(_mp4, index, &inPl, &outPl);
        if (ret != GPMF_OK)
        {
          cleanup();
          return CONV_NO_PAYLOAD;
        }
        DEBUG("MP4 Payload time %.3f to %.3f seconds\n", inPl, outPl);

        ret = GPMF_Init(_ms, _payload, payloadsize);
        if (ret != GPMF_OK)
        {
          cleanup();
          return CONV_INIT_ERROR;
        }
        // Find all the available Streams and the data carrying FourCC
        while (GPMF_OK == GPMF_FindNext(_ms, GPMF_KEY_STREAM, GPMF_RECURSE_LEVELS))
        {
          if (GPMF_OK == GPMF_SeekToSamples(_ms)) //find the last FOURCC within the stream
          {
            uint32_t key = GPMF_Key(_ms);
            GPMF_SampleType type = static_cast<GPMF_SampleType>(GPMF_Type(_ms));
            uint32_t elements = GPMF_ElementsInStruct(_ms);
            uint32_t samples = GPMF_Repeat(_ms);

            if (samples)
            {
              double in = 0.0, out = 0.0;
              double rate = GetGPMFSampleRateAndTimes(_mp4, _ms, 0.0, index, &in, &out);

              DEBUG("  STRM of %c%c%c%c %.3f-%.3fs %.3fHz ", PRINTF_4CC(key), in, out, rate);

              if (type == GPMF_TYPE_COMPLEX)
              {
                GPMF_stream find_stream;
                GPMF_CopyState(_ms, &find_stream);

                if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_TYPE, GPMF_CURRENT_LEVEL))
                {
                  char tmp[64];
                  char *data = (char *)GPMF_RawData(&find_stream);
                  int size = GPMF_RawDataSize(&find_stream);

                  if (size < sizeof(tmp))
                  {
                    memcpy(tmp, data, size);
                    tmp[size] = 0;
                    DEBUG("of type %s ", tmp);
                  }
                }

              }
              else
              {
                DEBUG("of type %c ", type);
              }

              DEBUG("with %d sample%s ", samples, samples > 1 ? "s" : "");

              if (elements > 1)
                DEBUG("-- %d elements per sample", elements);

              DEBUG("\n");
            }
          }
        }
        GPMF_ResetState(_ms);
        DEBUG("\n"); 

        do
        {
          switch (GPMF_Key (_ms))
          {
            case STR2FOURCC ("ACCL"):
              process_accel (index);
              break;

            case STR2FOURCC ("GPS5"):
            case STR2FOURCC ("GPRI"):
              process_gps (index);
              break;
          }
        }
        while (GPMF_OK == GPMF_Next (_ms, GPMF_RECURSE_LEVELS));

        GPMF_ResetState(_ms);
        DEBUG("\n");
      }
    }

    return ret;
  }

  void converter::process_accel(const uint32_t index)
  {
    const char     aType         = GPMF_Type            (_ms);
    const uint32_t aComponentsNb = GPMF_ElementsInStruct(_ms);
    const uint32_t aSamplesNb    = GPMF_Repeat          (_ms);
    const uint32_t aBufferSize   = aComponentsNb * aSamplesNb * sizeof(double);

    double *aBuffer = static_cast<double*>(malloc (aBufferSize));
    if (aBuffer && aSamplesNb)
    {
      char*    aUnitsBuffer   = NULL;
      uint32_t aUnitSamplesNb = 1;

      //Search for any units to display
      GPMF_stream find_stream;
      GPMF_CopyState(_ms, &find_stream);
      if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_SI_UNITS, GPMF_CURRENT_LEVEL) ||
          GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_UNITS,    GPMF_CURRENT_LEVEL))
      {
        char*  aUnitsData = static_cast<char*>(GPMF_RawData (&find_stream));
        const int aLength = GPMF_StructSize     (&find_stream);
        aUnitSamplesNb    = GPMF_Repeat         (&find_stream);

        aUnitsBuffer = static_cast<char*>(malloc ((aLength + 1) * aUnitSamplesNb));
        for (uint32_t i = 0; i < aUnitSamplesNb; i++)
        {
          memcpy(&aUnitsBuffer[i * aLength], aUnitsData, aLength);
          aUnitsBuffer[i * (aLength + 1) + aLength] = 0;
          aUnitsData += aLength;
        }
      }

      GPMF_ScaledData(_ms, aBuffer, aBufferSize, 0, aSamplesNb, GPMF_TYPE_DOUBLE);  //Output scaled data as floats

      for (uint32_t i = 0; i < aSamplesNb; i++)
      {
        DEBUG("%c%c%c%c ", PRINTF_4CC(GPMF_Key (_ms)));

        std::vector<float> accel_sample;

        //get timestamp for that sample
        double accel_rate,accel_start,accel_end;
        accel_rate = 1/GetGPMFSampleRateAndTimes(_mp4, _ms, 0.0, index, &accel_start, &accel_end);
        _accel[accel_start+accel_rate*i]=accel_sample;

        //get all value for that sample (lat, long, etc)
        DEBUG("TIME: %.3fs - ",accel_start+accel_rate*i);
        for (uint32_t j = 0; j < aComponentsNb; j++)
        {
          _accel[accel_start+accel_rate*i].push_back(aBuffer[i * aComponentsNb + j]);
          DEBUG("%.6f%s, " ,_accel[accel_start+accel_rate*i][j], &aUnitsBuffer[j % aUnitSamplesNb]);
        }
        DEBUG("\n");
      }

      free (aUnitsBuffer);
      free (aBuffer);
    }
  }

  void converter::process_gps(const uint32_t index)
  {
    uint32_t samples = GPMF_Repeat(_ms);
    uint32_t elements = GPMF_ElementsInStruct(_ms);
    uint32_t buffersize = samples * elements * sizeof(float);
    GPMF_stream find_stream;
    float *ptr, *tmpbuffer = static_cast<float*>(malloc(buffersize));
    char units[10][6] = { "" };
    uint32_t unit_samples = 1;

    if (tmpbuffer && samples)
    {
      uint32_t i, j;

      //Search for any units to display
      GPMF_CopyState(_ms, &find_stream);
      if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_SI_UNITS, GPMF_CURRENT_LEVEL) ||
        GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_UNITS, GPMF_CURRENT_LEVEL))
      {
        char *data = (char *)GPMF_RawData(&find_stream);
        int ssize = GPMF_StructSize(&find_stream);
        unit_samples = GPMF_Repeat(&find_stream);

        for (i = 0; i < unit_samples; i++)
        {
          memcpy(units[i], data, ssize);
          units[i][ssize] = 0;
          data += ssize;
        }
      }

      //GPMF_FormattedData(_ms, tmpbuffer, buffersize, 0, samples); // Output data in LittleEnd, but no scale
      GPMF_ScaledData(_ms, tmpbuffer, buffersize, 0, samples, GPMF_TYPE_FLOAT);  //Output scaled data as floats

      ptr = tmpbuffer;
      for (i = 0; i < samples; i++)
      {
        // create gps_sample to store in map
        std::vector<float> gps_sample;

        //get timestamp for that sample
        double gps_rate,gps_start,gps_end;
        gps_rate = 1/GetGPMFSampleRateAndTimes(_mp4, _ms, 0.0, index, &gps_start, &gps_end);
        _gps[gps_start+gps_rate*i]=gps_sample;

        //get all value for that sample (lat, long, etc)
        DEBUG("TIME: %.3fs - ",gps_start+gps_rate*i);
        for (j = 0; j < elements; j++)
        {
          _gps[gps_start+gps_rate*i].push_back(*ptr++);
          DEBUG("%.6f%s, " ,_gps[gps_start+gps_rate*i][j], units[j%unit_samples]);
        }
        DEBUG("\n");
      }
      free(tmpbuffer);
    }
  }
  
  int32_t converter::populate_images()
  {
    int32_t ret = CONV_OK;

    float ts = 0.0;
    float step = 1.0 / _fr; // timestep
    float real_ts; // real timestamp from image capture
    std::string name;  // name of exported image
    sensorframe_t sf; // sensor frame for each image

    uint32_t idx = 0;
    uint32_t skipped = 0;
    while(true)
    {
      float timestep = ts+idx*step;
      ret = _extractor.get_frame(timestep,real_ts,_idx_offset+idx,name);
      if(ret == img_extr::EXTR_CANT_FRAME_OUT_OF_BOUNDS)
      {
        DEBUG("Done populating, we are off bounds.\n");
        _n_images = idx - skipped;
        DEBUG("Number of images extracted for database is %u.\n",_n_images);
        //final offset for next batch of files
        _idx_offset+=idx;
        return CONV_OK;
      }
      else if(ret == img_extr::EXTR_SKIPPING_FRAME)
      {
        DEBUG("Skipping frame. Don't save to list\n");
        idx++;
        skipped++;
        continue;
      }

      else if(ret)
      {
        DEBUG("ERROR GETTING FRAME\n");
        return CONV_ERROR;
      }
      // populate a sensor frame for each image and put only timestamp for now
      // (interpolated gps, and other sensors will be populated later)
      sf.ts = real_ts+_idx_offset*step;
      _sensor_frames[name] = sf;
      DEBUG("ts: %.5f, real ts: %.5f, name: %s\n\n",timestep+_idx_offset*step,_sensor_frames[name].ts,name.c_str());
      idx++;
    }
    //final offset for next batch of files
    _idx_offset+=idx;
    return ret;
  }
  
  int32_t converter::sensors_to_sensorframes()
  {
    int32_t ret = CONV_OK;

    // for each image and sensor frame in the map, get the closest 2 timestamps
    // for each sensor map and interpolate its info.
    for (auto& sf:_sensor_frames)
    { 
      DEBUG("\n\n");
      DEBUG("Image sample name: %s.\n",sf.first.c_str());

      // get the timestamp for image
      float ts = sf.second.ts;

      interpolate_data (ts, _gps,   sf.second.gps);
      interpolate_data (ts, _accel, sf.second.accel);
    }


    return ret;
  }

  void converter::interpolate_data(float ts, std::map<float,std::vector<float> >& in, float* out)
  {
    /*
      Get gps info right before and right after that timestamp.
      Special cases are first image and last image, that may not
      have 2 data points, so we get the closest one. First case never
      happens to us because first frame of opencv video is always 0.0 and
      same for gopro sensor data. Also the astronomical chance that a sensor
      ts coincides with sample time of image.
    */
    float prev_ts=in.begin()->first, next_ts=in.rbegin()->first, delta_ts;

    // this covers the general case (middle of file) and the coincidence
    for(auto const& sample: in)
    {
      if(sample.first < ts)
      {
        prev_ts = sample.first;
      }
      else if(sample.first > ts)
      {
        next_ts = sample.first;
        break;
      }
      else //same
      {
        prev_ts = sample.first;
        next_ts = sample.first;
        break;
      }
    }

    DEBUG("      prev data ts: %.10f.\n",prev_ts);
    DEBUG("            img ts: %.10f.\n",ts);
    DEBUG("      next data ts: %.10f.\n",next_ts);

    // interpolate for each one, taking into consideration the 0 delta case,
    // which happens when gps frame coincides with image sample, and in the
    // last image, which may not have gps data after it.
    size_t dimNb = in.begin ()->second.size ();
    for(int i=0; i<dimNb; i++)
    {
      if(next_ts>prev_ts)
      {
        delta_ts = next_ts - prev_ts; // delta ts
        float m = (in[next_ts][i] - in[prev_ts][i]) / delta_ts;
        out[i] = in[prev_ts][i] + m * (ts - prev_ts);
      }
      else
      {
        // DEBUG("-------------------Same ts!\n");
        out[i] = in[prev_ts][i];
      }
      DEBUG("      prev data[%d]: %.10f.\n",i,in[prev_ts][i]);
      DEBUG("   Interpolated[%d]: %.10f.\n",i,out[i]);
      DEBUG("      next data[%d]: %.10f.\n",i,in[next_ts][i]);
    }
  }
  
  int32_t converter::sensorframes_to_yaml(YAML::Emitter & out)
  {

    int32_t ret = CONV_OK;

    //comments with some info about the program run
    //put every sensor frame in yaml file
    bool first = true;
    for (auto& sf:_sensor_frames)
    {
      // create entry for the file name
      out << YAML::Key << sf.first;
      out << YAML::Value;

      // if it is the first video of this file, comment where it was taken
      if(first)
      {
        out << YAML::Comment("Original File: "+_input+", Frame rate: "+std::to_string(_fr));
        first=false;
      }

      // create timestamp
      out << YAML::BeginMap;
      out << YAML::Key << "ts";
      out << YAML::Value << sf.second.ts;
      
      // output gps data
      out << YAML::Key << "gps";
      out << YAML::Value;

      out << YAML::BeginMap;
      out << YAML::Key << "lat" << YAML::Value << sf.second.gps[0];
      out << YAML::Key << "long" << YAML::Value<< sf.second.gps[1];
      out << YAML::Key << "alt" << YAML::Value << sf.second.gps[2];
      out << YAML::Key << "2dv" << YAML::Value << sf.second.gps[3];
      out << YAML::Key << "3dv" << YAML::Value << sf.second.gps[4]; 
      out << YAML::EndMap;

      out << YAML::Key << "accel";
      out << YAML::Value;

      out << YAML::BeginMap;
      out << YAML::Key << "2dX" << YAML::Value << sf.second.accel[0];
      out << YAML::Key << "2dY" << YAML::Value << sf.second.accel[1];
      out << YAML::Key << "2dZ" << YAML::Value << sf.second.accel[2];
      out << YAML::EndMap;
      
      out << YAML::EndMap;
    }

    // debug() impl makes no sense here because there is already a << 
    // impl for the whole file
    if(_verbose)
    {
      std::cout << out.c_str() << std::endl << std::endl;
    }

    return ret;

  }

  // destructor
  converter::~converter()
  {
  }  

}
