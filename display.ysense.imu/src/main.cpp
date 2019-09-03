
#include <thread>
#include <vector>
#include <iostream>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/hex.hpp>

#include <glog/logging.h>
#include <gflags/gflags.h>

#include "buffered_async_serial.h"
#include "analysis_data.h"

#include "sensors/imu.h"
#include "options/node-options.hpp"
#include "yesense.imu.hpp"
#include "vslam_integrator/imu_integrator.h"
#include "map_structure/localization_summary_map/localization-summary-map.h"
#include "map_structure/localization_summary_map/localization-summary-map-creation.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(save_map_folder, "",
              "Save map to folder; if empty nothing is saved.");

using namespace std;

int main(int argc, char* argv[])
{	
	FLAGS_max_log_size = 10;
  FLAGS_colorlogtostderr = true;
  FLAGS_stop_logging_if_full_disk = true;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::SetLogDestination(google::GLOG_INFO, "e:\\Logs\\INFO_");
  google::SetStderrLogging(google::GLOG_INFO);
  google::SetLogFilenameExtension("log_");			
	google::ParseCommandLineFlags(&argc, &argv, true);		

	const auto node_options = common::LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  if (node_options.mode == string("Localization")) {
		CHECK(node_options.vio_localization_map_folder.empty());
    // Optionally load localization map.
    std::unique_ptr<summary_map::LocalizationSummaryMap> localization_map;

    localization_map.reset(new summary_map::LocalizationSummaryMap);
    if (!localization_map->LoadFromFolder(node_options.vio_localization_map_folder)) {
      LOG(WARNING) << "Could not load a localization summary map from "
                   << node_options.vio_localization_map_folder
                   << ". Will try to load it as a full VI map.";
      vi_map::VIMap vi_map;
      CHECK(vi_map::serialization::loadMapFromFolder(
          node_options.vio_localization_map_folder, &vi_map))
          << "Loading a VI map failed. Either provide a valid "
             "localization map "
          << "or leave the map folder flag empty.";

      localization_map.reset(new summary_map::LocalizationSummaryMap);
      summary_map::CreateLocalizationSummaryMapForWellConstrainedLandmarks(vi_map, localization_map.get());
      // Make sure the localization map is not empty.
      CHECK_GT(localization_map->GLandmarkPosition().cols(), 0);
    }
  }

	return 0;
}
