FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/wifi_lookup/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/wifi_lookup/WifiData.h"
  "../msg_gen/cpp/include/wifi_lookup/Wifi.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
