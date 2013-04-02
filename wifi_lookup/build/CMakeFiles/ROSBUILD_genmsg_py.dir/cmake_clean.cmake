FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/wifi_lookup/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/wifi_lookup/msg/__init__.py"
  "../src/wifi_lookup/msg/_WifiData.py"
  "../src/wifi_lookup/msg/_Wifi.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
