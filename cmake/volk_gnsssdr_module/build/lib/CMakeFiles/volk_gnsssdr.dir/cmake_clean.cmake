file(REMOVE_RECURSE
  "libvolk_gnsssdr.pdb"
  "libvolk_gnsssdr.so"
  "libvolk_gnsssdr.so.0.0.19"
)

# Per-language clean rules from dependency scanning.
foreach(lang C)
  include(CMakeFiles/volk_gnsssdr.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
